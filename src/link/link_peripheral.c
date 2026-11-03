#include <zmk/nemu_link/link.h>

#include <string.h>

bool nemu_link_link_peripheral_init(struct nemu_link_link_peripheral *p, uint8_t source,
                                    uint32_t nonce, uint64_t now, uint32_t interval_ms,
                                    uint32_t release_ms, uint32_t buttons) {
    if (!p || source >= NEMU_LINK_LINK_SOURCES || !nonce || !interval_ms ||
        interval_ms >= release_ms) {
        return false;
    }
    *p = (struct nemu_link_link_peripheral){
        .phase = NEMU_LINK_LINK_PERIPHERAL_HELLO,
        .source = source,
        .peer_nonce = nonce,
        .confirmed_ms = now,
        .interval_ms = interval_ms,
        .release_ms = release_ms,
        .allowed_buttons = buttons,
    };
    return true;
}

bool nemu_link_link_peripheral_restart(struct nemu_link_link_peripheral *p, uint32_t nonce,
                                       uint64_t now) {
    if (p->phase != NEMU_LINK_LINK_PERIPHERAL_LOST || nonce == p->peer_nonce ||
        now < p->confirmed_ms) {
        return false;
    }
    return nemu_link_link_peripheral_init(p, p->source, nonce, now, p->interval_ms, p->release_ms,
                                          p->allowed_buttons);
}

void nemu_link_link_peripheral_tick(struct nemu_link_link_peripheral *p, uint64_t now) {
    if (p->phase != NEMU_LINK_LINK_PERIPHERAL_LOST && now >= p->confirmed_ms &&
        now - p->confirmed_ms >= p->release_ms) {
        p->phase = NEMU_LINK_LINK_PERIPHERAL_LOST;
        memset(p->sent, 0, sizeof(p->sent));
    }
}

bool nemu_link_link_peripheral_next(struct nemu_link_link_peripheral *p, uint64_t now,
                                    struct nemu_link_link_keys keys, uint32_t buttons,
                                    struct nemu_link_link_message *m) {
    nemu_link_link_peripheral_tick(p, now);
    if (!m || p->phase == NEMU_LINK_LINK_PERIPHERAL_LOST || now < p->confirmed_ms ||
        (p->has_sent && now < p->last_sent_ms) || (buttons & ~p->allowed_buttons)) {
        return false;
    }
    bool changed = p->phase != NEMU_LINK_LINK_PERIPHERAL_HELLO &&
                   (!nemu_link_link_keys_equal(keys, p->last_keys) || buttons != p->last_buttons);
    if (p->has_sent && !changed && now - p->last_sent_ms < p->interval_ms) {
        return false;
    }
    *m = (struct nemu_link_link_message){
        .kind = NEMU_LINK_LINK_HELLO, .source = p->source, .peer_nonce = p->peer_nonce};
    if (p->phase != NEMU_LINK_LINK_PERIPHERAL_HELLO) {
        m->kind = NEMU_LINK_LINK_STATE;
        m->epoch = p->epoch;
        m->session = p->session;
        m->sequence = ++p->sequence;
        m->keys = keys;
        m->buttons = buttons;
        p->sent[p->next_slot] =
            (struct nemu_link_link_sent){.valid = true, .sequence = m->sequence, .sent_ms = now};
        p->next_slot = (p->next_slot + 1) % NEMU_LINK_LINK_ACK_SLOTS;
        p->last_keys = keys;
        p->last_buttons = buttons;
    }
    p->has_sent = true;
    p->last_sent_ms = now;
    return true;
}

bool nemu_link_link_peripheral_receive(struct nemu_link_link_peripheral *p, uint64_t now,
                                       const struct nemu_link_link_message *m) {
    nemu_link_link_peripheral_tick(p, now);
    if (!m || p->phase == NEMU_LINK_LINK_PERIPHERAL_LOST || now < p->confirmed_ms ||
        m->source != p->source || m->peer_nonce != p->peer_nonce || !m->epoch || !m->session ||
        nemu_link_link_keys_any(m->keys) || m->buttons) {
        return false;
    }
    if (m->kind == NEMU_LINK_LINK_CHALLENGE) {
        if (p->phase != NEMU_LINK_LINK_PERIPHERAL_HELLO || !p->has_sent || m->sequence) {
            return false;
        }
        p->epoch = m->epoch;
        p->session = m->session;
        p->phase = NEMU_LINK_LINK_PERIPHERAL_SYNCING;
        p->has_sent = false;
        /* CHALLENGE isn't proof of processed input and doesn't renew the lease. */
        return true;
    }
    if (m->kind != NEMU_LINK_LINK_ACK || p->phase == NEMU_LINK_LINK_PERIPHERAL_HELLO ||
        m->epoch != p->epoch || m->session != p->session ||
        (p->phase == NEMU_LINK_LINK_PERIPHERAL_UP &&
         !nemu_link_link_sequence_after(m->sequence, p->ack_sequence))) {
        return false;
    }
    for (unsigned int i = 0; i < NEMU_LINK_LINK_ACK_SLOTS; ++i) {
        const struct nemu_link_link_sent *sent = &p->sent[i];
        if (!sent->valid || sent->sequence != m->sequence || now < sent->sent_ms ||
            now - sent->sent_ms >= p->release_ms) {
            continue;
        }
        /* Use the original send time, not receipt time. Draining a queue of
         * delayed, monotonically increasing ACKs cannot keep a dead peer alive. */
        p->confirmed_ms = sent->sent_ms;
        p->ack_sequence = m->sequence;
        p->phase = NEMU_LINK_LINK_PERIPHERAL_UP;
        for (unsigned int j = 0; j < NEMU_LINK_LINK_ACK_SLOTS; ++j) {
            if (!nemu_link_link_sequence_after(p->sent[j].sequence, m->sequence)) {
                p->sent[j].valid = false;
            }
        }
        return true;
    }
    return false;
}
