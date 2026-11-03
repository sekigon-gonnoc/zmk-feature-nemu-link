#include <zmk/nemu_link/hop.h>
#include <stddef.h>
#include <string.h>

static const uint8_t default_channels[NEMU_LINK_HOP_CHANNEL_COUNT] =
    NEMU_LINK_DEFAULT_RADIO_CHANNELS;
uint8_t nemu_link_hop_channels[NEMU_LINK_HOP_CHANNEL_COUNT] = NEMU_LINK_DEFAULT_RADIO_CHANNELS;
uint8_t nemu_link_hop_channel_count = NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT;

bool nemu_link_hop_set_channels(const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                uint8_t channel_count) {
    if (!channels || !channel_count || channel_count > NEMU_LINK_HOP_CHANNEL_COUNT) {
        return false;
    }
    for (uint8_t i = 0; i < channel_count; i++) {
        if (channels[i] > NEMU_LINK_RADIO_CHANNEL_MAX || (channels[i] & 1U)) {
            return false;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (channels[i] == channels[j]) {
                return false;
            }
        }
    }
    memcpy(nemu_link_hop_channels, channels, channel_count);
    for (uint8_t i = channel_count; i < NEMU_LINK_HOP_CHANNEL_COUNT; i++) {
        nemu_link_hop_channels[i] = default_channels[i];
    }
    nemu_link_hop_channel_count = channel_count;
    return true;
}

static uint8_t following(uint8_t index) {
    return (uint8_t)((index + 1U) % nemu_link_hop_channel_count);
}
bool nemu_link_hop_timing_valid(const struct nemu_link_hop_timing *t) {
    return t && t->probe_ms >= 10 && t->loss_ms && t->announce_ms &&
           (uint64_t)t->peer_dwell_ms >= 3ULL * t->probe_ms + 20 &&
           (uint64_t)t->central_dwell_ms >=
               ((uint64_t)nemu_link_hop_channel_count + 1ULL) * t->peer_dwell_ms + 20 &&
           t->loss_ms > t->central_dwell_ms &&
           (uint64_t)t->announce_ms >= NEMU_LINK_HOP_COMMIT_PROBES * (uint64_t)t->probe_ms &&
           t->loss_ms > NEMU_LINK_HOP_FOLLOW_PROBES * (uint64_t)t->probe_ms;
}
bool nemu_link_hop_notice_valid(const struct nemu_link_hop_notice *n) {
    return n && n->index < nemu_link_hop_channel_count && n->kind >= NEMU_LINK_HOP_STABLE &&
           n->kind <= NEMU_LINK_HOP_COMMIT && (n->epoch || n->kind == NEMU_LINK_HOP_PROBE);
}
bool nemu_link_hop_peripheral_init(struct nemu_link_hop_peripheral *p, uint64_t now,
                                   const struct nemu_link_hop_timing *t) {
    if (!p || !nemu_link_hop_timing_valid(t)) {
        return false;
    }
    *p = (struct nemu_link_hop_peripheral){
        .timing = *t, .searching = true, .heard_ms = now, .changed_ms = now};
    return true;
}
bool nemu_link_hop_peripheral_bind(struct nemu_link_hop_peripheral *p, uint32_t boot) {
    if (!boot) {
        return false;
    }
    if (boot != p->boot) {
        p->boot = boot;
        p->epoch = 0;
        p->pending = false;
        p->committed = false;
        p->searching = true;
    }
    return true;
}
bool nemu_link_hop_peripheral_receive(struct nemu_link_hop_peripheral *p, uint32_t boot,
                                      uint64_t now, const struct nemu_link_hop_notice *n) {
    if (!boot || boot != p->boot || !nemu_link_hop_notice_valid(n) ||
        (n->kind != NEMU_LINK_HOP_STABLE && n->kind != NEMU_LINK_HOP_PREPARE &&
         n->kind != NEMU_LINK_HOP_COMMIT) ||
        n->epoch < p->epoch) {
        return false;
    }
    if (n->kind == NEMU_LINK_HOP_PREPARE || n->kind == NEMU_LINK_HOP_COMMIT) {
        /* A peer discovering an in-progress announcement does not know its
         * old generation yet. Bind it from the validated proposal. */
        if (n->epoch < 2 || n->index == p->index ||
            (p->epoch &&
             (p->epoch == UINT32_MAX || n->epoch != p->epoch + 1 || p->index != p->known_index)) ||
            (p->pending && n->index != p->pending_index) ||
            (n->kind == NEMU_LINK_HOP_COMMIT && !p->pending)) {
            return false;
        }
        p->epoch = n->epoch - 1;
        p->known_index = p->index;
        p->pending = true;
        p->pending_index = n->index;
        p->committed |= n->kind == NEMU_LINK_HOP_COMMIT;
    } else {
        if (n->index != p->index || (n->epoch == p->epoch && n->index != p->known_index) ||
            (p->pending && n->epoch == p->epoch)) {
            return false;
        }
        p->epoch = n->epoch;
        p->known_index = n->index;
        p->pending = false;
        p->committed = false;
    }
    p->heard_ms = now;
    p->searching = false;
    return true;
}
struct nemu_link_hop_notice
nemu_link_hop_peripheral_notice(const struct nemu_link_hop_peripheral *p) {
    if (p->searching || !p->epoch) {
        return (struct nemu_link_hop_notice){
            .epoch = p->epoch, .index = p->index, .kind = NEMU_LINK_HOP_PROBE};
    }
    return (struct nemu_link_hop_notice){.epoch = p->epoch + (p->pending ? 1U : 0U),
                                         .index = p->pending ? p->pending_index : p->known_index,
                                         .kind = p->pending ? NEMU_LINK_HOP_ECHO
                                                            : NEMU_LINK_HOP_STABLE};
}
static bool lost(const struct nemu_link_hop_peripheral *p, uint64_t now) {
    uint32_t timeout =
        p->committed ? NEMU_LINK_HOP_FOLLOW_PROBES * p->timing.probe_ms : p->timing.loss_ms;
    return now - p->heard_ms >= timeout;
}
bool nemu_link_hop_peripheral_next(const struct nemu_link_hop_peripheral *p, uint64_t now,
                                   uint8_t *index) {
    if (!index || (!p->searching && !lost(p, now)) ||
        (p->searching && now - p->changed_ms < p->timing.peer_dwell_ms)) {
        return false;
    }
    *index = p->pending && !p->searching ? p->pending_index : following(p->index);
    return true;
}
bool nemu_link_hop_peripheral_applied(struct nemu_link_hop_peripheral *p, uint64_t now,
                                      uint8_t index) {
    uint8_t expected;
    if (!nemu_link_hop_peripheral_next(p, now, &expected) || index != expected) {
        return false;
    }
    p->index = index;
    p->changed_ms = now;
    p->searching = true;
    p->pending = false;
    p->committed = false;
    p->has_probed = false;
    return true;
}
bool nemu_link_hop_probe_due(const struct nemu_link_hop_peripheral *p, uint64_t now) {
    return (p->searching || lost(p, now)) &&
           (!p->has_probed || now - p->probed_ms >= p->timing.probe_ms);
}
void nemu_link_hop_probe_sent(struct nemu_link_hop_peripheral *p, uint64_t now) {
    p->has_probed = true;
    p->probed_ms = now;
}
bool nemu_link_hop_idle_allowed(const struct nemu_link_hop_peripheral *p, uint64_t now) {
    (void)now;
    /* PTX receives only during its own transaction. Discovery/HELLO therefore
     * needs no continuous RX or HFCLK between probes. Keep pending moves awake. */
    return !p->pending;
}
