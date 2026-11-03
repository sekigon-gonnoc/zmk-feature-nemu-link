#include <errno.h>
#include <string.h>
#include <mbedtls/platform_util.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/nemu_link/mini.h>
#include <zmk/nemu_link/security_role.h>
#include <zmk/nemu_link/nrf_ccm.h>
#include <zmk/nemu_link/power_trace.h>
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
#include <zmk/nemu_link/ccm_radio.h>
#endif

bool nemu_link_mini_wire(size_t size) {
    return size == NEMU_LINK_MINI_HEARTBEAT_SIZE || size == NEMU_LINK_MINI_ACK_SIZE;
}

bool nemu_link_mini_forward(uint32_t previous, uint16_t low, uint32_t *full) {
    uint32_t delta = (uint16_t)(low - (uint16_t)previous);
    if (!previous || !delta || delta > NEMU_LINK_MINI_FORWARD_WINDOW ||
        previous >= NEMU_LINK_COUNTER_REKEY_AT || delta >= NEMU_LINK_COUNTER_REKEY_AT - previous) {
        return false;
    }
    *full = previous + delta;
    return true;
}

bool nemu_link_mini_ack_counter(uint32_t reference, uint16_t low, uint32_t *full) {
    uint32_t age = (uint16_t)((uint16_t)reference - low);
    if (!reference || age > NEMU_LINK_MINI_FORWARD_WINDOW || age >= reference) {
        return false;
    }
    *full = reference - age;
    return true;
}

bool nemu_link_mini_down_forward(uint32_t previous, uint16_t wire, uint32_t *full) {
    uint32_t delta =
        ((wire & NEMU_LINK_MINI_COUNTER_MASK) - (previous & NEMU_LINK_MINI_COUNTER_MASK)) &
        NEMU_LINK_MINI_COUNTER_MASK;
    if (!previous || !delta || delta > NEMU_LINK_MINI_DOWN_WINDOW ||
        previous >= NEMU_LINK_COUNTER_REKEY_AT || delta >= NEMU_LINK_COUNTER_REKEY_AT - previous)
        return false;
    *full = previous + delta;
    return true;
}

bool nemu_link_mini_ack_high(uint32_t reference, uint16_t wire, uint32_t *high) {
    if (!reference)
        return false;
    uint32_t next = reference >> NEMU_LINK_MINI_COUNTER_BITS;
    if ((next & 1) != (wire >> NEMU_LINK_MINI_ACK_HIGH_BIT)) {
        if (!next)
            return false;
        next--;
    }
    *high = next;
    return true;
}

static struct nemu_link_mini_metadata metadata(const struct nemu_link_link_message *m,
                                               uint32_t mode,
                                               const struct nemu_link_hop_notice *h) {
    return (struct nemu_link_mini_metadata){.epoch = m->epoch,
                                            .session = m->session,
                                            .peer_nonce = m->peer_nonce,
                                            .hop_epoch = h->epoch,
                                            .mode = mode,
                                            .hop_index = h->index,
                                            .dongle = h->dongle_id,
                                            .source = m->source};
}

static bool equal(const struct nemu_link_mini_metadata *a,
                  const struct nemu_link_mini_metadata *b) {
    return a->epoch == b->epoch && a->session == b->session && a->peer_nonce == b->peer_nonce &&
           a->hop_epoch == b->hop_epoch && a->mode == b->mode && a->hop_index == b->hop_index &&
           a->dongle == b->dongle && a->source == b->source;
}

/* Canonical HMAC transcript for the Nemu-link mini v1 protocol. */
enum mini_context_layout {
    MINI_CONTEXT_LABEL_SIZE = sizeof("NEMU-LINK-mini-v1") - 1,
    MINI_CONTEXT_IV_OFFSET = MINI_CONTEXT_LABEL_SIZE,
    MINI_CONTEXT_EPOCH_OFFSET = MINI_CONTEXT_IV_OFFSET + NEMU_LINK_CCM_IV_SIZE,
    MINI_CONTEXT_SESSION_OFFSET = MINI_CONTEXT_EPOCH_OFFSET + NEMU_LINK_COUNTER_SIZE,
    MINI_CONTEXT_NONCE_OFFSET = MINI_CONTEXT_SESSION_OFFSET + NEMU_LINK_COUNTER_SIZE,
    MINI_CONTEXT_HOP_OFFSET = MINI_CONTEXT_NONCE_OFFSET + NEMU_LINK_COUNTER_SIZE,
    MINI_CONTEXT_MODE_OFFSET = MINI_CONTEXT_HOP_OFFSET + NEMU_LINK_COUNTER_SIZE,
    MINI_CONTEXT_INDEX_OFFSET = MINI_CONTEXT_MODE_OFFSET + NEMU_LINK_COUNTER_SIZE,
    MINI_CONTEXT_DONGLE_OFFSET,
    MINI_CONTEXT_SOURCE_OFFSET,
    MINI_CONTEXT_RESERVED_OFFSET,
    MINI_CONTEXT_RESERVED_SIZE = 5,
    MINI_CONTEXT_SIZE = MINI_CONTEXT_RESERVED_OFFSET + MINI_CONTEXT_RESERVED_SIZE,
};

static int context_derive(struct nemu_link_secure_peer *peer) {
    /* New key and IV for every agreed metadata tuple; stale short packets
     * cannot confirm a newer hop/mode merely by filling fields from a cache. */
    uint8_t input[MINI_CONTEXT_SIZE] = {0}, digest[NEMU_LINK_SHA256_DIGEST_SIZE];
    memcpy(input, "NEMU-LINK-mini-v1", MINI_CONTEXT_LABEL_SIZE);
    memcpy(input + MINI_CONTEXT_IV_OFFSET, peer->nonce_prefix, NEMU_LINK_CCM_IV_SIZE);
    const struct nemu_link_mini_metadata *m = &peer->mini.tx;
    sys_put_le32(m->epoch, input + MINI_CONTEXT_EPOCH_OFFSET);
    sys_put_le32(m->session, input + MINI_CONTEXT_SESSION_OFFSET);
    sys_put_le32(m->peer_nonce, input + MINI_CONTEXT_NONCE_OFFSET);
    sys_put_le32(m->hop_epoch, input + MINI_CONTEXT_HOP_OFFSET);
    sys_put_le32(m->mode, input + MINI_CONTEXT_MODE_OFFSET);
    input[MINI_CONTEXT_INDEX_OFFSET] = m->hop_index;
    input[MINI_CONTEXT_DONGLE_OFFSET] = m->dongle;
    input[MINI_CONTEXT_SOURCE_OFFSET] = m->source;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attr, NEMU_LINK_AES_KEY_SIZE * NEMU_LINK_RADIO_BITS_PER_BYTE);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_key_id_t key = 0;
    psa_status_t rc = psa_import_key(&attr, peer->ccm_key, sizeof(peer->ccm_key), &key);
    psa_reset_key_attributes(&attr);
    size_t n = 0;
    if (rc == PSA_SUCCESS) {
        rc = psa_mac_compute(key, PSA_ALG_HMAC(PSA_ALG_SHA_256), input, sizeof(input), digest,
                             sizeof(digest), &n);
        psa_destroy_key(key);
    }
    if (rc == PSA_SUCCESS && n == sizeof(digest)) {
        memcpy(peer->mini.key, digest, NEMU_LINK_AES_KEY_SIZE);
        memcpy(peer->mini.iv, digest + NEMU_LINK_AES_KEY_SIZE, NEMU_LINK_CCM_IV_SIZE);
    }
    mbedtls_platform_zeroize(input, sizeof(input));
    mbedtls_platform_zeroize(digest, sizeof(digest));
    return rc == PSA_SUCCESS && n == sizeof(digest) ? 0 : -EIO;
}

void nemu_link_mini_note(struct nemu_link_secure_peer *peer, bool tx,
                         const struct nemu_link_link_message *m, uint32_t mode,
                         const struct nemu_link_hop_notice *h) {
    if (!nemu_link_secure_ready(peer) || h->kind != NEMU_LINK_HOP_STABLE ||
        (m->kind != NEMU_LINK_LINK_STATE && m->kind != NEMU_LINK_LINK_ACK) || !m->epoch ||
        !m->session || !m->peer_nonce) {
        peer->mini.valid = false;
        peer->mini.have_tx = peer->mini.have_rx = false;
        return;
    }
    struct nemu_link_mini_metadata next = metadata(m, mode, h);
    if (!tx && m->kind == NEMU_LINK_LINK_STATE) {
        peer->mini.battery = m->battery;
    }
    if (tx) {
        peer->mini.tx = next;
        peer->mini.have_tx = true;
        if (m->kind == NEMU_LINK_LINK_STATE) {
            peer->mini.tx_battery = m->battery;
        }
    } else {
        peer->mini.rx = next;
        peer->mini.have_rx = true;
    }
    bool agreed = peer->mini.have_tx && peer->mini.have_rx && equal(&peer->mini.tx, &peer->mini.rx);
    /* Derive only when becoming agreed; unchanged periodic full frames are cheap. */
    if (!agreed) {
        peer->mini.valid = false;
    } else if (!peer->mini.valid) {
        peer->mini.valid = context_derive(peer) == 0;
    }
}

bool nemu_link_mini_can_send(const struct nemu_link_secure_peer *peer,
                             const struct nemu_link_link_message *m, uint32_t mode,
                             const struct nemu_link_hop_notice *h) {
    if (!peer->mini.valid || h->kind != NEMU_LINK_HOP_STABLE ||
        (m->kind == NEMU_LINK_LINK_STATE && m->battery != peer->mini.tx_battery) ||
        (m->kind != NEMU_LINK_LINK_ACK &&
         (m->kind != NEMU_LINK_LINK_STATE || nemu_link_link_keys_any(m->keys) || m->buttons))) {
        return false;
    }
    struct nemu_link_mini_metadata next = metadata(m, mode, h);
    return equal(&next, &peer->mini.tx);
}

static void nonce(const struct nemu_link_secure_peer *peer, uint8_t source, bool central,
                  uint32_t counter, uint8_t out[NEMU_LINK_CCM_NONCE_SIZE]) {
    sys_put_le32(counter, out);
    out[NEMU_LINK_CCM_DIRECTION_OFFSET] = source | (central ? NEMU_LINK_CCM_DIRECTION_BIT : 0);
    memcpy(out + NEMU_LINK_CCM_COUNTER_SIZE, peer->mini.iv, NEMU_LINK_CCM_IV_SIZE);
}

int nemu_link_mini_wrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                        const struct nemu_link_link_message *m,
                        uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *size) {
    central = NEMU_LINK_SECURITY_CENTRAL(central);
    if (!nemu_link_secure_ready(peer) || !peer->mini.valid || source != peer->mini.tx.source ||
        peer->tx_sequence >= NEMU_LINK_COUNTER_REKEY_AT ||
        (central ? m->kind != NEMU_LINK_LINK_ACK : m->kind != NEMU_LINK_LINK_STATE) ||
        nemu_link_link_keys_any(m->keys) || m->buttons || !m->sequence ||
        (!central && m->studio_pending)) {
        return -EINVAL;
    }
    uint32_t sequence = ++peer->tx_sequence;
    uint8_t plain[NEMU_LINK_MINI_ACK_PLAIN_SIZE] = {central ? NEMU_LINK_MINI_ACK_KIND
                                                            : NEMU_LINK_MINI_EMPTY_KIND},
            iv[NEMU_LINK_CCM_NONCE_SIZE];
    size_t n = central ? NEMU_LINK_MINI_ACK_PLAIN_SIZE : NEMU_LINK_MINI_HEARTBEAT_PLAIN_SIZE;
    if (central) {
        if (m->studio_pending) {
            plain[0] |= NEMU_LINK_MINI_STUDIO_PENDING;
        }
        sys_put_le16((uint16_t)m->sequence, plain + NEMU_LINK_MINI_KIND_SIZE);
    } else if (m->sequence != sequence) {
        return -EINVAL;
    }
    uint16_t wire_counter = central ? (uint16_t)((sequence & NEMU_LINK_MINI_COUNTER_MASK) |
                                                 ((m->sequence >> NEMU_LINK_MINI_COUNTER_BITS & 1)
                                                  << NEMU_LINK_MINI_ACK_HIGH_BIT))
                                    : (uint16_t)sequence;
    sys_put_le16(wire_counter, out);
    nonce(peer, source, central, sequence, iv);
    if (central) {
        /* Low ACK bits are encrypted; bind their omitted high bits too.
         * A delayed ACK must never confirm a newer STATE after a 16-bit wrap. */
        iv[NEMU_LINK_CCM_COUNTER_SIZE] ^= (uint8_t)(m->sequence >> NEMU_LINK_MINI_COUNTER_BITS);
        iv[NEMU_LINK_CCM_COUNTER_SIZE + 1] ^=
            (uint8_t)(m->sequence >> (NEMU_LINK_MINI_COUNTER_BITS + NEMU_LINK_RADIO_BITS_PER_BYTE));
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCRYPT_BEGIN, n);
    int rc;
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    if (!central) {
        rc = nemu_link_ccm_radio_queue(source, peer->mini.key, iv, plain, n, out,
                                       NEMU_LINK_MINI_COUNTER_SIZE);
    } else
#endif
    {
        rc = nemu_link_nrf_ccm_crypt(false, peer->mini.key, iv, plain, n,
                                     out + NEMU_LINK_MINI_COUNTER_SIZE,
                                     NEMU_LINK_FRAME_MAX_SIZE - NEMU_LINK_MINI_COUNTER_SIZE);
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCRYPT_END, (uint16_t)-rc);
    mbedtls_platform_zeroize(iv, sizeof(iv));
    mbedtls_platform_zeroize(plain, sizeof(plain));
    if (!rc) {
        *size = n + NEMU_LINK_MINI_COUNTER_SIZE + NEMU_LINK_CCM_MIC_SIZE;
    }
    return rc;
}

int nemu_link_mini_unwrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                          const uint8_t *wire, size_t size, uint32_t epoch, uint32_t session,
                          uint32_t peer_nonce, uint32_t ack_reference,
                          struct nemu_link_link_message *m, uint32_t *mode,
                          struct nemu_link_hop_notice *h) {
    central = NEMU_LINK_SECURITY_CENTRAL(central);
    uint32_t sequence;
    if (!nemu_link_secure_ready(peer) || !peer->mini.valid ||
        size != (central ? NEMU_LINK_MINI_HEARTBEAT_SIZE : NEMU_LINK_MINI_ACK_SIZE) ||
        source != peer->mini.rx.source || epoch != peer->mini.rx.epoch ||
        session != peer->mini.rx.session || peer_nonce != peer->mini.rx.peer_nonce) {
        return -EINVAL;
    }
    uint16_t counter = sys_get_le16(wire);
    bool fresh = central ? nemu_link_mini_forward(peer->rx_sequence, counter, &sequence)
                         : nemu_link_mini_down_forward(peer->rx_sequence, counter, &sequence);
    if (!fresh)
        return -EALREADY;
    uint8_t plain[NEMU_LINK_MINI_ACK_PLAIN_SIZE], iv[NEMU_LINK_CCM_NONCE_SIZE];
    uint32_t acknowledged = 0, high = 0;
    if (!central && !nemu_link_mini_ack_high(ack_reference, counter, &high))
        return -EINVAL;
    nonce(peer, source, !central, sequence, iv);
    if (!central) {
        iv[NEMU_LINK_CCM_COUNTER_SIZE] ^= (uint8_t)high;
        iv[NEMU_LINK_CCM_COUNTER_SIZE + 1] ^= (uint8_t)(high >> 8);
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_DECRYPT_BEGIN, size);
    int rc;
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    rc = nemu_link_ccm_radio_rx_take(source, peer->mini.key, iv, wire, size, plain, sizeof(plain));
    if (rc == -ENOENT)
#endif
    {
        rc = nemu_link_nrf_ccm_crypt(true, peer->mini.key, iv, wire + NEMU_LINK_MINI_COUNTER_SIZE,
                                     size - NEMU_LINK_MINI_COUNTER_SIZE, plain, sizeof(plain));
    }
    if (rc == -EACCES && ++peer->ccm_failures >= NEMU_LINK_CCM_FAILURE_LIMIT)
        nemu_link_secure_reset(peer);
    bool valid = !rc && (central ? plain[0] == NEMU_LINK_MINI_EMPTY_KIND
                                 : (plain[0] & (uint8_t)~NEMU_LINK_MINI_STUDIO_PENDING) ==
                                       NEMU_LINK_MINI_ACK_KIND);
    if (valid && !central) {
        valid = nemu_link_mini_ack_counter(
                    ack_reference, sys_get_le16(plain + NEMU_LINK_MINI_KIND_SIZE), &acknowledged) &&
                (acknowledged >> NEMU_LINK_MINI_COUNTER_BITS) == high;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_DECRYPT_END, (uint16_t)-rc);
    mbedtls_platform_zeroize(iv, sizeof(iv));
    if (rc) {
        mbedtls_platform_zeroize(plain, sizeof(plain));
        return rc;
    }
    if (valid) {
        *m = (struct nemu_link_link_message){
            .kind = central ? NEMU_LINK_LINK_STATE : NEMU_LINK_LINK_ACK,
            .source = source,
            .epoch = epoch,
            .session = session,
            .peer_nonce = peer_nonce,
            .sequence = central ? sequence : acknowledged,
            .battery = central ? peer->mini.battery : 0,
            .studio_pending = !central && (plain[0] & NEMU_LINK_MINI_STUDIO_PENDING)};
        *h = (struct nemu_link_hop_notice){.kind = NEMU_LINK_HOP_STABLE,
                                           .epoch = peer->mini.rx.hop_epoch,
                                           .index = peer->mini.rx.hop_index,
                                           .dongle_id = peer->mini.rx.dongle};
        *mode = peer->mini.rx.mode;
        peer->rx_sequence = sequence;
    }
    mbedtls_platform_zeroize(plain, sizeof(plain));
    return valid ? 0 : -EINVAL;
}
