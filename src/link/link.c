#include <zmk/nemu_link/link.h>
#include <zmk/nemu_link/wire.h>

#include <limits.h>
#include <string.h>

static bool valid_message(const struct nemu_link_link_message *m) {
    if (!m || m->source >= NEMU_LINK_LINK_SOURCES || !m->peer_nonce) {
        return false;
    }
    if (m->studio_pending && m->kind != NEMU_LINK_LINK_ACK) {
        return false;
    }
    if ((m->kind == NEMU_LINK_LINK_STATE && m->battery > 100 &&
         m->battery != NEMU_LINK_LINK_BATTERY_UNKNOWN) ||
        (m->kind != NEMU_LINK_LINK_STATE && m->battery)) {
        return false;
    }
    switch (m->kind) {
    case NEMU_LINK_LINK_HELLO:
        return !m->epoch && !m->session && !m->sequence && !nemu_link_link_keys_any(m->keys) &&
               !m->buttons;
    case NEMU_LINK_LINK_CHALLENGE:
        return m->epoch && m->session && !m->sequence && !nemu_link_link_keys_any(m->keys) &&
               !m->buttons;
    case NEMU_LINK_LINK_STATE:
        return m->epoch && m->session;
    case NEMU_LINK_LINK_ACK:
        return m->epoch && m->session && !nemu_link_link_keys_any(m->keys) && !m->buttons;
    default:
        return false;
    }
}

static void put32(uint8_t *p, uint32_t value) {
    for (unsigned int i = 0; i < 4; ++i) {
        p[i] = (uint8_t)(value >> (i * 8));
    }
}

static uint32_t get32(const uint8_t *p) {
    uint32_t value = 0;
    for (unsigned int i = 0; i < 4; ++i) {
        value |= (uint32_t)p[i] << (i * 8);
    }
    return value;
}

bool nemu_link_link_encode(uint8_t *wire, size_t size, const struct nemu_link_link_message *m) {
    return size == NEMU_LINK_LINK_WIRE_SIZE && nemu_link_link_core_encode(wire, size, m);
}

bool nemu_link_link_core_encode(uint8_t *wire, size_t size,
                                const struct nemu_link_link_message *m) {
    if (!wire || !valid_message(m) ||
        (size != NEMU_LINK_LINK_WIRE_SIZE &&
         (size != NEMU_LINK_LINK_CONTROL_WIRE_SIZE || m->kind == NEMU_LINK_LINK_STATE))) {
        return false;
    }
    wire[0] = NEMU_LINK_LINK_VERSION;
    wire[1] = (uint8_t)m->kind;
    wire[2] = m->source;
    wire[3] = m->kind == NEMU_LINK_LINK_ACK && m->studio_pending ? NEMU_LINK_LINK_STUDIO_PENDING
                                                                 : m->battery;
    put32(wire + 4, m->peer_nonce);
    put32(wire + 8, m->epoch);
    put32(wire + 12, m->session);
    put32(wire + 16, m->sequence);
    if (size == NEMU_LINK_LINK_WIRE_SIZE) {
        for (unsigned int i = 0; i < NEMU_LINK_LINK_KEY_WORDS; i++) {
            put32(wire + 20 + i * 8, (uint32_t)m->keys.words[i]);
            put32(wire + 24 + i * 8, (uint32_t)(m->keys.words[i] >> 32));
        }
        put32(wire + 20 + NEMU_LINK_LINK_KEY_BYTES, m->buttons);
    }
    return true;
}

bool nemu_link_link_decode(struct nemu_link_link_message *m, const uint8_t *wire, size_t size) {
    return size == NEMU_LINK_LINK_WIRE_SIZE && nemu_link_link_core_decode(m, wire, size);
}

bool nemu_link_link_core_decode(struct nemu_link_link_message *m, const uint8_t *wire,
                                size_t size) {
    if (!m || !wire ||
        (size != NEMU_LINK_LINK_WIRE_SIZE && size != NEMU_LINK_LINK_CONTROL_WIRE_SIZE) ||
        wire[0] != NEMU_LINK_LINK_VERSION ||
        (size == NEMU_LINK_LINK_CONTROL_WIRE_SIZE && wire[1] == NEMU_LINK_LINK_STATE)) {
        return false;
    }
    struct nemu_link_link_message decoded = {
        .kind = (enum nemu_link_link_kind)wire[1],
        .source = wire[2],
        .battery = wire[1] == NEMU_LINK_LINK_ACK ? 0 : wire[3],
        .studio_pending =
            wire[1] == NEMU_LINK_LINK_ACK && (wire[3] & NEMU_LINK_LINK_STUDIO_PENDING),
        .peer_nonce = get32(wire + 4),
        .epoch = get32(wire + 8),
        .session = get32(wire + 12),
        .sequence = get32(wire + 16),
        .buttons =
            size == NEMU_LINK_LINK_WIRE_SIZE ? get32(wire + 20 + NEMU_LINK_LINK_KEY_BYTES) : 0,
    };
    if (size == NEMU_LINK_LINK_WIRE_SIZE) {
        for (unsigned int i = 0; i < NEMU_LINK_LINK_KEY_WORDS; i++) {
            decoded.keys.words[i] =
                (uint64_t)get32(wire + 20 + i * 8) | ((uint64_t)get32(wire + 24 + i * 8) << 32);
        }
    }
    if ((decoded.kind == NEMU_LINK_LINK_ACK && (wire[3] & ~NEMU_LINK_LINK_STUDIO_PENDING)) ||
        !valid_message(&decoded)) {
        return false;
    }
    *m = decoded;
    return true;
}

bool nemu_link_link_sequence_after(uint32_t candidate, uint32_t previous) {
    uint32_t distance = candidate - previous;
    return distance != 0 && distance < UINT32_C(0x80000000);
}
