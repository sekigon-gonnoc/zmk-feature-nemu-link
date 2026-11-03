#include <zmk/nemu_link/compact.h>
#include <zmk/nemu_link/hop_frame.h>
#include <zmk/nemu_link/selection.h>
#include <zmk/nemu_link/split_frame.h>
#include <limits.h>
#include <string.h>

#define COMPACT_EMPTY NEMU_LINK_WIRE_EMPTY
#define COMPACT_ACK NEMU_LINK_WIRE_ACK
#define COMPACT_STATE NEMU_LINK_WIRE_STATE
#define COMPACT_INPUT_VAR NEMU_LINK_WIRE_INPUT_VAR
#define COMPACT_SENSOR_VAR NEMU_LINK_WIRE_SENSOR_VAR
#define COMPACT_SENSOR_FIXED NEMU_LINK_WIRE_SENSOR_FIXED
#define COMPACT_INPUT_FIXED NEMU_LINK_WIRE_INPUT_FIXED
#define COMPACT_INPUT_EVENT 2
#define COMPACT_SENSOR_EVENT 1

/* Native split records: type/device header, then ordered input records or
 * three int32 sensor fields. Compact fixed input folds sync/type into a byte. */
enum compact_layout {
    SPLIT_EVENT_HEADER_SIZE = 2,
    INPUT_RECORD_SIZE = 8,
    SENSOR_FIELD_COUNT = 3,
    SENSOR_EVENT_SIZE = SPLIT_EVENT_HEADER_SIZE + SENSOR_FIELD_COUNT * sizeof(int32_t),
    SENSOR_COMPACT_FIXED_SIZE = SENSOR_EVENT_SIZE - 1,
    KEY_BITMAP_MARKER = 15,
    KEY_WORD_BYTES = sizeof(uint64_t),
    KEY_WORD_BITS = KEY_WORD_BYTES * NEMU_LINK_RADIO_BITS_PER_BYTE,
};

struct buffer {
    uint8_t data[NEMU_LINK_COMPACT_BODY_MAX];
    size_t at;
    bool error;
};
struct reader {
    const uint8_t *data;
    size_t at, size;
    bool error;
};
static void byte(struct buffer *b, uint8_t v) {
    if (b->at == sizeof(b->data)) {
        b->error = true;
    } else {
        b->data[b->at++] = v;
    }
}
static uint8_t read_byte(struct reader *r) {
    if (r->at == r->size) {
        r->error = true;
        return 0;
    }
    return r->data[r->at++];
}
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void put32(uint8_t *p, uint32_t n) {
    for (unsigned i = 0; i < 4; i++) {
        p[i] = (uint8_t)(n >> (8 * i));
    }
}
static void word(struct buffer *b, uint32_t n) {
    for (unsigned i = 0; i < 4; i++) {
        byte(b, (uint8_t)(n >> (8 * i)));
    }
}
static uint32_t read_word(struct reader *r) {
    uint32_t n = 0;
    for (unsigned i = 0; i < 4; i++) {
        n |= (uint32_t)read_byte(r) << (8 * i);
    }
    return n;
}
static void var(struct buffer *b, uint32_t n) {
    do {
        byte(b, (uint8_t)((n & 127) | (n > 127 ? 128 : 0)));
        n >>= 7;
    } while (n);
}
static uint32_t read_var(struct reader *r) {
    uint32_t n = 0;
    for (unsigned i = 0; i < 5; i++) {
        uint8_t v = read_byte(r);
        if (r->error || (i == 4 && v > 15)) {
            r->error = true;
            return 0;
        }
        n |= (uint32_t)(v & 127) << (7 * i);
        if (!(v & 128)) {
            if (i && !v) {
                r->error = true; /* Reject non-canonical integer encodings. */
            }
            return n;
        }
    }
    r->error = true;
    return 0;
}
static uint32_t zig(uint32_t n) { return (n << 1) ^ (n & UINT32_C(0x80000000) ? UINT32_MAX : 0); }
static uint32_t unzig(uint32_t n) { return (n >> 1) ^ (0U - (n & 1)); }
static void mode_word(struct buffer *b, uint32_t token) {
    struct nemu_link_selection s;
    if (!token) {
        byte(b, 0);
    } else if (nemu_link_selection_decode(token, &s)) {
        byte(b, 1);
        var(b, (s.generation << NEMU_LINK_SELECTION_TARGET_BITS) | (uint32_t)(s.target - 1));
    } else {
        byte(b, 2);
        word(b, token);
    }
}
static uint32_t read_mode(struct reader *r) {
    switch (read_byte(r)) {
    case 0:
        return 0;
    case 1: {
        uint32_t n = read_var(r);
        if (n > UINT32_C(0x00ffffff)) {
            r->error = true;
        }
        return NEMU_LINK_SELECTION_PREFIX | n;
    }
    case 2:
        return read_word(r);
    default:
        r->error = true;
        return 0;
    }
}

bool nemu_link_compact_kind(uint8_t kind) {
    return kind >= COMPACT_EMPTY && kind <= COMPACT_INPUT_FIXED;
}

bool nemu_link_compact_pack(uint8_t source, bool link_allowed,
                            const uint8_t native[NEMU_LINK_FRAME_MAX_SIZE], size_t size,
                            uint8_t *kind, uint8_t body[NEMU_LINK_COMPACT_BODY_MAX],
                            size_t *body_size) {
    struct buffer b = {0};
    struct nemu_link_link_message m;
    struct nemu_link_hop_notice h;
    struct nemu_link_split_frame f;
    uint32_t mode;
    if (link_allowed && nemu_link_hop_frame_decode(&m, &mode, &h, source, native, size, NULL) &&
        h.kind == NEMU_LINK_HOP_STABLE &&
        (m.kind == NEMU_LINK_LINK_STATE || m.kind == NEMU_LINK_LINK_ACK)) {
        unsigned count = (unsigned)__builtin_popcountll(m.keys.words[0]) +
                         (unsigned)__builtin_popcountll(m.keys.words[1]);
        bool bitmap = count >= KEY_BITMAP_MARKER;
        *kind = m.kind == NEMU_LINK_LINK_ACK ? COMPACT_ACK
                : count || m.buttons         ? COMPACT_STATE
                                             : COMPACT_EMPTY;
        word(&b, m.sequence);
        var(&b, h.epoch);
        byte(&b, h.index);
        byte(&b, (uint8_t)(((bitmap ? KEY_BITMAP_MARKER : count) << 4) | (m.buttons ? 8 : 0) |
                           (m.studio_pending ? NEMU_LINK_COMPACT_STUDIO_PENDING : 0)));
        mode_word(&b, mode);
        if (m.kind == NEMU_LINK_LINK_STATE) {
            byte(&b, m.battery);
            if (bitmap) {
                for (unsigned i = 0; i < NEMU_LINK_LINK_KEY_BYTES; i++) {
                    byte(&b, (uint8_t)(m.keys.words[i / KEY_WORD_BYTES] >>
                                       (NEMU_LINK_RADIO_BITS_PER_BYTE * (i % KEY_WORD_BYTES))));
                }
            } else if (count) {
                for (unsigned w = 0; w < NEMU_LINK_LINK_KEY_WORDS; w++) {
                    uint64_t bits = m.keys.words[w];
                    while (bits) {
                        unsigned p = (unsigned)__builtin_ctzll(bits);
                        byte(&b, (uint8_t)(KEY_WORD_BITS * w + p));
                        bits &= bits - 1;
                    }
                }
            }
            if (m.buttons) {
                word(&b, m.buttons);
            }
        }
    } else if (nemu_link_split_decode(&f, native, size, NULL) && f.source == source &&
               f.kind == NEMU_LINK_SPLIT_EVENT && f.data[0] == COMPACT_INPUT_EVENT &&
               f.size >= SPLIT_EVENT_HEADER_SIZE + INPUT_RECORD_SIZE &&
               (f.size - SPLIT_EVENT_HEADER_SIZE) % INPUT_RECORD_SIZE == 0) {
        struct buffer fixed = {0};
        byte(&b, f.data[1]);
        byte(&fixed, f.data[1]);
        for (unsigned i = SPLIT_EVENT_HEADER_SIZE; i < f.size; i += INPUT_RECORD_SIZE) {
            if (f.data[i] > 1 || f.data[i + 1] > 127) {
                return false;
            }
            uint8_t flags = f.data[i + 1] | (f.data[i] ? 128 : 0);
            byte(&b, flags);
            var(&b, (uint32_t)f.data[i + 2] | (uint32_t)f.data[i + 3] << 8);
            var(&b, zig(le32(f.data + i + 4)));
            byte(&fixed, flags);
            for (unsigned j = SPLIT_EVENT_HEADER_SIZE; j < INPUT_RECORD_SIZE; j++) {
                byte(&fixed, f.data[i + j]);
            }
        }
        *kind = COMPACT_INPUT_VAR;
        if (b.at > fixed.at) {
            b = fixed;
            *kind = COMPACT_INPUT_FIXED;
        }
    } else if (nemu_link_split_decode(&f, native, size, NULL) && f.source == source &&
               f.kind == NEMU_LINK_SPLIT_EVENT && f.size == SENSOR_EVENT_SIZE &&
               f.data[0] == COMPACT_SENSOR_EVENT) {
        byte(&b, f.data[1]);
        for (unsigned i = SPLIT_EVENT_HEADER_SIZE; i < SENSOR_EVENT_SIZE; i += sizeof(int32_t)) {
            var(&b, zig(le32(f.data + i)));
        }
        *kind = COMPACT_SENSOR_VAR;
        if (b.at > SENSOR_COMPACT_FIXED_SIZE) {
            b = (struct buffer){0};
            for (unsigned i = 1; i < SENSOR_EVENT_SIZE; i++) {
                byte(&b, f.data[i]);
            }
            *kind = COMPACT_SENSOR_FIXED;
        }
    } else {
        return false;
    }
    if (b.error) {
        return false;
    }
    memcpy(body, b.data, b.at);
    *body_size = b.at;
    return true;
}

bool nemu_link_compact_unpack(uint8_t kind, uint8_t source, uint8_t dongle, const uint8_t *body,
                              size_t size, uint32_t security_sequence, uint32_t epoch,
                              uint32_t session, uint32_t nonce,
                              uint8_t native[NEMU_LINK_FRAME_MAX_SIZE], size_t *native_size) {
    if (!epoch || !session || !nonce || !security_sequence || !nemu_link_compact_kind(kind) ||
        size > NEMU_LINK_COMPACT_BODY_MAX) {
        return false;
    }
    struct reader r = {.data = body, .size = size};
    uint8_t restored[NEMU_LINK_FRAME_MAX_SIZE];
    size_t restored_size;
    if (kind <= COMPACT_STATE) {
        struct nemu_link_link_message m = {.kind = kind == COMPACT_ACK ? NEMU_LINK_LINK_ACK
                                                                       : NEMU_LINK_LINK_STATE,
                                           .source = source,
                                           .epoch = epoch,
                                           .session = session,
                                           .peer_nonce = nonce};
        m.sequence = read_word(&r);
        struct nemu_link_hop_notice h = {.kind = NEMU_LINK_HOP_STABLE, .dongle_id = dongle};
        h.epoch = read_var(&r);
        h.index = read_byte(&r);
        uint8_t flags = read_byte(&r), count = flags >> 4;
        bool buttons = (flags & 8) != 0;
        m.studio_pending = kind == COMPACT_ACK && (flags & NEMU_LINK_COMPACT_STUDIO_PENDING);
        if ((flags & (kind == COMPACT_ACK ? (uint8_t)~NEMU_LINK_COMPACT_STUDIO_PENDING : 7U)) ||
            !m.sequence || (kind == COMPACT_EMPTY && flags)) {
            return false;
        }
        uint32_t mode = read_mode(&r);
        if (kind != COMPACT_ACK) {
            m.battery = read_byte(&r);
            if (count == KEY_BITMAP_MARKER) {
                for (unsigned i = 0; i < NEMU_LINK_LINK_KEY_BYTES; i++) {
                    m.keys.words[i / KEY_WORD_BYTES] |=
                        (uint64_t)read_byte(&r)
                        << (NEMU_LINK_RADIO_BITS_PER_BYTE * (i % KEY_WORD_BYTES));
                }
            } else {
                uint8_t previous = 0;
                for (unsigned i = 0; i < count; i++) {
                    uint8_t p = read_byte(&r);
                    if (p >= NEMU_LINK_LINK_KEY_COUNT || (i && p <= previous)) {
                        return false;
                    }
                    nemu_link_link_keys_set(&m.keys, p, true);
                    previous = p;
                }
            }
            if (buttons) {
                m.buttons = read_word(&r);
            }
            if (kind == COMPACT_STATE && !nemu_link_link_keys_any(m.keys) && !m.buttons) {
                return false;
            }
        }
        restored_size = nemu_link_hop_frame_encode(restored, sizeof(restored), &m, mode, &h, NULL);
    } else {
        struct nemu_link_split_frame f = {.kind = NEMU_LINK_SPLIT_EVENT,
                                          .source = source,
                                          .epoch = epoch,
                                          .session = session,
                                          .nonce = nonce,
                                          .sequence = security_sequence};
        if (kind == COMPACT_INPUT_VAR || kind == COMPACT_INPUT_FIXED) {
            f.data[0] = COMPACT_INPUT_EVENT;
            f.data[1] = read_byte(&r);
            f.size = SPLIT_EVENT_HEADER_SIZE;
            while (!r.error && r.at < r.size) {
                if ((size_t)f.size + INPUT_RECORD_SIZE > sizeof(f.data)) {
                    return false;
                }
                uint8_t *p = f.data + f.size, flags = read_byte(&r);
                p[0] = flags >> 7;
                p[1] = flags & 127;
                uint32_t code, value;
                if (kind == COMPACT_INPUT_VAR) {
                    code = read_var(&r);
                    value = unzig(read_var(&r));
                } else {
                    code = read_byte(&r);
                    code |= (uint32_t)read_byte(&r) << 8;
                    value = read_word(&r);
                }
                if (code > UINT16_MAX) {
                    return false;
                }
                p[2] = (uint8_t)code;
                p[3] = (uint8_t)(code >> 8);
                put32(p + 4, value);
                f.size += INPUT_RECORD_SIZE;
            }
            if (f.size == SPLIT_EVENT_HEADER_SIZE) {
                return false;
            }
        } else {
            f.data[0] = COMPACT_SENSOR_EVENT;
            f.data[1] = read_byte(&r);
            f.size = SENSOR_EVENT_SIZE;
            for (unsigned i = SPLIT_EVENT_HEADER_SIZE; i < SENSOR_EVENT_SIZE;
                 i += sizeof(int32_t)) {
                put32(f.data + i, kind == COMPACT_SENSOR_VAR ? unzig(read_var(&r)) : read_word(&r));
            }
        }
        restored_size = nemu_link_split_encode(restored, &f, NULL);
    }
    if (r.error || r.at != r.size || !restored_size) {
        return false;
    }
    memcpy(native, restored, restored_size);
    *native_size = restored_size;
    return true;
}
