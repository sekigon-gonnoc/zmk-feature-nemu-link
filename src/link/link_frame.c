#include <zmk/nemu_link/link_frame.h>
#include <string.h>
static void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static bool valid_mode(enum nemu_link_link_kind kind, uint32_t mode) {
    return !mode || kind == NEMU_LINK_LINK_STATE || kind == NEMU_LINK_LINK_ACK;
}
/* Only STATE carries the complete key/button snapshot. */
size_t nemu_link_link_frame_encode(uint8_t *b, size_t capacity,
                                   const struct nemu_link_link_message *m, uint32_t mode,
                                   nemu_link_link_crc_fn crc) {
    return nemu_link_link_frame_encode_extension(b, capacity, m, mode, NULL, crc);
}

size_t nemu_link_link_frame_encode_extension(
    uint8_t *b, size_t capacity, const struct nemu_link_link_message *m, uint32_t mode,
    const uint8_t extension[NEMU_LINK_LINK_FRAME_EXTENSION_SIZE], nemu_link_link_crc_fn crc) {
    size_t extra = extension ? NEMU_LINK_LINK_FRAME_EXTENSION_SIZE : 0;
    if (!b || !m || capacity != NEMU_LINK_LINK_FRAME_SIZE + extra || !valid_mode(m->kind, mode)) {
        return 0;
    }
    size_t core_size = m->kind == NEMU_LINK_LINK_STATE ? NEMU_LINK_LINK_WIRE_SIZE
                                                       : NEMU_LINK_LINK_CONTROL_WIRE_SIZE;
    if (!nemu_link_link_core_encode(b + 7, core_size, m)) {
        return 0;
    }
    size_t size = 7 + core_size + 8 + extra;
    memcpy(b, "ZmKe", 4);
    b[4] = (uint8_t)(size - 9);
    b[5] = m->source;
    b[6] = extension ? 0x7f : 0x7e;
    put32(b + 7 + core_size, mode);
    if (extension) {
        memcpy(b + 11 + core_size, extension, extra);
    }
    put32(b + size - 4, crc ? crc(b, size - 4) : 0);
    return size;
}

bool nemu_link_link_frame_decode(struct nemu_link_link_message *m, uint32_t *mode, uint8_t pipe,
                                 const uint8_t *b, size_t size, nemu_link_link_crc_fn crc) {
    return nemu_link_link_frame_decode_extension(m, mode, pipe, b, size, false, crc);
}

bool nemu_link_link_frame_decode_extension(struct nemu_link_link_message *m, uint32_t *mode,
                                           uint8_t pipe, const uint8_t *b, size_t size,
                                           bool extended, nemu_link_link_crc_fn crc) {
    size_t extra = extended ? NEMU_LINK_LINK_FRAME_EXTENSION_SIZE : 0;
    if (!m || !mode || !b || size < 35 + extra || size > NEMU_LINK_LINK_FRAME_SIZE + extra ||
        pipe >= NEMU_LINK_LINK_SOURCES || memcmp(b, "ZmKe", 4) || b[4] != size - 9 ||
        b[5] != pipe || b[6] != (extended ? 0x7f : 0x7e) ||
        (crc && get32(b + size - 4) != crc(b, size - 4))) {
        return false;
    }
    size_t core_size =
        b[8] == NEMU_LINK_LINK_STATE ? NEMU_LINK_LINK_WIRE_SIZE : NEMU_LINK_LINK_CONTROL_WIRE_SIZE;
    if (size != 7 + core_size + 8 + extra) {
        return false;
    }
    struct nemu_link_link_message decoded;
    uint32_t token = get32(b + 7 + core_size);
    if (!nemu_link_link_core_decode(&decoded, b + 7, core_size) || decoded.source != pipe ||
        !valid_mode(decoded.kind, token)) {
        return false;
    }
    *m = decoded;
    *mode = token;
    return true;
}
