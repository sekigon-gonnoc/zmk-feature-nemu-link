#include <zmk/nemu_link/hop_frame.h>
#include <zmk/nemu_link/selection.h>
#include <string.h>

_Static_assert(NEMU_LINK_HOP_FRAME_SIZE == 64, "hop must fit the existing ESB payload limit");
static void put32(uint8_t *p, uint32_t v) {
    for (unsigned int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static bool direction(enum nemu_link_link_kind kind, const struct nemu_link_hop_notice *n) {
    if (!nemu_link_hop_notice_valid(n)) {
        return false;
    }
    return kind == NEMU_LINK_LINK_ACK || kind == NEMU_LINK_LINK_CHALLENGE
               ? n->kind == NEMU_LINK_HOP_STABLE || n->kind == NEMU_LINK_HOP_PREPARE ||
                     n->kind == NEMU_LINK_HOP_COMMIT
               : n->kind != NEMU_LINK_HOP_PREPARE && n->kind != NEMU_LINK_HOP_COMMIT;
}
size_t nemu_link_hop_frame_encode(uint8_t *bytes, size_t size,
                                  const struct nemu_link_link_message *m, uint32_t mode,
                                  const struct nemu_link_hop_notice *n, nemu_link_link_crc_fn crc) {
    if (!m || !n || n->dongle_id > NEMU_LINK_MAX_DONGLES || !direction(m->kind, n)) {
        return 0;
    }
    uint8_t extension[NEMU_LINK_HOP_FRAME_EXTENSION_SIZE] = {NEMU_LINK_HOP_PROTOCOL_VERSION,
                                                             NEMU_LINK_HOP_TABLE_VERSION};
    put32(extension + 2, n->epoch);
    extension[6] = n->index;
    extension[7] = (uint8_t)n->kind;
    extension[8] = n->dongle_id;
    return nemu_link_link_frame_encode_extension(bytes, size, m, mode, extension, crc);
}

bool nemu_link_hop_frame_decode(struct nemu_link_link_message *m, uint32_t *mode,
                                struct nemu_link_hop_notice *n, uint8_t pipe, const uint8_t *bytes,
                                size_t size, nemu_link_link_crc_fn crc) {
    struct nemu_link_link_message message;
    uint32_t token;
    if (!m || !mode || !n ||
        !nemu_link_link_frame_decode_extension(&message, &token, pipe, bytes, size, true, crc)) {
        return false;
    }
    size_t ext = size - NEMU_LINK_HOP_FRAME_EXTENSION_SIZE - 4;
    if (bytes[ext] != NEMU_LINK_HOP_PROTOCOL_VERSION ||
        bytes[ext + 1] != NEMU_LINK_HOP_TABLE_VERSION || bytes[ext + 8] > NEMU_LINK_MAX_DONGLES) {
        return false;
    }
    struct nemu_link_hop_notice decoded = {.epoch = get32(bytes + ext + 2),
                                           .index = bytes[ext + 6],
                                           .kind = (enum nemu_link_hop_kind)bytes[ext + 7],
                                           .dongle_id = bytes[ext + 8]};
    if (!direction(message.kind, &decoded)) {
        return false;
    }
    *m = message;
    *mode = token;
    *n = decoded;
    return true;
}
