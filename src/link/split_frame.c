#include <zmk/nemu_link/split_frame.h>
#include <string.h>
static void put32(uint8_t *p, uint32_t value) {
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(value >> (8 * i));
    }
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static bool valid(const struct nemu_link_split_frame *f) {
    return (f->kind == NEMU_LINK_SPLIT_EVENT || f->kind == NEMU_LINK_SPLIT_COMMAND) &&
           f->source < 2 && f->epoch && f->session && f->nonce && f->sequence && f->size > 0 &&
           f->size <= NEMU_LINK_SPLIT_DATA_SIZE;
}
size_t nemu_link_split_encode(uint8_t w[64], const struct nemu_link_split_frame *f,
                              uint32_t (*crc)(const uint8_t *, size_t)) {
    if (!w || !f || !valid(f)) {
        return false;
    }
    memset(w, 0, 64);
    memcpy(w, "ErEv", 4);
    w[4] = 2;
    w[6] = f->kind;
    w[7] = f->source;
    put32(w + 8, f->epoch);
    put32(w + 12, f->session);
    put32(w + 16, f->nonce);
    put32(w + 20, f->sequence);
    w[24] = f->size;
    memcpy(w + 25, f->data, f->size);
    size_t size = 29 + f->size;
    put32(w + size - 4, crc ? crc(w, size - 4) : 0);
    return size;
}
bool nemu_link_split_decode(struct nemu_link_split_frame *f, const uint8_t *w, size_t size,
                            uint32_t (*crc)(const uint8_t *, size_t)) {
    if (!f || !w || size < 30 || size > 64 || memcmp(w, "ErEv", 4) || w[4] != 2 || w[5] ||
        size != 29u + w[24] || (crc && get32(w + size - 4) != crc(w, size - 4))) {
        return false;
    }
    *f = (struct nemu_link_split_frame){.kind = w[6],
                                        .source = w[7],
                                        .size = w[24],
                                        .epoch = get32(w + 8),
                                        .session = get32(w + 12),
                                        .nonce = get32(w + 16),
                                        .sequence = get32(w + 20)};
    if (!valid(f)) {
        return false;
    }
    memcpy(f->data, w + 25, f->size);
    return true;
}

bool nemu_link_split_accept(struct nemu_link_split_rx *rx,
                            const struct nemu_link_split_frame *expected,
                            const struct nemu_link_split_frame *f) {
    if (!rx || !expected || !f || !valid(f) || f->source != expected->source ||
        f->epoch != expected->epoch || f->session != expected->session ||
        f->nonce != expected->nonce) {
        return false;
    }
    if (rx->epoch == f->epoch && rx->session == f->session && rx->nonce == f->nonce &&
        (f->sequence == rx->sequence ||
         (uint32_t)(f->sequence - rx->sequence) >= UINT32_C(0x80000000))) {
        return false;
    }
    *rx = (struct nemu_link_split_rx){f->epoch, f->session, f->nonce, f->sequence};
    return true;
}
