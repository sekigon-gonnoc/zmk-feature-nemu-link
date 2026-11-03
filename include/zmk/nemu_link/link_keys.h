#ifndef ZMK_NEMU_LINK_KEYS_H
#define ZMK_NEMU_LINK_KEYS_H

#include <stdbool.h>
#include <stdint.h>

#define NEMU_LINK_LINK_KEY_COUNT 128
#define NEMU_LINK_LINK_KEY_WORDS 2
#define NEMU_LINK_LINK_KEY_BYTES 16

/* Global positions 0..127, independent of source ID. No compiler-specific
 * 128-bit integer or native-endian structure is used on the wire. */
struct nemu_link_link_keys {
    uint64_t words[NEMU_LINK_LINK_KEY_WORDS];
};

static inline struct nemu_link_link_keys nemu_link_link_keys_from_u64(uint64_t low) {
    return (struct nemu_link_link_keys){.words = {low, 0}};
}

static inline bool nemu_link_link_keys_any(struct nemu_link_link_keys keys) {
    return keys.words[0] || keys.words[1];
}

static inline bool nemu_link_link_keys_equal(struct nemu_link_link_keys a,
                                             struct nemu_link_link_keys b) {
    return a.words[0] == b.words[0] && a.words[1] == b.words[1];
}

static inline struct nemu_link_link_keys nemu_link_link_keys_or(struct nemu_link_link_keys a,
                                                                struct nemu_link_link_keys b) {
    return (struct nemu_link_link_keys){
        .words = {a.words[0] | b.words[0], a.words[1] | b.words[1]}};
}

static inline struct nemu_link_link_keys nemu_link_link_keys_and(struct nemu_link_link_keys a,
                                                                 struct nemu_link_link_keys b) {
    return (struct nemu_link_link_keys){
        .words = {a.words[0] & b.words[0], a.words[1] & b.words[1]}};
}

static inline struct nemu_link_link_keys nemu_link_link_keys_without(struct nemu_link_link_keys a,
                                                                     struct nemu_link_link_keys b) {
    return (struct nemu_link_link_keys){
        .words = {a.words[0] & ~b.words[0], a.words[1] & ~b.words[1]}};
}

static inline bool nemu_link_link_keys_test(struct nemu_link_link_keys keys, uint32_t position) {
    return position < NEMU_LINK_LINK_KEY_COUNT &&
           (keys.words[position / 64] & (UINT64_C(1) << (position % 64)));
}

/* Out-of-range input is rejected without modifying the bitmap. */
static inline bool nemu_link_link_keys_set(struct nemu_link_link_keys *keys, uint32_t position,
                                           bool pressed) {
    if (!keys || position >= NEMU_LINK_LINK_KEY_COUNT) {
        return false;
    }
    uint64_t bit = UINT64_C(1) << (position % 64);
    if (pressed) {
        keys->words[position / 64] |= bit;
    } else {
        keys->words[position / 64] &= ~bit;
    }
    return true;
}

#endif
