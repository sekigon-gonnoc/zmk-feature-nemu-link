#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Reuse the authenticated heartbeat mode word; no larger/more frequent frames. */
#define NEMU_LINK_MAX_DONGLES 8U
#define NEMU_LINK_SELECTION_TARGET_BITS 3U
#define NEMU_LINK_SELECTION_TARGET_MASK ((1U << NEMU_LINK_SELECTION_TARGET_BITS) - 1U)
#define NEMU_LINK_SELECTION_PREFIX UINT32_C(0xe7000000)
#define NEMU_LINK_SELECTION_MAX (UINT32_C(0x00ffffff) >> NEMU_LINK_SELECTION_TARGET_BITS)
struct nemu_link_selection {
    uint32_t generation;
    uint8_t target;
};
static inline bool nemu_link_selection_valid(struct nemu_link_selection s) {
    return s.generation <= NEMU_LINK_SELECTION_MAX && s.target >= 1 &&
           s.target <= NEMU_LINK_MAX_DONGLES;
}
static inline uint32_t nemu_link_selection_encode(struct nemu_link_selection s) {
    return nemu_link_selection_valid(s)
               ? NEMU_LINK_SELECTION_PREFIX | (s.generation << NEMU_LINK_SELECTION_TARGET_BITS) |
                     (s.target - 1U)
               : 0;
}
static inline bool nemu_link_selection_decode(uint32_t token, struct nemu_link_selection *s) {
    if (!s || (token & UINT32_C(0xff000000)) != NEMU_LINK_SELECTION_PREFIX) {
        return false;
    }
    *s = (struct nemu_link_selection){
        .generation = (token & UINT32_C(0x00ffffff)) >> NEMU_LINK_SELECTION_TARGET_BITS,
        .target = (uint8_t)((token & NEMU_LINK_SELECTION_TARGET_MASK) + 1U)};
    return true;
}
static inline bool nemu_link_selection_equal(struct nemu_link_selection a,
                                             struct nemu_link_selection b) {
    return a.generation == b.generation && a.target == b.target;
}
/* No wall-clock ordering across dongles. Equal-generation conflicts choose the lower ID. */
static inline bool nemu_link_selection_merge(struct nemu_link_selection *s,
                                             struct nemu_link_selection remote) {
    if (!s || !nemu_link_selection_valid(remote) ||
        (nemu_link_selection_valid(*s) &&
         (remote.generation < s->generation ||
          (remote.generation == s->generation && remote.target >= s->target)))) {
        return false;
    }
    *s = remote;
    return true;
}
/* Fail at exhaustion, never wrap and make old records look newer. */
static inline bool nemu_link_selection_next(struct nemu_link_selection *s, uint8_t target) {
    if (!s || !nemu_link_selection_valid(*s) || s->generation == NEMU_LINK_SELECTION_MAX ||
        target < 1 || target > NEMU_LINK_MAX_DONGLES) {
        return false;
    }
    s->generation++;
    s->target = target;
    return true;
}
