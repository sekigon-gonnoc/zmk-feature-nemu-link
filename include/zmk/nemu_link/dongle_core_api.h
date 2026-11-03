#pragma once

#include <zmk/nemu_link/hop_link.h>

struct nemu_link_dongle_core_api {
    bool (*link_central_init)(struct nemu_link_link_central *, uint32_t, uint32_t,
                              const uint32_t[NEMU_LINK_LINK_SOURCES]);
    void (*link_central_tick)(struct nemu_link_link_central *, uint8_t, uint64_t,
                              struct nemu_link_link_result *);
    bool (*link_central_receive)(struct nemu_link_link_central *, uint8_t,
                                 const struct nemu_link_link_message *, uint64_t,
                                 struct nemu_link_link_result *);
    bool (*hop_central_init)(struct nemu_link_hop_central *, uint32_t, uint64_t,
                             const struct nemu_link_hop_timing *);
    bool (*hop_central_request_index)(struct nemu_link_hop_central *, uint64_t, uint8_t);
    bool (*hop_central_request)(struct nemu_link_hop_central *, uint64_t);
    struct nemu_link_hop_notice (*hop_central_notice)(const struct nemu_link_hop_central *);
    bool (*hop_central_receive)(struct nemu_link_hop_central *, uint8_t, uint64_t,
                                const struct nemu_link_hop_notice *);
    bool (*hop_central_next)(const struct nemu_link_hop_central *, uint64_t,
                             struct nemu_link_hop_notice *);
    bool (*hop_central_applied)(struct nemu_link_hop_central *, uint64_t,
                                const struct nemu_link_hop_notice *);
    bool (*hop_link_central_receive)(struct nemu_link_link_central *,
                                     struct nemu_link_hop_central *, uint8_t,
                                     const struct nemu_link_link_message *,
                                     const struct nemu_link_hop_notice *, uint64_t,
                                     struct nemu_link_link_result *);
};
