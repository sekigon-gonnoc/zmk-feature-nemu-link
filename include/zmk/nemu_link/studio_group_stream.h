#pragma once

#include <zmk/nemu_link/studio_transport.h>
#include <zmk/nemu_link/studio_rx.h>

typedef size_t (*nemu_link_studio_group_count_fn)(void *context);
typedef uint32_t (*nemu_link_studio_group_id_fn)(void *context, size_t index);
typedef bool (*nemu_link_studio_group_apply_fn)(void *context, uint32_t group_id,
                                                const uint8_t *record, size_t size);
typedef bool (*nemu_link_studio_group_commit_fn)(void *context,
                                                 struct nemu_link_studio_version *version);

struct nemu_link_studio_group_rx {
    union {
        struct nemu_link_studio_rx_progress progress;
        struct {
            NEMU_LINK_STUDIO_RX_PROGRESS_FIELDS;
        };
    };
    uint8_t metadata[NEMU_LINK_STUDIO_VERSION_SIZE];
    uint8_t *record;
    size_t record_capacity, record_size, record_used;
    size_t group_count, group_index;
    uint32_t checksum, group_id;
    uint8_t group_header[6], group_header_used;
};

bool nemu_link_studio_group_rx_init(struct nemu_link_studio_group_rx *rx, uint8_t *record,
                                    size_t record_capacity);
bool nemu_link_studio_group_receive(struct nemu_link_studio_group_rx *rx,
                                    const struct nemu_link_studio_packet *packet,
                                    struct nemu_link_studio_version *live,
                                    enum nemu_link_studio_side side, size_t group_count,
                                    nemu_link_studio_group_id_fn group_id,
                                    nemu_link_studio_group_apply_fn apply,
                                    nemu_link_studio_group_commit_fn commit, void *context,
                                    uint64_t now, struct nemu_link_studio_packet *reply);
