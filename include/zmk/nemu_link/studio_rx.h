#pragma once

#include <zmk/nemu_link/studio_transport.h>

/* Both receivers expose the existing field names while sharing a typed view
 * of the transport progress. Payload buffers and validation stay domain-local. */
#define NEMU_LINK_STUDIO_RX_PROGRESS_FIELDS                                                        \
    struct nemu_link_studio_version base, target;                                                  \
    struct nemu_link_studio_packet last;                                                           \
    uint64_t last_progress;                                                                        \
    uint16_t transaction, metadata_received, received;                                             \
    bool started, receipt, expired

struct nemu_link_studio_rx_progress {
    NEMU_LINK_STUDIO_RX_PROGRESS_FIELDS;
};

bool nemu_link_studio_rx_metadata(struct nemu_link_studio_rx_progress *rx, uint8_t *metadata,
                                  size_t capacity, const struct nemu_link_studio_packet *packet,
                                  uint64_t now);
