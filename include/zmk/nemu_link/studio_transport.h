#pragma once
#include <zmk/nemu_link/studio_sync.h>
#include <zmk/nemu_link/hop.h>
#include <zmk/nemu_link/link_frame.h>

#define NEMU_LINK_STUDIO_FRAME_SIZE 64
#define NEMU_LINK_STUDIO_VERSION_SIZE 84
#define NEMU_LINK_STUDIO_FRAME_TAG 0x80
#define NEMU_LINK_STUDIO_WIRE_VERSION 1
#define NEMU_LINK_STUDIO_DELTA_META_SIZE (2 * NEMU_LINK_STUDIO_VERSION_SIZE + 2)
#ifndef CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE
#define CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE 0
#endif

enum nemu_link_studio_packet_kind {
    NEMU_LINK_STUDIO_META = 1,
    NEMU_LINK_STUDIO_DATA,
    NEMU_LINK_STUDIO_COMMIT,
    NEMU_LINK_STUDIO_META_ACK,
    NEMU_LINK_STUDIO_DATA_ACK,
    NEMU_LINK_STUDIO_COMMIT_ACK,
    NEMU_LINK_STUDIO_ERROR,
    NEMU_LINK_STUDIO_POLL,
    NEMU_LINK_STUDIO_DELTA_META,
    NEMU_LINK_STUDIO_GROUP_META,
    NEMU_LINK_STUDIO_GROUP_DATA,
    NEMU_LINK_STUDIO_GROUP_COMMIT,
    NEMU_LINK_STUDIO_GROUP_META_ACK,
    NEMU_LINK_STUDIO_GROUP_DATA_ACK,
    NEMU_LINK_STUDIO_GROUP_COMMIT_ACK,
    NEMU_LINK_STUDIO_REMOTE_REQUEST,
    NEMU_LINK_STUDIO_REMOTE_RESPONSE,
    NEMU_LINK_STUDIO_SCHEMA_HASH,
    NEMU_LINK_STUDIO_REMOTE_BASE,
    NEMU_LINK_STUDIO_REMOTE_BASE_ACK,
    NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH,
    NEMU_LINK_STUDIO_OWNER_RPC_REQUEST,
    NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE,
};
struct nemu_link_studio_packet {
    enum nemu_link_studio_packet_kind kind;
    uint16_t transaction, offset;
    uint8_t length, data[NEMU_LINK_STUDIO_CHUNK_SIZE];
};
static inline bool nemu_link_studio_transaction_newer(uint16_t candidate, uint16_t current) {
    if (!current) {
        return candidate != 0;
    }
    uint16_t distance = (uint16_t)(candidate - current);
    return distance && distance < 0x8000u;
}
/* link session + actually applied hop channel. No configuration packet renews the
 * snapshot/ACK/hop lease. Normal STATE/ACK traffic remains mandatory. */
struct nemu_link_studio_route {
    enum nemu_link_studio_side local_side;
    uint32_t boot, session, nonce, channel_epoch;
    uint8_t channel_index, source, dongle_id;
};
bool nemu_link_studio_route_central(struct nemu_link_studio_route *route,
                                    const struct nemu_link_link_central *link,
                                    const struct nemu_link_hop_central *hop, uint64_t now);
bool nemu_link_studio_route_central_source(struct nemu_link_studio_route *route,
                                           const struct nemu_link_link_central *link,
                                           const struct nemu_link_hop_central *hop, uint8_t source,
                                           uint64_t now);
bool nemu_link_studio_route_peripheral(struct nemu_link_studio_route *route,
                                       const struct nemu_link_link_peripheral *link,
                                       const struct nemu_link_hop_peripheral *hop, uint64_t now);
bool nemu_link_studio_route_peripheral_source(struct nemu_link_studio_route *route,
                                              const struct nemu_link_link_peripheral *link,
                                              const struct nemu_link_hop_peripheral *hop,
                                              uint8_t source, uint64_t now);
/* True when two route snapshots belong to the same authenticated link session.
 * Hop channel epoch/index may differ between independently transmitted frames. */
bool nemu_link_studio_route_same_session(const struct nemu_link_studio_route *a,
                                         const struct nemu_link_studio_route *b);
struct nemu_link_studio_owner_request_assembly {
    uint16_t transaction;
    uint8_t received_mask;
    uint8_t bytes[36];
    struct nemu_link_studio_route route;
};
/* Assemble the two fixed OWNER_RPC_REQUEST chunks. Duplicate and out-of-order
 * chunks are accepted; returns true once all request bytes are available. */
bool nemu_link_studio_owner_request_assemble(
    struct nemu_link_studio_owner_request_assembly *assembly,
    const struct nemu_link_studio_packet *packet, const struct nemu_link_studio_route *route,
    uint8_t bytes[36]);
bool nemu_link_studio_packet_valid(const struct nemu_link_studio_packet *packet);
/* Encode returns transmitted byte count, or zero on error. Decode takes actual RX length. */
size_t nemu_link_studio_frame_encode(uint8_t *bytes, size_t size,
                                     const struct nemu_link_studio_route *route,
                                     const struct nemu_link_studio_packet *packet,
                                     nemu_link_link_crc_fn crc);
bool nemu_link_studio_frame_decode(struct nemu_link_studio_packet *packet,
                                   const struct nemu_link_studio_route *route, uint8_t pipe,
                                   const uint8_t *bytes, size_t size, nemu_link_link_crc_fn crc);
/* Also used by future atomic persistence. Explicit field encoding, no padding. */
bool nemu_link_studio_version_encode(uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE],
                                     const struct nemu_link_studio_version *version);
bool nemu_link_studio_version_decode(struct nemu_link_studio_version *version,
                                     const uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE]);

enum nemu_link_studio_tx_phase {
    NEMU_LINK_STUDIO_TX_IDLE,
    NEMU_LINK_STUDIO_TX_META,
    NEMU_LINK_STUDIO_TX_DATA,
    NEMU_LINK_STUDIO_TX_COMMIT,
    NEMU_LINK_STUDIO_TX_DONE,
    NEMU_LINK_STUDIO_TX_FAILED
};
typedef bool (*nemu_link_studio_read_fn)(void *, uint32_t, uint8_t *, size_t);
struct nemu_link_studio_tx {
    enum nemu_link_studio_tx_phase phase;
    enum nemu_link_studio_result error;
    uint16_t transaction, offset;
    uint64_t last_progress, last_sent;
    uint32_t retry_ms, timeout_ms;
    bool sent, delta, groups, prepared;
    uint16_t transfer_size;
    struct nemu_link_studio_packet proposal;
    nemu_link_studio_read_fn read;
    void *read_context;
    const uint8_t *image; /* Caller-owned immutable snapshot until done/abort. */
    struct nemu_link_studio_version version;
    uint8_t metadata[NEMU_LINK_STUDIO_DELTA_META_SIZE];
};
/* Zero-initialize TX once per link session. start requires an idle/done/failed TX
 * and a nonzero ID not used by another active transfer in that session. IDs may wrap. */
bool nemu_link_studio_tx_start(struct nemu_link_studio_tx *tx, uint16_t transaction,
                               const struct nemu_link_studio_version *version, const uint8_t *image,
                               uint64_t now, uint32_t retry_ms, uint32_t timeout_ms);
/* Reader must serve the previously checksummed, stable saved map until abort/done. */
bool nemu_link_studio_tx_start_reader(struct nemu_link_studio_tx *, uint16_t,
                                      const struct nemu_link_studio_version *,
                                      nemu_link_studio_read_fn, void *, uint64_t, uint32_t,
                                      uint32_t);
bool nemu_link_studio_tx_start_groups_reader(struct nemu_link_studio_tx *, uint16_t,
                                             const struct nemu_link_studio_version *,
                                             nemu_link_studio_read_fn, void *, uint64_t, uint32_t,
                                             uint32_t);
/* Delta reader serves ascending index:u16, length:u8, record[] entries.
 * Target still describes the complete map. Receiver must match base exactly. */
bool nemu_link_studio_tx_start_delta(struct nemu_link_studio_tx *, uint16_t,
                                     const struct nemu_link_studio_version *,
                                     const struct nemu_link_studio_version *, uint16_t,
                                     nemu_link_studio_read_fn, void *, uint64_t, uint32_t,
                                     uint32_t);
/* next is a proposal; submitted is called only after queue insertion succeeds. */
bool nemu_link_studio_tx_next(struct nemu_link_studio_tx *tx, uint64_t now,
                              struct nemu_link_studio_packet *packet);
void nemu_link_studio_tx_submitted(struct nemu_link_studio_tx *tx, uint64_t now);
bool nemu_link_studio_tx_receive(struct nemu_link_studio_tx *tx,
                                 const struct nemu_link_studio_packet *reply, uint64_t now);

struct nemu_link_studio_transfer_rx {
    struct nemu_link_studio_rx image;
    uint16_t transaction, metadata_received;
    uint8_t metadata[NEMU_LINK_STUDIO_VERSION_SIZE];
    uint64_t last_progress;
    bool expired;
};
/* Reinitialize on link session change, NOT on a channel hop. IDs are nonzero and
 * may wrap within each direction/session. Incoming/own outgoing transfers have separate state. */
bool nemu_link_studio_transfer_rx_init(struct nemu_link_studio_transfer_rx *rx, uint8_t *staging,
                                       size_t capacity, uint32_t timeout_ms);
void nemu_link_studio_transfer_rx_tick(struct nemu_link_studio_transfer_rx *rx, uint64_t now);
/* Call only AFTER frame/route validation. POLL/ACK packets have no RX effect.
 * COMMIT runs callbacks in caller context; never call it in radio IRQ or while
 * holding the link worker/lock across a flash operation. */
bool nemu_link_studio_transfer_receive(struct nemu_link_studio_transfer_rx *rx,
                                       const struct nemu_link_studio_packet *packet,
                                       struct nemu_link_studio_version *live, uint64_t now,
                                       nemu_link_studio_validate_fn validate,
                                       nemu_link_studio_persist_fn persist, void *context,
                                       struct nemu_link_studio_packet *reply);
