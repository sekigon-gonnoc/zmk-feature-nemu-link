#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <zmk/nemu_link/key_record.h>

#define NEMU_LINK_HOP_CHANNEL_COUNT NEMU_LINK_RADIO_CHANNEL_COUNT_MAX
#define NEMU_LINK_HOP_TABLE_VERSION 1
#define NEMU_LINK_HOP_PROTOCOL_VERSION 1
#define NEMU_LINK_HOP_SOURCES 2
extern uint8_t nemu_link_hop_channels[NEMU_LINK_HOP_CHANNEL_COUNT];
extern uint8_t nemu_link_hop_channel_count;
bool nemu_link_hop_set_channels(const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                uint8_t channel_count);

enum nemu_link_hop_kind {
    NEMU_LINK_HOP_STABLE,
    NEMU_LINK_HOP_PREPARE,
    NEMU_LINK_HOP_PROBE,
    NEMU_LINK_HOP_ECHO,
    NEMU_LINK_HOP_COMMIT
};
struct nemu_link_hop_notice {
    uint32_t epoch;
    uint8_t index;
    uint8_t dongle_id;
    enum nemu_link_hop_kind kind;
};
struct nemu_link_hop_timing {
    uint32_t loss_ms;
    uint32_t announce_ms;
    uint32_t probe_ms;
    uint32_t peer_dwell_ms;
    uint32_t central_dwell_ms;
};
/* After both peers echo PREPARE, advertise COMMIT for five probe periods.
 * Peers only shorten loss detection after COMMIT, to ten probe periods. */
#define NEMU_LINK_HOP_COMMIT_PROBES 5U
#define NEMU_LINK_HOP_FOLLOW_PROBES 10U
/* All times are local monotonic ms. Caller must service at least every 10ms.
 * Dwell margins allow a full fast scan and two transactions for delayed ACK.
 * These bounds assume successful radio operations and bounded packet loss. */
bool nemu_link_hop_timing_valid(const struct nemu_link_hop_timing *timing);
bool nemu_link_hop_notice_valid(const struct nemu_link_hop_notice *notice);

struct nemu_link_hop_central {
    struct nemu_link_hop_timing timing;
    uint32_t boot; /* Bound to link central boot epoch; generation never wraps. */
    uint32_t epoch;
    uint8_t channel_count;
    uint8_t index;
    bool searching;
    bool pending;
    bool committing;
    uint8_t receipt_mask;
    uint8_t pending_index;
    uint64_t switch_ms;
    uint64_t changed_ms;
    uint64_t seen_ms[NEMU_LINK_HOP_SOURCES];
};
bool nemu_link_hop_central_init(struct nemu_link_hop_central *c, uint32_t boot, uint64_t now,
                                const struct nemu_link_hop_timing *timing);
/* Explicit interference/diagnostic request. One silent side alone does not
 * trigger migration. Both active ECHOs allow a short COMMIT window; otherwise
 * the full announcement window and ordinary loss detection remain in use. */
bool nemu_link_hop_central_request(struct nemu_link_hop_central *c, uint64_t now);
bool nemu_link_hop_central_request_index(struct nemu_link_hop_central *c, uint64_t now,
                                         uint8_t index);
struct nemu_link_hop_notice nemu_link_hop_central_notice(const struct nemu_link_hop_central *c);
/* Call only for a fresh link-accepted uplink on the currently applied channel.
 * HELLO may carry PROBE; this is not permission to refresh link key liveness. */
bool nemu_link_hop_central_receive(struct nemu_link_hop_central *c, uint8_t source, uint64_t now,
                                   const struct nemu_link_hop_notice *echo);
/* next() is pure. Retune after quiescing the radio, then call applied(). Failure
 * leaves the old generation authoritative. Expire link input BEFORE doing this. */
bool nemu_link_hop_central_next(const struct nemu_link_hop_central *c, uint64_t now,
                                struct nemu_link_hop_notice *target);
bool nemu_link_hop_central_applied(struct nemu_link_hop_central *c, uint64_t now,
                                   const struct nemu_link_hop_notice *target);

struct nemu_link_hop_peripheral {
    struct nemu_link_hop_timing timing;
    uint32_t boot;
    uint32_t epoch;
    uint8_t known_index;
    uint8_t pending_index;
    uint8_t index; /* Actually applied RF index; separate from authority. */
    bool searching;
    bool pending;
    bool committed;
    uint64_t heard_ms;
    uint64_t changed_ms;
    uint64_t probed_ms;
    bool has_probed;
};
bool nemu_link_hop_peripheral_init(struct nemu_link_hop_peripheral *p, uint64_t now,
                                   const struct nemu_link_hop_timing *timing);
/* Bind only after a fresh link CHALLENGE validated against the current nonce.
 * A changed boot invalidates old generations. Same-boot reconnect preserves
 * the high-water mark, so delayed older channel commands remain rejected. */
bool nemu_link_hop_peripheral_bind(struct nemu_link_hop_peripheral *p, uint32_t boot);
/* Only fresh, validated link CHALLENGE/ACK may confirm authority/liveness.
 * PREPARE enables frequent STATEs; COMMIT also enables short loss detection. */
bool nemu_link_hop_peripheral_receive(struct nemu_link_hop_peripheral *p, uint32_t boot,
                                      uint64_t now, const struct nemu_link_hop_notice *notice);
struct nemu_link_hop_notice
nemu_link_hop_peripheral_notice(const struct nemu_link_hop_peripheral *p);
bool nemu_link_hop_peripheral_next(const struct nemu_link_hop_peripheral *p, uint64_t now,
                                   uint8_t *index);
bool nemu_link_hop_peripheral_applied(struct nemu_link_hop_peripheral *p, uint64_t now,
                                      uint8_t index);
/* Extra STATE traffic exists in search and during a pending move. Ordinary
 * heartbeat/snapshot handles stable traffic. Probes use link framing/session. */
bool nemu_link_hop_probe_due(const struct nemu_link_hop_peripheral *p, uint64_t now);
void nemu_link_hop_probe_sent(struct nemu_link_hop_peripheral *p, uint64_t now);
bool nemu_link_hop_idle_allowed(const struct nemu_link_hop_peripheral *p, uint64_t now);
