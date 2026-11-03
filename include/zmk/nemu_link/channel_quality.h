#pragma once
#include <zmk/nemu_link/hop.h>

#define NEMU_LINK_QUALITY_SCALE 1000U
struct nemu_link_channel_scan {
    uint16_t busy_ewma;
    uint16_t failure_penalty;
    uint32_t scan_windows;
    uint64_t last_scan_ms;
    uint64_t penalty_expiry;
};
struct nemu_link_peer_quality {
    uint64_t last_rx_ms;
    uint32_t session, sequence, received_data, samples;
    uint16_t loss_ewma;
    bool have_state;
};
struct nemu_link_channel_policy {
    struct nemu_link_channel_scan channels[NEMU_LINK_HOP_CHANNEL_COUNT];
    struct nemu_link_peer_quality peers[NEMU_LINK_HOP_SOURCES];
    uint64_t next_scan_ms, bad_since_ms, validation_deadline, cooldown_until;
    uint8_t scan_index, validation_index;
    bool degrading, validating;
};
struct nemu_link_channel_config {
    uint32_t scan_idle_ms, scan_interval_ms, min_scan_windows;
    uint32_t degrade_ms, validation_ms, cooldown_ms, penalty_ms;
    uint16_t loss_threshold;
    uint32_t min_link_samples;
};
enum nemu_link_migration_result {
    NEMU_LINK_MIGRATION_WAIT,
    NEMU_LINK_MIGRATION_OK,
    NEMU_LINK_MIGRATION_FAILED
};
void nemu_link_channel_init(struct nemu_link_channel_policy *p, uint64_t now);
/* Only fresh authenticated application packets reach these entry points. */
void nemu_link_channel_rx(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now);
/* Accepted split/Studio packets share the STATE's authenticated uplink counter. */
void nemu_link_channel_data(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now,
                            uint32_t session);
void nemu_link_channel_state(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now,
                             uint32_t session, uint32_t sequence);
bool nemu_link_channel_scan_due(struct nemu_link_channel_policy *p,
                                const struct nemu_link_channel_config *cfg,
                                const struct nemu_link_hop_central *hop, uint64_t now,
                                uint8_t *index);
void nemu_link_channel_scan_record(struct nemu_link_channel_policy *p, uint8_t index, uint64_t now,
                                   uint16_t busy, uint16_t samples);
bool nemu_link_channel_candidate(struct nemu_link_channel_policy *p,
                                 const struct nemu_link_channel_config *cfg,
                                 const struct nemu_link_hop_central *hop, uint64_t now,
                                 bool both_up, uint8_t *index);
void nemu_link_channel_changed(struct nemu_link_channel_policy *p);
void nemu_link_channel_validate(struct nemu_link_channel_policy *p,
                                const struct nemu_link_channel_config *cfg, uint8_t index,
                                uint64_t now);
enum nemu_link_migration_result nemu_link_channel_result(struct nemu_link_channel_policy *p,
                                                         const struct nemu_link_channel_config *cfg,
                                                         const struct nemu_link_hop_central *hop,
                                                         uint64_t now, bool both_up);
