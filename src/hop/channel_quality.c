#include <zmk/nemu_link/channel_quality.h>

static uint16_t ewma(uint16_t old, uint16_t sample) {
    return (uint16_t)(((uint32_t)old * 7U + sample) / 8U);
}
void nemu_link_channel_init(struct nemu_link_channel_policy *p, uint64_t now) {
    *p = (struct nemu_link_channel_policy){.next_scan_ms = now};
    for (uint8_t source = 0; source < NEMU_LINK_HOP_SOURCES; source++) {
        p->peers[source].last_rx_ms = now;
    }
}
void nemu_link_channel_rx(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now) {
    p->peers[source].last_rx_ms = now;
}
void nemu_link_channel_data(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now,
                            uint32_t session) {
    struct nemu_link_peer_quality *q = &p->peers[source];
    nemu_link_channel_rx(p, source, now);
    if (q->have_state && q->session == session) {
        q->received_data++;
    }
}
void nemu_link_channel_state(struct nemu_link_channel_policy *p, uint8_t source, uint64_t now,
                             uint32_t session, uint32_t sequence) {
    struct nemu_link_peer_quality *q = &p->peers[source];
    nemu_link_channel_rx(p, source, now);
    if (q->have_state && q->session == session) {
        /* The secure uplink counter includes STATE, split and Studio packets.
         * Evaluate once per fresh STATE, subtracting all accepted intervening
         * data packets as well as this STATE from the expected packet count. */
        uint32_t delta = sequence - q->sequence;
        uint16_t missed =
            (uint16_t)((uint64_t)(delta - q->received_data - 1U) * NEMU_LINK_QUALITY_SCALE / delta);
        q->loss_ewma = ewma(q->loss_ewma, missed);
        q->samples++;
    } else {
        q->loss_ewma = 0;
        q->samples = 0;
    }
    q->session = session;
    q->sequence = sequence;
    q->received_data = 0;
    q->have_state = true;
}
bool nemu_link_channel_scan_due(struct nemu_link_channel_policy *p,
                                const struct nemu_link_channel_config *cfg,
                                const struct nemu_link_hop_central *hop, uint64_t now,
                                uint8_t *index) {
    if (nemu_link_hop_channel_count < 2 || hop->pending || hop->searching || p->validating ||
        now < p->next_scan_ms || now - p->peers[0].last_rx_ms < cfg->scan_idle_ms ||
        now - p->peers[1].last_rx_ms < cfg->scan_idle_ms ||
        (now - hop->seen_ms[0] >= hop->timing.loss_ms &&
         now - hop->seen_ms[1] >= hop->timing.loss_ms)) {
        return false;
    }
    p->next_scan_ms = now + cfg->scan_interval_ms;
    if (p->scan_index == hop->index) {
        p->scan_index = (uint8_t)((p->scan_index + 1U) % nemu_link_hop_channel_count);
    }
    *index = p->scan_index;
    p->scan_index = (uint8_t)((p->scan_index + 1U) % nemu_link_hop_channel_count);
    return true;
}
void nemu_link_channel_scan_record(struct nemu_link_channel_policy *p, uint8_t index, uint64_t now,
                                   uint16_t busy, uint16_t samples) {
    struct nemu_link_channel_scan *q = &p->channels[index];
    uint16_t ratio = (uint16_t)((uint32_t)busy * NEMU_LINK_QUALITY_SCALE / samples);
    q->busy_ewma = q->scan_windows ? ewma(q->busy_ewma, ratio) : ratio;
    q->scan_windows++;
    q->last_scan_ms = now;
}
bool nemu_link_channel_candidate(struct nemu_link_channel_policy *p,
                                 const struct nemu_link_channel_config *cfg,
                                 const struct nemu_link_hop_central *hop, uint64_t now,
                                 bool both_up, uint8_t *index) {
    bool bad =
        both_up && !hop->pending && !hop->searching && !p->validating && now >= p->cooldown_until;
    for (uint8_t source = 0; source < NEMU_LINK_HOP_SOURCES; source++) {
        const struct nemu_link_peer_quality *q = &p->peers[source];
        bad &= q->samples >= cfg->min_link_samples && q->loss_ewma >= cfg->loss_threshold &&
               now - q->last_rx_ms < hop->timing.loss_ms;
    }
    if (!bad) {
        p->degrading = false;
        return false;
    }
    if (!p->degrading) {
        p->degrading = true;
        p->bad_since_ms = now;
    }
    if (now - p->bad_since_ms < cfg->degrade_ms) {
        return false;
    }
    uint16_t best = UINT16_MAX;
    bool found = false;
    for (unsigned int i = 0; i < nemu_link_hop_channel_count; i++) {
        const struct nemu_link_channel_scan *q = &p->channels[i];
        if (i == hop->index || q->scan_windows < cfg->min_scan_windows || now < q->penalty_expiry) {
            continue;
        }
        /* Treat differences smaller than 5 percent as a tie. */
        if (!found || q->busy_ewma + 50U < best) {
            best = q->busy_ewma;
            *index = (uint8_t)i;
            found = true;
        }
    }
    return found;
}
void nemu_link_channel_changed(struct nemu_link_channel_policy *p) {
    p->degrading = false;
    for (uint8_t source = 0; source < NEMU_LINK_HOP_SOURCES; source++) {
        p->peers[source].have_state = false;
        p->peers[source].received_data = 0;
        p->peers[source].samples = 0;
        p->peers[source].loss_ewma = 0;
    }
}
void nemu_link_channel_validate(struct nemu_link_channel_policy *p,
                                const struct nemu_link_channel_config *cfg, uint8_t index,
                                uint64_t now) {
    p->validating = true;
    p->validation_index = index;
    p->validation_deadline = now + cfg->validation_ms;
}
enum nemu_link_migration_result nemu_link_channel_result(struct nemu_link_channel_policy *p,
                                                         const struct nemu_link_channel_config *cfg,
                                                         const struct nemu_link_hop_central *hop,
                                                         uint64_t now, bool both_up) {
    if (!p->validating) {
        return NEMU_LINK_MIGRATION_WAIT;
    }
    bool good = both_up && !hop->searching && hop->index == p->validation_index;
    for (uint8_t source = 0; source < NEMU_LINK_HOP_SOURCES; source++) {
        const struct nemu_link_peer_quality *q = &p->peers[source];
        good &= q->samples >= cfg->min_link_samples && q->loss_ewma < cfg->loss_threshold &&
                now - q->last_rx_ms < hop->timing.loss_ms;
    }
    if (good) {
        p->validating = false;
        p->cooldown_until = now + cfg->cooldown_ms;
        return NEMU_LINK_MIGRATION_OK;
    }
    if (now >= p->validation_deadline || hop->index != p->validation_index || hop->searching) {
        struct nemu_link_channel_scan *q = &p->channels[p->validation_index];
        q->failure_penalty = NEMU_LINK_QUALITY_SCALE;
        q->penalty_expiry = now + cfg->penalty_ms;
        p->validating = false;
        return NEMU_LINK_MIGRATION_FAILED;
    }
    return NEMU_LINK_MIGRATION_WAIT;
}
