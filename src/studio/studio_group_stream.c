#include <zmk/nemu_link/studio_group_stream.h>

#include <string.h>
#include <zephyr/sys/util.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
#include <zmk/dynamic_settings/schema.h>
#endif

static bool shared_contract_matches(const struct nemu_link_studio_version *version) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    return !memcmp(version->schema, dynamic_settings_shared_contract_sha256,
                   sizeof(version->schema));
#else
    ARG_UNUSED(version);
    return true;
#endif
}

static uint32_t get_u32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

static uint16_t get_u16(const uint8_t *bytes) {
    return (uint16_t)bytes[0] | (uint16_t)bytes[1] << 8;
}

static void error(struct nemu_link_studio_packet *reply, enum nemu_link_studio_result result,
                  enum nemu_link_studio_packet_kind request_kind) {
    reply->kind = NEMU_LINK_STUDIO_ERROR;
    reply->length = 2;
    reply->data[0] = result;
    reply->data[1] = request_kind;
}

bool nemu_link_studio_group_rx_init(struct nemu_link_studio_group_rx *rx, uint8_t *record,
                                    size_t record_capacity) {
    if (!rx || !record || !record_capacity) {
        return false;
    }
    *rx = (struct nemu_link_studio_group_rx){.record = record, .record_capacity = record_capacity};
    return true;
}

static bool apply_bytes(struct nemu_link_studio_group_rx *rx, const uint8_t *bytes, size_t size,
                        size_t group_count, nemu_link_studio_group_id_fn group_id,
                        nemu_link_studio_group_apply_fn apply, void *context) {
    while (size--) {
        if (rx->group_header_used < sizeof(rx->group_header)) {
            rx->group_header[rx->group_header_used++] = *bytes++;
            if (rx->group_header_used == sizeof(rx->group_header)) {
                rx->group_id = get_u32(rx->group_header);
                rx->record_size = get_u16(rx->group_header + 4);
                rx->record_used = 0;
                if (rx->group_index >= group_count ||
                    group_id(context, rx->group_index) != rx->group_id || !rx->record_size ||
                    rx->record_size > rx->record_capacity) {
                    return false;
                }
            }
            continue;
        }
        rx->record[rx->record_used++] = *bytes++;
        if (rx->record_used == rx->record_size) {
            if (!apply(context, rx->group_id, rx->record, rx->record_size)) {
                return false;
            }
            rx->group_index++;
            rx->group_header_used = 0;
            rx->record_size = 0;
            rx->record_used = 0;
        }
    }
    return true;
}

bool nemu_link_studio_group_receive(struct nemu_link_studio_group_rx *rx,
                                    const struct nemu_link_studio_packet *p,
                                    struct nemu_link_studio_version *live,
                                    enum nemu_link_studio_side side, size_t group_count,
                                    nemu_link_studio_group_id_fn group_id,
                                    nemu_link_studio_group_apply_fn apply,
                                    nemu_link_studio_group_commit_fn commit, void *context,
                                    uint64_t now, struct nemu_link_studio_packet *reply) {
    if (!rx || !live || !group_id || !apply || !commit || !reply ||
        !nemu_link_studio_packet_valid(p) ||
        (p->kind != NEMU_LINK_STUDIO_GROUP_META && p->kind != NEMU_LINK_STUDIO_GROUP_DATA &&
         p->kind != NEMU_LINK_STUDIO_GROUP_COMMIT)) {
        return false;
    }
    *reply = (struct nemu_link_studio_packet){
        .transaction = p->transaction,
        .offset = p->offset,
        .data = {0, (uint8_t)p->kind},
    };
    if (rx->transaction != p->transaction) {
        if (!nemu_link_studio_transaction_newer(p->transaction, rx->transaction) ||
            p->kind != NEMU_LINK_STUDIO_GROUP_META || p->offset) {
            error(reply, NEMU_LINK_STUDIO_STALE, p->kind);
            return true;
        }
        uint8_t *record = rx->record;
        size_t capacity = rx->record_capacity;
        *rx = (struct nemu_link_studio_group_rx){
            .record = record,
            .record_capacity = capacity,
            .transaction = p->transaction,
            .last_progress = now,
        };
    }
    if (!rx->receipt && (now < rx->last_progress || now - rx->last_progress >= 10000)) {
        rx->expired = true;
    }
    if (rx->expired) {
        error(reply, NEMU_LINK_STUDIO_TIMEOUT, p->kind);
        return true;
    }
    if (p->kind == NEMU_LINK_STUDIO_GROUP_META) {
        if (!nemu_link_studio_rx_metadata(&rx->progress, rx->metadata, sizeof(rx->metadata), p,
                                          now)) {
            error(reply, NEMU_LINK_STUDIO_INVALID, p->kind);
            return true;
        }
        if (rx->metadata_received == sizeof(rx->metadata) && !rx->started && !rx->receipt) {
            if (!nemu_link_studio_version_decode(&rx->target, rx->metadata)) {
                rx->expired = true;
                error(reply, NEMU_LINK_STUDIO_INVALID, p->kind);
                return true;
            }
            enum nemu_link_studio_plan plan =
                shared_contract_matches(live) && shared_contract_matches(&rx->target)
                    ? nemu_link_studio_plan(side, live, &rx->target)
                    : NEMU_LINK_STUDIO_INCOMPATIBLE;
            if (plan == NEMU_LINK_STUDIO_EQUAL) {
                rx->base = *live;
                rx->receipt = true;
                reply->kind = NEMU_LINK_STUDIO_GROUP_COMMIT_ACK;
                reply->offset = (uint16_t)rx->target.size;
                return true;
            }
            if (plan != NEMU_LINK_STUDIO_RECEIVE) {
                rx->expired = true;
                error(reply,
                      plan == NEMU_LINK_STUDIO_INCOMPATIBLE ? NEMU_LINK_STUDIO_INVALID
                                                            : NEMU_LINK_STUDIO_STALE,
                      p->kind);
                return true;
            }
            rx->base = *live;
            rx->group_count = group_count;
            rx->started = true;
        }
        reply->kind = NEMU_LINK_STUDIO_GROUP_META_ACK;
        reply->offset = rx->metadata_received;
        return true;
    }
    if (rx->receipt && nemu_link_studio_version_equal(live, &rx->base)) {
        reply->kind = NEMU_LINK_STUDIO_GROUP_COMMIT_ACK;
        reply->offset = (uint16_t)rx->target.size;
        return true;
    }
    if (!rx->started || rx->metadata_received != sizeof(rx->metadata) ||
        !nemu_link_studio_version_equal(live, &rx->base)) {
        error(reply, NEMU_LINK_STUDIO_STALE, p->kind);
        return true;
    }
    if (p->kind == NEMU_LINK_STUDIO_GROUP_DATA) {
        if (p->offset < rx->received && p->offset == rx->last.offset &&
            p->length == rx->last.length && rx->last.kind == p->kind &&
            !memcmp(p->data, rx->last.data, p->length)) {
            reply->kind = NEMU_LINK_STUDIO_GROUP_DATA_ACK;
            reply->offset = rx->received;
            return true;
        }
        if (p->offset != rx->received || (uint32_t)rx->received + p->length > rx->target.size ||
            !apply_bytes(rx, p->data, p->length, group_count, group_id, apply, context)) {
            rx->expired = true;
            error(reply, NEMU_LINK_STUDIO_STORAGE_FAILED, p->kind);
            return true;
        }
        rx->checksum = nemu_link_studio_checksum(rx->checksum, rx->received, p->data, p->length);
        rx->received = (uint16_t)(rx->received + p->length);
        rx->last = *p;
        rx->last_progress = now;
        reply->kind = NEMU_LINK_STUDIO_GROUP_DATA_ACK;
        reply->offset = rx->received;
        return true;
    }
    uint8_t checksum[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE];
    nemu_link_studio_checksum_bytes(rx->checksum, checksum);
    if (rx->received != rx->target.size || rx->group_index != rx->group_count ||
        rx->group_header_used || rx->record_used || memcmp(checksum, rx->target.checksum, 32)) {
        rx->expired = true;
        error(reply, NEMU_LINK_STUDIO_INVALID, p->kind);
        return true;
    }
    struct nemu_link_studio_version applied = rx->target;
    if (!commit(context, &applied)) {
        rx->expired = true;
        error(reply, NEMU_LINK_STUDIO_STORAGE_FAILED, p->kind);
        return true;
    }
    *live = applied;
    rx->base = applied;
    rx->receipt = true;
    rx->started = false;
    reply->kind = NEMU_LINK_STUDIO_GROUP_COMMIT_ACK;
    reply->offset = (uint16_t)rx->target.size;
    return true;
}
