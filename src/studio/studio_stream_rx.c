#include <zmk/nemu_link/studio_stream.h>
#include <string.h>
static void error(struct nemu_link_studio_packet *reply, enum nemu_link_studio_result value) {
    reply->kind = NEMU_LINK_STUDIO_ERROR;
    reply->length = 2;
    reply->data[0] = (uint8_t)value;
}
/* Entries may cross chunk boundaries. Strictly increasing indices prevent duplicate
 * record application; chunk retransmissions are checked before reaching this parser. */
static bool delta_feed(struct nemu_link_studio_stream_rx *rx,
                       const struct nemu_link_studio_schema *schema, const uint8_t *bytes,
                       size_t size, nemu_link_studio_record_fn apply, void *context) {
    while (size--) {
        if (rx->delta_used >= sizeof(rx->delta_buffer)) {
            return false;
        }
        rx->delta_buffer[rx->delta_used++] = *bytes++;
        if (rx->delta_used < 3) {
            continue;
        }
        uint16_t index =
            (uint16_t)((uint16_t)rx->delta_buffer[0] | (uint16_t)rx->delta_buffer[1] << 8);
        size_t length = rx->delta_buffer[2];
        if (!length || length > NEMU_LINK_STUDIO_RECORD_MAX ||
            index >= nemu_link_studio_record_count(schema) ||
            (!rx->delta_have_previous && index != 0) ||
            (rx->delta_have_previous && index <= rx->delta_previous)) {
            return false;
        }
        if (rx->delta_used == length + 3) {
            if (!nemu_link_studio_record_valid(schema, index, rx->delta_buffer + 3, length) ||
                !apply(context, index, rx->delta_buffer + 3, length)) {
                return false;
            }
            rx->delta_previous = index;
            rx->delta_have_previous = true;
            rx->delta_used = 0;
        }
    }
    return true;
}
bool nemu_link_studio_stream_receive(struct nemu_link_studio_stream_rx *rx,
                                     const struct nemu_link_studio_packet *p,
                                     struct nemu_link_studio_version *live,
                                     const struct nemu_link_studio_schema *schema, uint64_t now,
                                     nemu_link_studio_record_fn apply,
                                     nemu_link_studio_version_fn commit, void *context,
                                     struct nemu_link_studio_packet *reply) {
    if (!nemu_link_studio_packet_valid(p) ||
        (p->kind != NEMU_LINK_STUDIO_META && p->kind != NEMU_LINK_STUDIO_DELTA_META &&
         p->kind != NEMU_LINK_STUDIO_DATA && p->kind != NEMU_LINK_STUDIO_COMMIT)) {
        return false;
    }
    *reply = (struct nemu_link_studio_packet){
        .transaction = p->transaction, .offset = p->offset, .data = {0, (uint8_t)p->kind}};
    rx->offered = false;
    if (p->transaction != rx->transaction) {
        if (!nemu_link_studio_transaction_newer(p->transaction, rx->transaction) ||
            (p->kind != NEMU_LINK_STUDIO_META && p->kind != NEMU_LINK_STUDIO_DELTA_META) ||
            p->offset) {
            error(reply, NEMU_LINK_STUDIO_STALE);
            return true;
        }
        *rx = (struct nemu_link_studio_stream_rx){.transaction = p->transaction,
                                                  .last_progress = now,
                                                  .delta = p->kind == NEMU_LINK_STUDIO_DELTA_META};
    }
    if (!rx->receipt && now - rx->last_progress >= 10000) {
        rx->expired = true;
    }
    if (rx->expired) {
        error(reply, NEMU_LINK_STUDIO_TIMEOUT);
        return true;
    }
    size_t metadata_size =
        rx->delta ? NEMU_LINK_STUDIO_DELTA_META_SIZE : NEMU_LINK_STUDIO_VERSION_SIZE;
    if (p->kind == NEMU_LINK_STUDIO_META || p->kind == NEMU_LINK_STUDIO_DELTA_META) {
        if ((p->kind == NEMU_LINK_STUDIO_DELTA_META) != rx->delta) {
            error(reply, NEMU_LINK_STUDIO_INVALID);
            return true;
        }
        if (!nemu_link_studio_rx_metadata(&rx->progress, rx->metadata, metadata_size, p, now)) {
            error(reply, NEMU_LINK_STUDIO_INVALID);
            return true;
        }
        if (rx->metadata_received == metadata_size) {
            if (!nemu_link_studio_version_decode(&rx->target, rx->metadata)) {
                rx->expired = true;
                error(reply, NEMU_LINK_STUDIO_INVALID);
                return true;
            }
            rx->offered = true;
            enum nemu_link_studio_plan plan =
                nemu_link_studio_plan(NEMU_LINK_STUDIO_KEYBOARD, live, &rx->target);
            if (plan == NEMU_LINK_STUDIO_EQUAL) {
                rx->receipt = true;
                rx->base = *live;
                reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
                reply->offset = (uint16_t)rx->target.size;
                return true;
            }
            if (plan != NEMU_LINK_STUDIO_RECEIVE) {
                rx->expired = true;
                error(reply, NEMU_LINK_STUDIO_STALE);
                return true;
            }
            if (rx->delta) {
                struct nemu_link_studio_version base;
                uint16_t size =
                    (uint16_t)((uint16_t)rx->metadata[2 * NEMU_LINK_STUDIO_VERSION_SIZE] |
                               (uint16_t)rx->metadata[2 * NEMU_LINK_STUDIO_VERSION_SIZE + 1] << 8);
                if (!size ||
                    !nemu_link_studio_version_decode(&base, rx->metadata +
                                                                NEMU_LINK_STUDIO_VERSION_SIZE) ||
                    !nemu_link_studio_version_equal(live, &base) ||
                    memcmp(base.schema, rx->target.schema, 32)) {
                    rx->expired = true;
                    error(reply, NEMU_LINK_STUDIO_STALE);
                    return true;
                }
                rx->transfer_size = size;
            } else {
                rx->transfer_size = (uint16_t)rx->target.size;
            }
            if (!rx->started) {
                rx->base = *live;
                rx->started = true;
                nemu_link_studio_stream_init(&rx->stream, schema, apply, context);
            }
        }
        reply->kind = NEMU_LINK_STUDIO_META_ACK;
        reply->offset = rx->metadata_received;
        return true;
    }
    if (rx->receipt && nemu_link_studio_version_equal(live, &rx->base)) {
        reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
        reply->offset = (uint16_t)rx->target.size;
        return true;
    }
    if (!rx->started || !nemu_link_studio_version_equal(live, &rx->base)) {
        rx->expired = true;
        error(reply, NEMU_LINK_STUDIO_STALE);
        return true;
    }
    if (p->kind == NEMU_LINK_STUDIO_DATA) {
        if (p->offset == rx->received && (uint32_t)p->offset + p->length <= rx->transfer_size) {
            if (!(rx->delta ? delta_feed(rx, schema, p->data, p->length, apply, context)
                            : nemu_link_studio_stream_feed(&rx->stream, p->data, p->length))) {
                rx->expired = true;
                error(reply, NEMU_LINK_STUDIO_STORAGE_FAILED);
                return true;
            }
            rx->received = (uint16_t)(p->offset + p->length);
            rx->last = *p;
            rx->last_progress = now;
        } else if (p->offset != rx->last.offset || p->length != rx->last.length ||
                   rx->last.kind != NEMU_LINK_STUDIO_DATA ||
                   memcmp(p->data, rx->last.data, p->length)) {
            error(reply, NEMU_LINK_STUDIO_INVALID);
            return true;
        }
        reply->kind = NEMU_LINK_STUDIO_DATA_ACK;
        reply->offset = rx->received;
        return true;
    }
    if (rx->received != rx->transfer_size ||
        (rx->delta ? (rx->delta_used || !rx->delta_have_previous)
                   : !nemu_link_studio_stream_finish(&rx->stream, rx->target.size,
                                                     rx->target.checksum))) {
        rx->expired = true;
        error(reply, NEMU_LINK_STUDIO_INVALID);
        return true;
    }
    struct nemu_link_studio_version applied = rx->target;
    if (!commit(context, &applied)) {
        rx->expired = true;
        error(reply, NEMU_LINK_STUDIO_STORAGE_FAILED);
        return true;
    }
    *live = applied;
    rx->base = applied;
    rx->receipt = true;
    rx->started = false;
    reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
    reply->offset = (uint16_t)rx->target.size;
    return true;
}
