#include <zmk/nemu_link/studio_transport.h>
#include <zmk/nemu_link/studio_rx.h>
#include <string.h>

bool nemu_link_studio_rx_metadata(struct nemu_link_studio_rx_progress *rx, uint8_t *metadata,
                                  size_t capacity, const struct nemu_link_studio_packet *p,
                                  uint64_t now) {
    if (p->offset > rx->metadata_received || (uint32_t)p->offset + p->length > capacity) {
        return false;
    }
    if (p->offset < rx->metadata_received) {
        return (uint32_t)p->offset + p->length <= rx->metadata_received &&
               !memcmp(metadata + p->offset, p->data, p->length);
    }
    memcpy(metadata + p->offset, p->data, p->length);
    rx->metadata_received = (uint16_t)(p->offset + p->length);
    rx->last_progress = now;
    return true;
}

static bool active(const struct nemu_link_studio_tx *tx) {
    return tx->phase >= NEMU_LINK_STUDIO_TX_META && tx->phase <= NEMU_LINK_STUDIO_TX_COMMIT;
}
static bool tx_tick(struct nemu_link_studio_tx *tx, uint64_t now) {
    if (active(tx) && (now < tx->last_progress || now - tx->last_progress >= tx->timeout_ms)) {
        tx->phase = NEMU_LINK_STUDIO_TX_FAILED;
        tx->error = NEMU_LINK_STUDIO_TIMEOUT;
    }
    return active(tx);
}
bool nemu_link_studio_tx_start(struct nemu_link_studio_tx *tx, uint16_t transaction,
                               const struct nemu_link_studio_version *version, const uint8_t *image,
                               uint64_t now, uint32_t retry_ms, uint32_t timeout_ms) {
    if (!tx || !version || !image || !transaction ||
        (tx->transaction && !nemu_link_studio_transaction_newer(transaction, tx->transaction)) ||
        active(tx) || !retry_ms || timeout_ms <= retry_ms) {
        return false;
    }
    uint8_t metadata[NEMU_LINK_STUDIO_VERSION_SIZE], checksum[32];
    if (!nemu_link_studio_version_encode(metadata, version)) {
        return false;
    }
    nemu_link_studio_checksum_bytes(nemu_link_studio_checksum(0, 0, image, version->size),
                                    checksum);
    if (memcmp(checksum, version->checksum, 32)) {
        return false;
    }
    *tx = (struct nemu_link_studio_tx){.phase = NEMU_LINK_STUDIO_TX_META,
                                       .transaction = transaction,
                                       .last_progress = now,
                                       .retry_ms = retry_ms,
                                       .timeout_ms = timeout_ms,
                                       .transfer_size = (uint16_t)version->size,
                                       .image = image,
                                       .version = *version};
    memcpy(tx->metadata, metadata, sizeof(metadata));
    return true;
}
static uint8_t amount(uint32_t total, uint16_t offset) {
    uint32_t remaining = total - offset;
    return remaining < NEMU_LINK_STUDIO_CHUNK_SIZE ? (uint8_t)remaining
                                                   : NEMU_LINK_STUDIO_CHUNK_SIZE;
}
static bool proposal(const struct nemu_link_studio_tx *tx, struct nemu_link_studio_packet *p) {
    *p = (struct nemu_link_studio_packet){.transaction = tx->transaction, .offset = tx->offset};
    if (tx->phase == NEMU_LINK_STUDIO_TX_META) {
        p->kind = tx->groups ? NEMU_LINK_STUDIO_GROUP_META
                             : (tx->delta ? NEMU_LINK_STUDIO_DELTA_META : NEMU_LINK_STUDIO_META);
        p->length =
            amount(tx->delta ? NEMU_LINK_STUDIO_DELTA_META_SIZE : NEMU_LINK_STUDIO_VERSION_SIZE,
                   tx->offset);
        memcpy(p->data, tx->metadata + tx->offset, p->length);
    } else if (tx->phase == NEMU_LINK_STUDIO_TX_DATA) {
        p->kind = tx->groups ? NEMU_LINK_STUDIO_GROUP_DATA : NEMU_LINK_STUDIO_DATA;
        p->length = amount(tx->transfer_size, tx->offset);
        if (tx->read) {
            if (!tx->read(tx->read_context, tx->offset, p->data, p->length)) {
                return false;
            }
        } else {
            memcpy(p->data, tx->image + tx->offset, p->length);
        }
    } else {
        p->kind = tx->groups ? NEMU_LINK_STUDIO_GROUP_COMMIT : NEMU_LINK_STUDIO_COMMIT;
        p->offset = 0;
    }
    return true;
}
bool nemu_link_studio_tx_next(struct nemu_link_studio_tx *tx, uint64_t now,
                              struct nemu_link_studio_packet *p) {
    if (!tx || !p || !tx_tick(tx, now) ||
        (tx->sent && now >= tx->last_sent && now - tx->last_sent < tx->retry_ms)) {
        return false;
    }
    if (!tx->prepared && !proposal(tx, &tx->proposal)) {
        tx->phase = NEMU_LINK_STUDIO_TX_FAILED;
        tx->error = NEMU_LINK_STUDIO_STALE;
        return false;
    }
    tx->prepared = true;
    *p = tx->proposal;
    return true;
}
void nemu_link_studio_tx_submitted(struct nemu_link_studio_tx *tx, uint64_t now) {
    if (tx->prepared && tx_tick(tx, now)) {
        tx->sent = true;
        tx->last_sent = now;
    }
}
bool nemu_link_studio_tx_receive(struct nemu_link_studio_tx *tx,
                                 const struct nemu_link_studio_packet *p, uint64_t now) {
    if (!tx || !nemu_link_studio_packet_valid(p) || !tx_tick(tx, now) || !tx->sent ||
        p->transaction != tx->transaction) {
        return false;
    }
    if (!tx->prepared) {
        return false;
    }
    const struct nemu_link_studio_packet sent = tx->proposal;
    uint32_t metadata_size =
        tx->delta ? NEMU_LINK_STUDIO_DELTA_META_SIZE : NEMU_LINK_STUDIO_VERSION_SIZE;
    enum nemu_link_studio_packet_kind meta_ack =
        tx->groups ? NEMU_LINK_STUDIO_GROUP_META_ACK : NEMU_LINK_STUDIO_META_ACK;
    enum nemu_link_studio_packet_kind data_ack =
        tx->groups ? NEMU_LINK_STUDIO_GROUP_DATA_ACK : NEMU_LINK_STUDIO_DATA_ACK;
    enum nemu_link_studio_packet_kind commit_ack =
        tx->groups ? NEMU_LINK_STUDIO_GROUP_COMMIT_ACK : NEMU_LINK_STUDIO_COMMIT_ACK;
    if (p->kind == NEMU_LINK_STUDIO_ERROR) {
        if (p->offset != sent.offset || p->data[1] != sent.kind) {
            return false;
        }
        tx->phase = NEMU_LINK_STUDIO_TX_FAILED;
        tx->error = (enum nemu_link_studio_result)p->data[0];
        return true;
    }
    uint32_t end = (uint32_t)sent.offset + sent.length;
    if (p->kind == commit_ack && p->offset == tx->version.size &&
        (tx->phase == NEMU_LINK_STUDIO_TX_COMMIT ||
         (tx->phase == NEMU_LINK_STUDIO_TX_META && end == metadata_size))) {
        tx->phase = NEMU_LINK_STUDIO_TX_DONE;
    } else if (tx->phase == NEMU_LINK_STUDIO_TX_META && p->kind == meta_ack && p->offset == end) {
        tx->offset = p->offset;
        if (tx->offset == metadata_size) {
            tx->phase = NEMU_LINK_STUDIO_TX_DATA;
            tx->offset = 0;
        }
    } else if (tx->phase == NEMU_LINK_STUDIO_TX_DATA && p->kind == data_ack && p->offset == end) {
        tx->offset = p->offset;
        if (tx->offset == tx->transfer_size) {
            tx->phase = NEMU_LINK_STUDIO_TX_COMMIT;
            tx->offset = 0;
        }
    } else {
        return false;
    }
    tx->last_progress = now;
    tx->sent = false;
    tx->prepared = false;
    return true;
}

bool nemu_link_studio_transfer_rx_init(struct nemu_link_studio_transfer_rx *rx, uint8_t *staging,
                                       size_t capacity, uint32_t timeout_ms) {
    if (!rx) {
        return false;
    }
    *rx = (struct nemu_link_studio_transfer_rx){0};
    return nemu_link_studio_rx_init(&rx->image, staging, capacity, timeout_ms);
}
void nemu_link_studio_transfer_rx_tick(struct nemu_link_studio_transfer_rx *rx, uint64_t now) {
    if (rx->transaction && !rx->image.receipt &&
        (now < rx->last_progress || now - rx->last_progress >= rx->image.timeout_ms)) {
        rx->expired = true;
        nemu_link_studio_rx_abort(&rx->image);
    }
    if (nemu_link_studio_rx_tick(&rx->image, now) == NEMU_LINK_STUDIO_TIMEOUT) {
        rx->expired = true;
    }
}
static void error(struct nemu_link_studio_packet *reply, enum nemu_link_studio_result result) {
    reply->kind = NEMU_LINK_STUDIO_ERROR;
    reply->length = 2;
    reply->data[0] = (uint8_t)result;
}
bool nemu_link_studio_transfer_receive(struct nemu_link_studio_transfer_rx *rx,
                                       const struct nemu_link_studio_packet *p,
                                       struct nemu_link_studio_version *live, uint64_t now,
                                       nemu_link_studio_validate_fn validate,
                                       nemu_link_studio_persist_fn persist, void *context,
                                       struct nemu_link_studio_packet *reply) {
    if (!rx || !live || !reply || !nemu_link_studio_packet_valid(p) ||
        (p->kind != NEMU_LINK_STUDIO_META && p->kind != NEMU_LINK_STUDIO_DATA &&
         p->kind != NEMU_LINK_STUDIO_COMMIT)) {
        return false;
    }
    nemu_link_studio_transfer_rx_tick(rx, now);
    *reply = (struct nemu_link_studio_packet){
        .transaction = p->transaction, .offset = p->offset, .data = {0, (uint8_t)p->kind}};
    if (p->transaction != rx->transaction) {
        if (!nemu_link_studio_transaction_newer(p->transaction, rx->transaction) ||
            p->kind != NEMU_LINK_STUDIO_META || p->offset) {
            error(reply, NEMU_LINK_STUDIO_STALE);
            return true;
        }
        nemu_link_studio_rx_abort(&rx->image);
        rx->transaction = p->transaction;
        rx->metadata_received = 0;
        rx->last_progress = now;
        rx->expired = false;
    }
    if (rx->expired) {
        error(reply, NEMU_LINK_STUDIO_TIMEOUT);
        return true;
    }
    if (p->kind == NEMU_LINK_STUDIO_META) {
        if (p->offset > rx->metadata_received) {
            error(reply, NEMU_LINK_STUDIO_INVALID);
            return true;
        }
        if (p->offset < rx->metadata_received) {
            if ((uint32_t)p->offset + p->length > rx->metadata_received ||
                memcmp(rx->metadata + p->offset, p->data, p->length)) {
                error(reply, NEMU_LINK_STUDIO_INVALID);
                return true;
            }
        } else {
            memcpy(rx->metadata + p->offset, p->data, p->length);
            rx->metadata_received = (uint16_t)(rx->metadata_received + p->length);
            rx->last_progress = now;
        }
        if (rx->metadata_received == NEMU_LINK_STUDIO_VERSION_SIZE) {
            struct nemu_link_studio_version target;
            if (!nemu_link_studio_version_decode(&target, rx->metadata)) {
                rx->expired = true;
                error(reply, NEMU_LINK_STUDIO_INVALID);
                return true;
            }
            enum nemu_link_studio_result result =
                nemu_link_studio_rx_begin(&rx->image, rx->transaction, live, &target, now);
            if (result == NEMU_LINK_STUDIO_ALREADY_COMMITTED) {
                reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
                reply->offset = (uint16_t)target.size;
                return true;
            }
            if (result != NEMU_LINK_STUDIO_OK && result != NEMU_LINK_STUDIO_DUPLICATE) {
                rx->expired = true;
                error(reply, result);
                return true;
            }
        }
        reply->kind = NEMU_LINK_STUDIO_META_ACK;
        reply->offset = rx->metadata_received;
        return true;
    }
    if (rx->metadata_received != NEMU_LINK_STUDIO_VERSION_SIZE) {
        error(reply, NEMU_LINK_STUDIO_STALE);
        return true;
    }
    if (p->kind == NEMU_LINK_STUDIO_DATA) {
        /* A reordered data retry after COMMIT must not manufacture a stale error
         * that defeats the queued COMMIT_ACK. A newer live version still wins. */
        if (rx->image.receipt && nemu_link_studio_version_equal(live, &rx->image.target) &&
            (uint32_t)p->offset + p->length <= rx->image.received &&
            memcmp(rx->image.staging + p->offset, p->data, p->length) == 0) {
            reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
            reply->offset = (uint16_t)live->size;
            return true;
        }
        enum nemu_link_studio_result result = nemu_link_studio_rx_chunk(
            &rx->image, rx->transaction, p->offset, p->data, p->length, now);
        if (result != NEMU_LINK_STUDIO_OK && result != NEMU_LINK_STUDIO_DUPLICATE) {
            error(reply, result);
            return true;
        }
        if (result == NEMU_LINK_STUDIO_OK) {
            rx->last_progress = now;
        }
        reply->kind = NEMU_LINK_STUDIO_DATA_ACK;
        reply->offset = (uint16_t)rx->image.received;
        return true;
    }
    enum nemu_link_studio_result result = nemu_link_studio_rx_commit(
        &rx->image, rx->transaction, live, now, validate, persist, context);
    if (result == NEMU_LINK_STUDIO_OK || result == NEMU_LINK_STUDIO_ALREADY_COMMITTED) {
        reply->kind = NEMU_LINK_STUDIO_COMMIT_ACK;
        reply->offset = (uint16_t)live->size;
    } else {
        error(reply, result);
    }
    return true;
}

bool nemu_link_studio_tx_start_reader(struct nemu_link_studio_tx *tx, uint16_t transaction,
                                      const struct nemu_link_studio_version *version,
                                      nemu_link_studio_read_fn read, void *context, uint64_t now,
                                      uint32_t retry_ms, uint32_t timeout_ms) {
    if (!tx || !version || !read || !transaction ||
        (tx->transaction && !nemu_link_studio_transaction_newer(transaction, tx->transaction)) ||
        active(tx) || !retry_ms || timeout_ms <= retry_ms) {
        return false;
    }
    uint8_t metadata[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (!nemu_link_studio_version_encode(metadata, version)) {
        return false;
    }
    *tx = (struct nemu_link_studio_tx){.phase = NEMU_LINK_STUDIO_TX_META,
                                       .transaction = transaction,
                                       .last_progress = now,
                                       .retry_ms = retry_ms,
                                       .timeout_ms = timeout_ms,
                                       .transfer_size = (uint16_t)version->size,
                                       .read = read,
                                       .read_context = context,
                                       .version = *version};
    memcpy(tx->metadata, metadata, sizeof(metadata));
    return true;
}

bool nemu_link_studio_tx_start_groups_reader(struct nemu_link_studio_tx *tx, uint16_t transaction,
                                             const struct nemu_link_studio_version *version,
                                             nemu_link_studio_read_fn read, void *context,
                                             uint64_t now, uint32_t retry_ms, uint32_t timeout_ms) {
    if (!nemu_link_studio_tx_start_reader(tx, transaction, version, read, context, now, retry_ms,
                                          timeout_ms)) {
        return false;
    }
    tx->groups = true;
    return true;
}

bool nemu_link_studio_tx_start_delta(struct nemu_link_studio_tx *tx, uint16_t transaction,
                                     const struct nemu_link_studio_version *version,
                                     const struct nemu_link_studio_version *base, uint16_t size,
                                     nemu_link_studio_read_fn read, void *context, uint64_t now,
                                     uint32_t retry_ms, uint32_t timeout_ms) {
    uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (!size || !base || !version || memcmp(base->schema, version->schema, 32) ||
        !nemu_link_studio_version_encode(bytes, base) ||
        !nemu_link_studio_tx_start_reader(tx, transaction, version, read, context, now, retry_ms,
                                          timeout_ms)) {
        return false;
    }
    tx->delta = true;
    tx->transfer_size = size;
    memcpy(tx->metadata + NEMU_LINK_STUDIO_VERSION_SIZE, bytes, sizeof(bytes));
    tx->metadata[2 * NEMU_LINK_STUDIO_VERSION_SIZE] = (uint8_t)size;
    tx->metadata[2 * NEMU_LINK_STUDIO_VERSION_SIZE + 1] = (uint8_t)(size >> 8);
    return true;
}
