#include <zmk/nemu_link/studio_sync.h>
#include <string.h>

static bool compatible(const struct nemu_link_studio_version *a,
                       const struct nemu_link_studio_version *b) {
    return memcmp(a->schema, b->schema, sizeof(a->schema)) == 0;
}

bool nemu_link_studio_version_equal(const struct nemu_link_studio_version *a,
                                    const struct nemu_link_studio_version *b) {
    return compatible(a, b) && a->clock[0] == b->clock[0] && a->clock[1] == b->clock[1] &&
           a->size == b->size && memcmp(a->checksum, b->checksum, sizeof(a->checksum)) == 0;
}

enum nemu_link_studio_plan nemu_link_studio_plan(enum nemu_link_studio_side side,
                                                 const struct nemu_link_studio_version *local,
                                                 const struct nemu_link_studio_version *remote) {
    if ((side != NEMU_LINK_STUDIO_KEYBOARD && side != NEMU_LINK_STUDIO_DONGLE) ||
        ((!memcmp(local->schema + 28, "ESG1", 4) || !memcmp(remote->schema + 28, "ESG1", 4)) &&
         (memcmp(local->schema + 28, remote->schema + 28, 4) ||
          memcmp(local->schema, remote->schema, 4)))) {
        return NEMU_LINK_STUDIO_INCOMPATIBLE;
    }
    bool greater = local->clock[0] > remote->clock[0] || local->clock[1] > remote->clock[1];
    bool less = local->clock[0] < remote->clock[0] || local->clock[1] < remote->clock[1];
    if (!greater && !less) {
        /* Different builds retain different tails and checksums after clipping.
         * Equal revisions are the convergence criterion across those builds. */
        return memcmp(local->schema, remote->schema, sizeof(local->schema)) ||
                       nemu_link_studio_version_equal(local, remote)
                   ? NEMU_LINK_STUDIO_EQUAL
                   : NEMU_LINK_STUDIO_DIVERGED;
    }
    if (greater && less) {
        return side == NEMU_LINK_STUDIO_KEYBOARD ? NEMU_LINK_STUDIO_RESOLVE_KEYBOARD
                                                 : NEMU_LINK_STUDIO_WAIT_KEYBOARD;
    }
    return greater ? NEMU_LINK_STUDIO_SEND : NEMU_LINK_STUDIO_RECEIVE;
}

bool nemu_link_studio_saved(struct nemu_link_studio_version *next,
                            const struct nemu_link_studio_version *current,
                            enum nemu_link_studio_side side,
                            const uint8_t checksum[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE],
                            uint32_t size) {
    if ((side != NEMU_LINK_STUDIO_KEYBOARD && side != NEMU_LINK_STUDIO_DONGLE) || size == 0 ||
        current->clock[side] == UINT64_MAX) {
        return false;
    }
    struct nemu_link_studio_version result = *current;
    result.clock[side]++;
    result.size = size;
    memcpy(result.checksum, checksum, sizeof(result.checksum));
    *next = result;
    return true;
}

bool nemu_link_studio_resolve(struct nemu_link_studio_version *next,
                              const struct nemu_link_studio_version *keyboard,
                              const struct nemu_link_studio_version *dongle) {
    if (nemu_link_studio_plan(NEMU_LINK_STUDIO_KEYBOARD, keyboard, dongle) !=
            NEMU_LINK_STUDIO_RESOLVE_KEYBOARD ||
        keyboard->clock[0] == UINT64_MAX || dongle->clock[0] == UINT64_MAX) {
        return false;
    }
    struct nemu_link_studio_version result = *keyboard;
    result.clock[0] =
        (keyboard->clock[0] > dongle->clock[0] ? keyboard->clock[0] : dongle->clock[0]) + 1;
    result.clock[1] = keyboard->clock[1] > dongle->clock[1] ? keyboard->clock[1] : dongle->clock[1];
    *next = result;
    return true;
}

bool nemu_link_studio_rx_init(struct nemu_link_studio_rx *rx, uint8_t *staging, size_t capacity,
                              uint64_t timeout_ms) {
    if (!rx || !staging || capacity == 0 || timeout_ms == 0) {
        return false;
    }
    *rx = (struct nemu_link_studio_rx){
        .staging = staging, .capacity = capacity, .timeout_ms = timeout_ms};
    return true;
}

void nemu_link_studio_rx_abort(struct nemu_link_studio_rx *rx) {
    rx->active = false;
    rx->receipt = false;
    rx->received = 0;
}

enum nemu_link_studio_result nemu_link_studio_rx_tick(struct nemu_link_studio_rx *rx,
                                                      uint64_t now) {
    if (rx->active &&
        (now < rx->last_progress_ms || now - rx->last_progress_ms >= rx->timeout_ms)) {
        nemu_link_studio_rx_abort(rx);
        return NEMU_LINK_STUDIO_TIMEOUT;
    }
    return NEMU_LINK_STUDIO_OK;
}

enum nemu_link_studio_result
nemu_link_studio_rx_begin(struct nemu_link_studio_rx *rx, uint64_t transaction,
                          const struct nemu_link_studio_version *live,
                          const struct nemu_link_studio_version *target, uint64_t now) {
    (void)nemu_link_studio_rx_tick(rx, now);
    if (transaction == 0 || target->size == 0) {
        return NEMU_LINK_STUDIO_INVALID;
    }
    if (rx->active) {
        if (transaction != rx->transaction) {
            return NEMU_LINK_STUDIO_BUSY;
        }
        return nemu_link_studio_version_equal(&rx->target, target) &&
                       nemu_link_studio_version_equal(&rx->base, live)
                   ? NEMU_LINK_STUDIO_DUPLICATE
                   : NEMU_LINK_STUDIO_STALE;
    }
    if (rx->receipt && transaction == rx->transaction &&
        !nemu_link_studio_version_equal(&rx->target, target)) {
        return NEMU_LINK_STUDIO_STALE;
    }
    if (!compatible(live, target)) {
        return NEMU_LINK_STUDIO_STALE;
    }
    enum nemu_link_studio_plan plan =
        nemu_link_studio_plan(NEMU_LINK_STUDIO_KEYBOARD, live, target);
    if (plan == NEMU_LINK_STUDIO_EQUAL) {
        rx->target = *target;
        rx->transaction = transaction;
        rx->receipt = true;
        return NEMU_LINK_STUDIO_ALREADY_COMMITTED;
    }
    if (plan != NEMU_LINK_STUDIO_RECEIVE) {
        return NEMU_LINK_STUDIO_STALE;
    }
    if (target->size > rx->capacity) {
        return NEMU_LINK_STUDIO_NO_SPACE;
    }
    rx->base = *live;
    rx->target = *target;
    rx->transaction = transaction;
    rx->received = 0;
    rx->last_progress_ms = now;
    rx->active = true;
    rx->receipt = false;
    return NEMU_LINK_STUDIO_OK;
}

enum nemu_link_studio_result nemu_link_studio_rx_chunk(struct nemu_link_studio_rx *rx,
                                                       uint64_t transaction, uint32_t offset,
                                                       const uint8_t *bytes, size_t size,
                                                       uint64_t now) {
    enum nemu_link_studio_result result = nemu_link_studio_rx_tick(rx, now);
    if (result != NEMU_LINK_STUDIO_OK) {
        return result;
    }
    if (!rx->active || transaction != rx->transaction) {
        return NEMU_LINK_STUDIO_STALE;
    }
    if (!bytes || size == 0 || size > NEMU_LINK_STUDIO_CHUNK_SIZE || offset > rx->target.size ||
        size > rx->target.size - offset) {
        return NEMU_LINK_STUDIO_INVALID;
    }
    if (offset < rx->received && size <= rx->received - offset) {
        return memcmp(rx->staging + offset, bytes, size) == 0 ? NEMU_LINK_STUDIO_DUPLICATE
                                                              : NEMU_LINK_STUDIO_INVALID;
    }
    if (offset != rx->received) {
        return NEMU_LINK_STUDIO_INVALID;
    }
    memcpy(rx->staging + offset, bytes, size);
    rx->received += size;
    rx->last_progress_ms = now;
    return NEMU_LINK_STUDIO_OK;
}

enum nemu_link_studio_result
nemu_link_studio_rx_commit(struct nemu_link_studio_rx *rx, uint64_t transaction,
                           struct nemu_link_studio_version *live, uint64_t now,
                           nemu_link_studio_validate_fn validate,
                           nemu_link_studio_persist_fn persist, void *context) {
    enum nemu_link_studio_result result = nemu_link_studio_rx_tick(rx, now);
    if (result != NEMU_LINK_STUDIO_OK) {
        return result;
    }
    if (!rx->active) {
        return rx->receipt && transaction == rx->transaction &&
                       nemu_link_studio_version_equal(live, &rx->target)
                   ? NEMU_LINK_STUDIO_ALREADY_COMMITTED
                   : NEMU_LINK_STUDIO_STALE;
    }
    if (transaction != rx->transaction) {
        return NEMU_LINK_STUDIO_STALE;
    }
    if (!nemu_link_studio_version_equal(live, &rx->base)) {
        nemu_link_studio_rx_abort(rx);
        return NEMU_LINK_STUDIO_STALE;
    }
    if (rx->received != rx->target.size || !validate || !persist) {
        return NEMU_LINK_STUDIO_INVALID;
    }
    uint8_t checksum[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE];
    nemu_link_studio_checksum_bytes(nemu_link_studio_checksum(0, 0, rx->staging, rx->received),
                                    checksum);
    if (memcmp(checksum, rx->target.checksum, sizeof(checksum)) != 0 ||
        !validate(context, rx->staging, rx->received)) {
        nemu_link_studio_rx_abort(rx);
        return NEMU_LINK_STUDIO_INVALID;
    }
    if (!persist(context, &rx->target, rx->staging, rx->received)) {
        return NEMU_LINK_STUDIO_STORAGE_FAILED;
    }
    *live = rx->target;
    rx->active = false;
    rx->receipt = true;
    return NEMU_LINK_STUDIO_OK;
}
