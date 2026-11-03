#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* XOR32 LE in the first four bytes, zero padding keeps existing wire/settings offsets. */
#define NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE 32
#define NEMU_LINK_STUDIO_CHUNK_SIZE 24

enum nemu_link_studio_side { NEMU_LINK_STUDIO_KEYBOARD, NEMU_LINK_STUDIO_DONGLE };

/* Explicit wire encoding lives in studio_transport.h; never send this struct. */
struct nemu_link_studio_version {
    uint8_t schema[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE];
    uint64_t clock[2];
    uint8_t checksum[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE];
    uint32_t size;
};

enum nemu_link_studio_plan {
    NEMU_LINK_STUDIO_EQUAL,
    NEMU_LINK_STUDIO_SEND,
    NEMU_LINK_STUDIO_RECEIVE,
    NEMU_LINK_STUDIO_RESOLVE_KEYBOARD,
    NEMU_LINK_STUDIO_WAIT_KEYBOARD,
    NEMU_LINK_STUDIO_INCOMPATIBLE,
    NEMU_LINK_STUDIO_DIVERGED, /* Same revision but different content: storage/protocol error. */
};

bool nemu_link_studio_version_equal(const struct nemu_link_studio_version *a,
                                    const struct nemu_link_studio_version *b);
enum nemu_link_studio_plan nemu_link_studio_plan(enum nemu_link_studio_side local_side,
                                                 const struct nemu_link_studio_version *local,
                                                 const struct nemu_link_studio_version *remote);
/* Pure proposals. Persist content AND metadata before publishing the result. */
bool nemu_link_studio_saved(struct nemu_link_studio_version *next,
                            const struct nemu_link_studio_version *current,
                            enum nemu_link_studio_side side,
                            const uint8_t checksum[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE],
                            uint32_t size);
bool nemu_link_studio_resolve(struct nemu_link_studio_version *next,
                              const struct nemu_link_studio_version *keyboard,
                              const struct nemu_link_studio_version *dongle);
/* Start with checksum=0, offset=0; offset is the number of preceding image bytes. */
uint32_t nemu_link_studio_checksum(uint32_t checksum, size_t offset, const uint8_t *bytes,
                                   size_t size);
void nemu_link_studio_checksum_bytes(uint32_t checksum,
                                     uint8_t bytes[NEMU_LINK_STUDIO_CHECKSUM_FIELD_SIZE]);

enum nemu_link_studio_result {
    NEMU_LINK_STUDIO_OK,
    NEMU_LINK_STUDIO_DUPLICATE,
    NEMU_LINK_STUDIO_ALREADY_COMMITTED,
    NEMU_LINK_STUDIO_INVALID,
    NEMU_LINK_STUDIO_BUSY,
    NEMU_LINK_STUDIO_STALE,
    NEMU_LINK_STUDIO_TIMEOUT,
    NEMU_LINK_STUDIO_NO_SPACE,
    NEMU_LINK_STUDIO_STORAGE_FAILED,
};

typedef bool (*nemu_link_studio_validate_fn)(void *context, const uint8_t *bytes, size_t size);
/* Must durably replace the complete image + version atomically before true.
 * On false, old image + version must survive. A power cut does not return:
 * recovery must select the old or the complete new image + version together.
 * This portable core does NOT make ZMK's per-setting writes atomic. */
typedef bool (*nemu_link_studio_persist_fn)(void *context,
                                            const struct nemu_link_studio_version *version,
                                            const uint8_t *bytes, size_t size);

/* Caller serializes save/reset/receive/commit on one worker and authenticates/binds
 * frames to the current left<->dongle session. Right-side configuration is rejected
 * by the future adapter. Staging must not alias live data; no dynamic allocation. */
struct nemu_link_studio_rx {
    uint8_t *staging;
    size_t capacity;
    uint64_t timeout_ms, last_progress_ms, transaction;
    size_t received;
    struct nemu_link_studio_version base, target;
    bool active, receipt;
};

bool nemu_link_studio_rx_init(struct nemu_link_studio_rx *rx, uint8_t *staging, size_t capacity,
                              uint64_t timeout_ms);
void nemu_link_studio_rx_abort(struct nemu_link_studio_rx *rx);
enum nemu_link_studio_result
nemu_link_studio_rx_begin(struct nemu_link_studio_rx *rx, uint64_t transaction,
                          const struct nemu_link_studio_version *live,
                          const struct nemu_link_studio_version *target, uint64_t now);
enum nemu_link_studio_result nemu_link_studio_rx_chunk(struct nemu_link_studio_rx *rx,
                                                       uint64_t transaction, uint32_t offset,
                                                       const uint8_t *bytes, size_t size,
                                                       uint64_t now);
enum nemu_link_studio_result nemu_link_studio_rx_tick(struct nemu_link_studio_rx *rx, uint64_t now);
enum nemu_link_studio_result
nemu_link_studio_rx_commit(struct nemu_link_studio_rx *rx, uint64_t transaction,
                           struct nemu_link_studio_version *live, uint64_t now,
                           nemu_link_studio_validate_fn validate,
                           nemu_link_studio_persist_fn persist, void *context);
