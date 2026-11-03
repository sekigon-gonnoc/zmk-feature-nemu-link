#pragma once
#include <zmk/nemu_link/studio_image.h>
#include <zmk/nemu_link/studio_sync.h>
/* Largest DSC1 record: behavior length + 63 byte name + two u32 params.
 * Memory is independent of layer and position count. */
#define NEMU_LINK_STUDIO_RECORD_MAX 72
/* Called after each record validates. Direct writes are allowed; interrupted
 * transfers keep already applied records and do not advance the revision. */
typedef bool (*nemu_link_studio_record_fn)(void *, uint16_t, const uint8_t *, size_t);
struct nemu_link_studio_stream {
    const struct nemu_link_studio_schema *schema;
    struct nemu_link_studio_schema wire_schema; /* Sender dimensions, fixed-size state. */
    nemu_link_studio_record_fn record;
    void *context;
    uint32_t checksum, size;
    uint16_t index;
    uint8_t buffer[NEMU_LINK_STUDIO_RECORD_MAX];
    size_t used, needed;
    bool failed;
};
uint16_t nemu_link_studio_record_count(const struct nemu_link_studio_schema *schema);
bool nemu_link_studio_record_valid(const struct nemu_link_studio_schema *, uint16_t,
                                   const uint8_t *, size_t);
void nemu_link_studio_stream_init(struct nemu_link_studio_stream *,
                                  const struct nemu_link_studio_schema *,
                                  nemu_link_studio_record_fn, void *);
bool nemu_link_studio_stream_feed(struct nemu_link_studio_stream *, const uint8_t *, size_t);
bool nemu_link_studio_stream_finish(const struct nemu_link_studio_stream *, uint32_t,
                                    const uint8_t checksum[32]);

#include <zmk/nemu_link/studio_transport.h>
#include <zmk/nemu_link/studio_rx.h>
/* Commit may replace schema/checksum/size with the local projection, keeping clocks. */
typedef bool (*nemu_link_studio_version_fn)(void *, struct nemu_link_studio_version *);
struct nemu_link_studio_stream_rx {
    union {
        struct nemu_link_studio_rx_progress progress;
        struct {
            NEMU_LINK_STUDIO_RX_PROGRESS_FIELDS;
        };
    };
    struct nemu_link_studio_stream stream;
    uint8_t metadata[NEMU_LINK_STUDIO_DELTA_META_SIZE];
    uint16_t transfer_size, delta_previous;
    uint8_t delta_buffer[NEMU_LINK_STUDIO_RECORD_MAX + 3];
    size_t delta_used;
    bool offered, delta, delta_have_previous;
};
/* One record buffer + one retransmission chunk; no image-sized allocations.
 * Failed/interrupted transfers retain applied records without publishing target. */
bool nemu_link_studio_stream_receive(struct nemu_link_studio_stream_rx *,
                                     const struct nemu_link_studio_packet *,
                                     struct nemu_link_studio_version *,
                                     const struct nemu_link_studio_schema *, uint64_t,
                                     nemu_link_studio_record_fn, nemu_link_studio_version_fn,
                                     void *, struct nemu_link_studio_packet *);
