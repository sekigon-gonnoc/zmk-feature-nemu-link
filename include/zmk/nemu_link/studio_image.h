#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NEMU_LINK_STUDIO_MAX_LAYERS 32
#define NEMU_LINK_STUDIO_MAX_POSITIONS 128
#define NEMU_LINK_STUDIO_MAX_NAME 63

struct nemu_link_studio_behavior {
    const char *name; /* Shared schema name, not a build-local behavior ID. */
    uint8_t params;
};
struct nemu_link_studio_schema {
    uint8_t layers;
    uint16_t positions, layouts;
    const struct nemu_link_studio_behavior *behaviors;
    size_t behavior_count;
};
struct nemu_link_studio_binding {
    const char *behavior;
    uint32_t param1, param2;
};
struct nemu_link_studio_layer {
    const char *name; /* UTF-8, at most MAX_NAME bytes, no embedded NUL. */
    const struct nemu_link_studio_binding *bindings;
};
struct nemu_link_studio_image {
    uint16_t layout;
    uint8_t active_layers;
    const uint8_t *order;                        /* Unique stable IDs; inactive layers omitted. */
    const struct nemu_link_studio_layer *layers; /* ALL stable layer IDs, in ID order. */
};

/* DSC1: magic[4], layout:u16, layers:u8, active:u8, positions:u16,
 * order[active], then for each layer name_len:u8, name[], then each binding
 * behavior_len:u8, behavior[], param1:u32, param2:u32. All integers little-endian.
 * Stream import maps numeric layer/position IDs into local capacity and drops
 * overflow. Full-image helpers still require identical dimensions.
 * No C struct padding, NVS IDs, runtime state, bonds, or host profiles. */
bool nemu_link_studio_image_encode(const struct nemu_link_studio_schema *schema,
                                   const struct nemu_link_studio_image *image, uint8_t *bytes,
                                   size_t capacity, size_t *size);
bool nemu_link_studio_image_validate(const struct nemu_link_studio_schema *schema,
                                     const uint8_t *bytes, size_t size);

/* The schema itself has a deterministic DSH1 encoding before XOR32. Include
 * ALL layout definitions (geometry, transforms and stock map), stock layer
 * definitions, behavior configuration and protocol revisions in build_contract.
 * It must be shared semantic data, not role-specific generated IDs/paths.
 * Build-time devicetree extraction and generated headers remain adapter work. */
bool nemu_link_studio_schema_checksum(const struct nemu_link_studio_schema *schema,
                                      const uint8_t build_contract[32], uint8_t *scratch,
                                      size_t capacity, uint8_t checksum[32]);
