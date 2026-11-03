#include <zmk/nemu_link/studio_image.h>
#include <zmk/nemu_link/studio_stream.h>
#include <zmk/nemu_link/studio_sync.h>
#include <string.h>

struct cursor {
    uint8_t *out;
    const uint8_t *in;
    size_t size, pos;
    bool ok;
};

static void put(struct cursor *c, uint32_t v, unsigned int n) {
    if (!c->ok || n > c->size - c->pos) {
        c->ok = false;
        return;
    }
    for (unsigned int i = 0; i < n; i++) {
        c->out[c->pos++] = (uint8_t)(v >> (8 * i));
    }
}
static uint32_t get(struct cursor *c, unsigned int n) {
    if (!c->ok || n > c->size - c->pos) {
        c->ok = false;
        return 0;
    }
    uint32_t v = 0;
    for (unsigned int i = 0; i < n; i++) {
        v |= (uint32_t)c->in[c->pos++] << (8 * i);
    }
    return v;
}
static size_t bounded_len(const char *s) {
    if (!s) {
        return NEMU_LINK_STUDIO_MAX_NAME + 1;
    }
    size_t n = 0;
    while (n <= NEMU_LINK_STUDIO_MAX_NAME && s[n]) {
        n++;
    }
    return n;
}
static bool identifier(const uint8_t *s, size_t size) {
    if (!size || size > NEMU_LINK_STUDIO_MAX_NAME) {
        return false;
    }
    for (size_t i = 0; i < size; i++) {
        if (s[i] < 0x21 || s[i] > 0x7e) {
            return false;
        }
    }
    return true;
}
static bool utf8(const uint8_t *s, size_t size) {
    for (size_t i = 0; i < size;) {
        uint32_t ch = s[i++];
        if (ch < 0x80) {
            if (!ch) {
                return false;
            }
            continue;
        }
        unsigned int extra;
        uint32_t minimum;
        if (ch >= 0xc2 && ch <= 0xdf) {
            extra = 1;
            minimum = 0x80;
            ch &= 0x1f;
        } else if (ch >= 0xe0 && ch <= 0xef) {
            extra = 2;
            minimum = 0x800;
            ch &= 0x0f;
        } else if (ch >= 0xf0 && ch <= 0xf4) {
            extra = 3;
            minimum = 0x10000;
            ch &= 7;
        } else {
            return false;
        }
        if (extra > size - i) {
            return false;
        }
        for (unsigned int j = 0; j < extra; j++) {
            uint8_t b = s[i++];
            if ((b & 0xc0) != 0x80) {
                return false;
            }
            ch = (ch << 6) | (b & 0x3f);
        }
        if (ch < minimum || ch > 0x10ffff || (ch >= 0xd800 && ch <= 0xdfff)) {
            return false;
        }
    }
    return true;
}
static bool schema_valid(const struct nemu_link_studio_schema *s) {
    if (!s || !s->layers || s->layers > NEMU_LINK_STUDIO_MAX_LAYERS || !s->positions ||
        s->positions > NEMU_LINK_STUDIO_MAX_POSITIONS || !s->layouts || !s->behaviors ||
        !s->behavior_count || s->behavior_count > UINT16_MAX) {
        return false;
    }
    for (size_t i = 0; i < s->behavior_count; i++) {
        const struct nemu_link_studio_behavior *b = &s->behaviors[i];
        size_t n = bounded_len(b->name);
        if (!identifier((const uint8_t *)b->name, n) || b->params > 2) {
            return false;
        }
        /* Sorted identities give deterministic checksums across build-local ordering. */
        if (i && strcmp(s->behaviors[i - 1].name, b->name) >= 0) {
            return false;
        }
    }
    return true;
}
static int behavior_params(const struct nemu_link_studio_schema *s, const uint8_t *name, size_t n) {
    for (size_t i = 0; i < s->behavior_count; i++) {
        if (strlen(s->behaviors[i].name) == n && memcmp(s->behaviors[i].name, name, n) == 0) {
            return s->behaviors[i].params;
        }
    }
    return -1;
}
static void put_name(struct cursor *c, const char *name) {
    size_t n = bounded_len(name);
    if (n > NEMU_LINK_STUDIO_MAX_NAME) {
        c->ok = false;
        return;
    }
    put(c, (uint32_t)n, 1);
    for (size_t i = 0; i < n; i++) {
        put(c, (uint8_t)name[i], 1);
    }
}
static const uint8_t *get_name(struct cursor *c, size_t *n) {
    *n = get(c, 1);
    if (!c->ok || *n > NEMU_LINK_STUDIO_MAX_NAME || *n > c->size - c->pos) {
        c->ok = false;
        return NULL;
    }
    const uint8_t *name = c->in + c->pos;
    c->pos += *n;
    return name;
}

bool nemu_link_studio_image_validate(const struct nemu_link_studio_schema *schema,
                                     const uint8_t *bytes, size_t size) {
    if (!schema_valid(schema) || !bytes || size < 10 || memcmp(bytes, "DSC1", 4) != 0) {
        return false;
    }
    struct cursor c = {.in = bytes, .size = size, .pos = 4, .ok = true};
    uint32_t layout = get(&c, 2), layers = get(&c, 1), active = get(&c, 1), positions = get(&c, 2);
    if (layout >= schema->layouts || layers != schema->layers || !active || active > layers ||
        positions != schema->positions) {
        return false;
    }
    uint32_t seen = 0;
    for (uint32_t i = 0; i < active; i++) {
        uint32_t id = get(&c, 1);
        if (!c.ok || id >= layers || (seen & (UINT32_C(1) << id))) {
            return false;
        }
        seen |= UINT32_C(1) << id;
    }
    for (uint32_t l = 0; l < layers; l++) {
        size_t n;
        const uint8_t *name = get_name(&c, &n);
        if (!name || !utf8(name, n)) {
            return false;
        }
        for (uint32_t p = 0; p < positions; p++) {
            name = get_name(&c, &n);
            if (!name) {
                return false;
            }
            int params = behavior_params(schema, name, n);
            uint32_t p1 = get(&c, 4), p2 = get(&c, 4);
            if (params < 0 || !c.ok || (params < 1 && p1) || (params < 2 && p2)) {
                return false;
            }
        }
    }
    return c.ok && c.pos == size;
}

bool nemu_link_studio_image_encode(const struct nemu_link_studio_schema *schema,
                                   const struct nemu_link_studio_image *image, uint8_t *bytes,
                                   size_t capacity, size_t *size) {
    if (!size) {
        return false;
    }
    *size = 0;
    if (!schema_valid(schema) || !image || !bytes || !image->order || !image->layers ||
        image->active_layers > schema->layers || capacity < 10) {
        return false;
    }
    struct cursor c = {.out = bytes, .size = capacity, .ok = true};
    put(&c, 0x31435344, 4);
    put(&c, image->layout, 2);
    put(&c, schema->layers, 1);
    put(&c, image->active_layers, 1);
    put(&c, schema->positions, 2);
    for (size_t i = 0; i < image->active_layers; i++) {
        put(&c, image->order[i], 1);
    }
    for (size_t l = 0; l < schema->layers; l++) {
        put_name(&c, image->layers[l].name);
        if (!image->layers[l].bindings) {
            return false;
        }
        for (size_t p = 0; p < schema->positions; p++) {
            const struct nemu_link_studio_binding *binding = &image->layers[l].bindings[p];
            put_name(&c, binding->behavior);
            put(&c, binding->param1, 4);
            put(&c, binding->param2, 4);
        }
    }
    if (!c.ok || !nemu_link_studio_image_validate(schema, bytes, c.pos)) {
        return false;
    }
    *size = c.pos;
    return true;
}

bool nemu_link_studio_schema_checksum(const struct nemu_link_studio_schema *schema,
                                      const uint8_t build_contract[32], uint8_t *scratch,
                                      size_t capacity, uint8_t checksum[32]) {
    if (!schema_valid(schema) || !build_contract || !scratch || !checksum) {
        return false;
    }
    struct cursor c = {.out = scratch, .size = capacity, .ok = true};
    put(&c, 0x31485344, 4);
    for (size_t i = 0; i < 32; i++) {
        put(&c, build_contract[i], 1);
    }
    put(&c, schema->layers, 1);
    put(&c, schema->positions, 2);
    put(&c, schema->layouts, 2);
    put(&c, (uint32_t)schema->behavior_count, 2);
    for (size_t i = 0; i < schema->behavior_count; i++) {
        put_name(&c, schema->behaviors[i].name);
        put(&c, schema->behaviors[i].params, 1);
    }
    if (!c.ok) {
        return false;
    }
    nemu_link_studio_checksum_bytes(nemu_link_studio_checksum(0, 0, scratch, c.pos), checksum);
    return true;
}

uint16_t nemu_link_studio_record_count(const struct nemu_link_studio_schema *s) {
    return (uint16_t)(1u + s->layers * (1u + s->positions));
}
bool nemu_link_studio_record_valid(const struct nemu_link_studio_schema *s, uint16_t index,
                                   const uint8_t *b, size_t n) {
    if (!schema_valid(s) || !b || !n || index >= nemu_link_studio_record_count(s)) {
        return false;
    }
    if (!index) {
        if (n < 11 || memcmp(b, "DSC1", 4) || (b[4] | b[5] << 8) >= s->layouts ||
            b[6] != s->layers || !b[7] || b[7] > s->layers || n != 10u + b[7] ||
            (b[8] | b[9] << 8) != s->positions) {
            return false;
        }
        uint32_t seen = 0;
        for (size_t i = 10; i < n; i++) {
            if (b[i] >= s->layers || (seen & (UINT32_C(1) << b[i]))) {
                return false;
            }
            seen |= UINT32_C(1) << b[i];
        }
        return true;
    }
    if (b[0] > NEMU_LINK_STUDIO_MAX_NAME) {
        return false;
    }
    if ((index - 1) % (s->positions + 1) == 0) {
        return n == 1u + b[0] && utf8(b + 1, b[0]);
    }
    if (n != 9u + b[0]) {
        return false;
    }
    int params = behavior_params(s, b + 1, b[0]);
    if (params < 0) {
        return false;
    }
    for (size_t i = 1u + b[0] + 4u * (unsigned int)params; i < n; i++) {
        if (b[i]) {
            return false;
        }
    }
    return true;
}
void nemu_link_studio_stream_init(struct nemu_link_studio_stream *s,
                                  const struct nemu_link_studio_schema *schema,
                                  nemu_link_studio_record_fn record, void *context) {
    *s = (struct nemu_link_studio_stream){
        .schema = schema, .wire_schema = *schema, .record = record, .context = context};
}
/* Validate the wire record, then project its index/header into local capacity.
 * XOR was accumulated before this conversion, including every discarded byte. */
static bool stream_record(struct nemu_link_studio_stream *s) {
    uint8_t *b = s->buffer;
    uint16_t index = s->index;
    size_t size = s->used;
    if (!index) {
        unsigned int layout = (unsigned int)b[4] | (unsigned int)b[5] << 8;
        s->wire_schema.layers = b[6];
        s->wire_schema.positions = (uint16_t)(b[8] | b[9] << 8);
        if (layout == UINT16_MAX) {
            return false;
        }
        s->wire_schema.layouts = (uint16_t)(layout + 1);
        if (!nemu_link_studio_record_valid(&s->wire_schema, 0, b, size)) {
            return false;
        }
        unsigned int active = 0;
        for (unsigned int i = 0; i < b[7]; i++) {
            if (b[10 + i] < s->schema->layers) {
                b[10 + active++] = b[10 + i];
            }
        }
        /* Keep at least one usable local layer if all active remote IDs overflow. */
        if (!active) {
            b[10] = 0;
            active = 1;
        }
        if (layout >= s->schema->layouts) {
            return false;
        }
        b[6] = s->schema->layers;
        b[7] = (uint8_t)active;
        b[8] = (uint8_t)s->schema->positions;
        b[9] = (uint8_t)(s->schema->positions >> 8);
        size = 10u + active;
    } else {
        unsigned int layer = (index - 1u) / (s->wire_schema.positions + 1u);
        unsigned int item = (index - 1u) % (s->wire_schema.positions + 1u);
        if (b[0] > NEMU_LINK_STUDIO_MAX_NAME) {
            return false;
        }
        if (layer >= s->schema->layers || item > s->schema->positions) {
            /* An unknown behavior is harmless outside the receiving keymap. */
            return item ? size == 9u + b[0] && identifier(b + 1, b[0])
                        : size == 1u + b[0] && utf8(b + 1, b[0]);
        }
        index = (uint16_t)(1u + layer * (s->schema->positions + 1u) + item);
    }
    return nemu_link_studio_record_valid(s->schema, index, b, size) &&
           (!s->record || s->record(s->context, index, b, size));
}
bool nemu_link_studio_stream_feed(struct nemu_link_studio_stream *s, const uint8_t *bytes,
                                  size_t size) {
    if (s->failed || !schema_valid(s->schema) || (!bytes && size)) {
        return false;
    }
    while (size--) {
        if (s->index >= nemu_link_studio_record_count(&s->wire_schema) ||
            s->used >= sizeof(s->buffer)) {
            s->failed = true;
            return false;
        }
        uint8_t b = *bytes++;
        s->buffer[s->used++] = b;
        s->checksum = nemu_link_studio_checksum(s->checksum, s->size, &b, 1);
        s->size++;
        if (!s->index && s->used == 8) {
            s->needed = 10u + b;
        } else if (s->index && s->used == 1) {
            s->needed = ((s->index - 1) % (s->wire_schema.positions + 1) == 0 ? 1u : 9u) + b;
        }
        if (s->needed > sizeof(s->buffer)) {
            s->failed = true;
            return false;
        }
        if (s->needed && s->used == s->needed) {
            if (!stream_record(s)) {
                s->failed = true;
                return false;
            }
            s->index++;
            s->used = 0;
            s->needed = 0;
        }
    }
    return true;
}
bool nemu_link_studio_stream_finish(const struct nemu_link_studio_stream *s, uint32_t size,
                                    const uint8_t checksum[32]) {
    if (s->failed || s->used || s->index != nemu_link_studio_record_count(&s->wire_schema) ||
        s->size != size) {
        return false;
    }
    uint8_t digest[32];
    nemu_link_studio_checksum_bytes(s->checksum, digest);
    return !memcmp(digest, checksum, 32);
}

uint32_t nemu_link_studio_checksum(uint32_t checksum, size_t offset, const uint8_t *bytes,
                                   size_t size) {
    /* Equivalent to XORing little-endian uint32_t words, with a zero-padded tail.
     * Byte access also works for unaligned buffers and arbitrary stream chunks. */
    while (size--) {
        checksum ^= (uint32_t)*bytes++ << (8u * (offset++ & 3u));
    }
    return checksum;
}
void nemu_link_studio_checksum_bytes(uint32_t checksum, uint8_t bytes[32]) {
    memset(bytes, 0, 32);
    for (unsigned int i = 0; i < 4; i++) {
        bytes[i] = (uint8_t)(checksum >> (8 * i));
    }
}
