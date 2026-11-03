#include <zmk/nemu_link/studio_transport.h>
#include <string.h>

static void put(uint8_t *p, uint64_t v, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}
static uint64_t get(const uint8_t *p, unsigned int n) {
    uint64_t v = 0;
    for (unsigned int i = 0; i < n; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}
bool nemu_link_studio_version_encode(uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE],
                                     const struct nemu_link_studio_version *v) {
    if (!bytes || !v || !v->size || v->size > UINT16_MAX) {
        return false;
    }
    memcpy(bytes, v->schema, 32);
    put(bytes + 32, v->clock[0], 8);
    put(bytes + 40, v->clock[1], 8);
    memcpy(bytes + 48, v->checksum, 32);
    put(bytes + 80, v->size, 4);
    return true;
}
bool nemu_link_studio_version_decode(struct nemu_link_studio_version *v,
                                     const uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE]) {
    if (!v || !bytes) {
        return false;
    }
    uint32_t size = (uint32_t)get(bytes + 80, 4);
    if (!size || size > UINT16_MAX) {
        return false;
    }
    *v = (struct nemu_link_studio_version){.size = size,
                                           .clock = {get(bytes + 32, 8), get(bytes + 40, 8)}};
    memcpy(v->schema, bytes, 32);
    memcpy(v->checksum, bytes + 48, 32);
    return true;
}
static bool recent(uint64_t now, uint64_t then, uint32_t limit) {
    return now >= then && now - then < limit;
}
bool nemu_link_studio_route_central(struct nemu_link_studio_route *route,
                                    const struct nemu_link_link_central *link,
                                    const struct nemu_link_hop_central *hop, uint64_t now) {
    return nemu_link_studio_route_central_source(route, link, hop,
                                                 CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE, now);
}
bool nemu_link_studio_route_central_source(struct nemu_link_studio_route *route,
                                           const struct nemu_link_link_central *link,
                                           const struct nemu_link_hop_central *hop, uint8_t source,
                                           uint64_t now) {
    if (!route || !link || !hop || source >= NEMU_LINK_LINK_SOURCES ||
        link->peers[source].phase != NEMU_LINK_LINK_UP || hop->searching || hop->pending ||
        hop->boot != link->epoch ||
        !recent(now, link->peers[source].last_seen_ms, link->release_ms)) {
        return false;
    }
    *route = (struct nemu_link_studio_route){.local_side = NEMU_LINK_STUDIO_DONGLE,
                                             .source = source,
                                             .boot = link->epoch,
                                             .session = link->peers[source].session,
                                             .nonce = link->peers[source].peer_nonce,
                                             .channel_epoch = hop->epoch,
                                             .channel_index = hop->index};
    return true;
}
bool nemu_link_studio_route_peripheral(struct nemu_link_studio_route *route,
                                       const struct nemu_link_link_peripheral *link,
                                       const struct nemu_link_hop_peripheral *hop, uint64_t now) {
    return nemu_link_studio_route_peripheral_source(route, link, hop,
                                                    CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE, now);
}
bool nemu_link_studio_route_peripheral_source(struct nemu_link_studio_route *route,
                                              const struct nemu_link_link_peripheral *link,
                                              const struct nemu_link_hop_peripheral *hop,
                                              uint8_t source, uint64_t now) {
    if (!route || !link || !hop || link->source != source ||
        link->phase != NEMU_LINK_LINK_PERIPHERAL_UP || hop->searching || hop->pending ||
        hop->boot != link->epoch || hop->index != hop->known_index ||
        !recent(now, link->confirmed_ms, link->release_ms)) {
        return false;
    }
    *route = (struct nemu_link_studio_route){.local_side = NEMU_LINK_STUDIO_KEYBOARD,
                                             .source = link->source,
                                             .boot = link->epoch,
                                             .session = link->session,
                                             .nonce = link->peer_nonce,
                                             .channel_epoch = hop->epoch,
                                             .channel_index = hop->index};
    return true;
}
bool nemu_link_studio_packet_valid(const struct nemu_link_studio_packet *p) {
    if (!p || p->length > NEMU_LINK_STUDIO_CHUNK_SIZE) {
        return false;
    }
    if (p->kind == NEMU_LINK_STUDIO_POLL) {
        return !p->transaction && !p->offset && !p->length;
    }
    if (p->kind == NEMU_LINK_STUDIO_SCHEMA_HASH) {
        return !p->transaction && (p->offset == 0 || p->offset == 24) &&
               p->length == (p->offset ? 8 : 24);
    }
    if (p->kind == NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH) {
        return p->transaction && (p->offset == 0 || p->offset == 24) &&
               p->length == (p->offset ? 8 : 24);
    }
    if (p->kind == NEMU_LINK_STUDIO_OWNER_RPC_REQUEST) {
        return p->transaction && (p->offset == 0 || p->offset == 24) &&
               p->length == (p->offset ? 12 : 24);
    }
    if (p->kind == NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE) {
        return p->transaction && (p->offset == 0 || p->offset == 24 || p->offset == 48) &&
               p->length == (p->offset == 48 ? 10 : 24);
    }
    if (!p->transaction) {
        return false;
    }
    switch (p->kind) {
    case NEMU_LINK_STUDIO_META:
    case NEMU_LINK_STUDIO_DELTA_META: {
        unsigned int total = p->kind == NEMU_LINK_STUDIO_META ? NEMU_LINK_STUDIO_VERSION_SIZE
                                                              : NEMU_LINK_STUDIO_DELTA_META_SIZE;
        return p->offset < total && p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0 &&
               p->length == (total - p->offset < NEMU_LINK_STUDIO_CHUNK_SIZE
                                 ? total - p->offset
                                 : NEMU_LINK_STUDIO_CHUNK_SIZE);
    }
    case NEMU_LINK_STUDIO_GROUP_META:
        return p->offset < NEMU_LINK_STUDIO_VERSION_SIZE &&
               p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0 &&
               p->length == (NEMU_LINK_STUDIO_VERSION_SIZE - p->offset < NEMU_LINK_STUDIO_CHUNK_SIZE
                                 ? NEMU_LINK_STUDIO_VERSION_SIZE - p->offset
                                 : NEMU_LINK_STUDIO_CHUNK_SIZE);
    case NEMU_LINK_STUDIO_DATA:
    case NEMU_LINK_STUDIO_GROUP_DATA:
        return p->length && p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0 &&
               (uint32_t)p->offset + p->length <= UINT16_MAX;
    case NEMU_LINK_STUDIO_COMMIT:
    case NEMU_LINK_STUDIO_GROUP_COMMIT:
        return !p->offset && !p->length;
    case NEMU_LINK_STUDIO_META_ACK:
        return !p->length && ((p->offset && p->offset <= NEMU_LINK_STUDIO_DELTA_META_SIZE &&
                               p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0) ||
                              p->offset == NEMU_LINK_STUDIO_VERSION_SIZE ||
                              p->offset == NEMU_LINK_STUDIO_DELTA_META_SIZE);
    case NEMU_LINK_STUDIO_GROUP_META_ACK:
        return !p->length && p->offset && p->offset <= NEMU_LINK_STUDIO_VERSION_SIZE &&
               (p->offset == NEMU_LINK_STUDIO_VERSION_SIZE ||
                p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0);
    case NEMU_LINK_STUDIO_DATA_ACK:
    case NEMU_LINK_STUDIO_GROUP_DATA_ACK:
    case NEMU_LINK_STUDIO_COMMIT_ACK:
    case NEMU_LINK_STUDIO_GROUP_COMMIT_ACK:
        return !p->length && p->offset;
    case NEMU_LINK_STUDIO_REMOTE_REQUEST:
        return !p->offset && p->length == NEMU_LINK_STUDIO_CHUNK_SIZE;
    case NEMU_LINK_STUDIO_REMOTE_RESPONSE:
        return !p->offset && (p->length == 16 || p->length == 24);
    case NEMU_LINK_STUDIO_REMOTE_BASE:
        return !p->offset && p->length == NEMU_LINK_STUDIO_CHUNK_SIZE;
    case NEMU_LINK_STUDIO_REMOTE_BASE_ACK:
        return !p->offset && p->length == 2;
    case NEMU_LINK_STUDIO_ERROR:
        return p->length == 2 && p->data[0] >= NEMU_LINK_STUDIO_INVALID &&
               p->data[0] <= NEMU_LINK_STUDIO_STORAGE_FAILED &&
               (((p->data[1] == NEMU_LINK_STUDIO_META ||
                  p->data[1] == NEMU_LINK_STUDIO_DELTA_META) &&
                 p->offset < (p->data[1] == NEMU_LINK_STUDIO_META
                                  ? NEMU_LINK_STUDIO_VERSION_SIZE
                                  : NEMU_LINK_STUDIO_DELTA_META_SIZE) &&
                 p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0) ||
                (p->data[1] == NEMU_LINK_STUDIO_DATA &&
                 p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0) ||
                (p->data[1] == NEMU_LINK_STUDIO_GROUP_META &&
                 p->offset < NEMU_LINK_STUDIO_VERSION_SIZE &&
                 p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0) ||
                (p->data[1] == NEMU_LINK_STUDIO_GROUP_DATA &&
                 p->offset % NEMU_LINK_STUDIO_CHUNK_SIZE == 0) ||
                (p->data[1] == NEMU_LINK_STUDIO_GROUP_COMMIT && p->offset == 0) ||
                (p->data[1] == NEMU_LINK_STUDIO_COMMIT && p->offset == 0));
    default:
        return false;
    }
}
static bool route_valid(const struct nemu_link_studio_route *r) {
    return r &&
           (r->local_side == NEMU_LINK_STUDIO_KEYBOARD ||
            r->local_side == NEMU_LINK_STUDIO_DONGLE) &&
           r->source < NEMU_LINK_LINK_SOURCES && r->boot && r->session && r->nonce &&
           r->channel_epoch && r->channel_index < nemu_link_hop_channel_count;
}
bool nemu_link_studio_route_same_session(const struct nemu_link_studio_route *a,
                                         const struct nemu_link_studio_route *b) {
    return route_valid(a) && route_valid(b) && a->local_side == b->local_side &&
           a->boot == b->boot && a->session == b->session && a->nonce == b->nonce &&
           a->source == b->source && a->dongle_id == b->dongle_id;
}
bool nemu_link_studio_owner_request_assemble(
    struct nemu_link_studio_owner_request_assembly *assembly,
    const struct nemu_link_studio_packet *packet, const struct nemu_link_studio_route *route,
    uint8_t bytes[36]) {
    if (!assembly || !packet || !route || !bytes ||
        packet->kind != NEMU_LINK_STUDIO_OWNER_RPC_REQUEST ||
        !nemu_link_studio_packet_valid(packet) ||
        !nemu_link_studio_route_same_session(route, route)) {
        return false;
    }
    if (packet->transaction != assembly->transaction ||
        !nemu_link_studio_route_same_session(route, &assembly->route)) {
        assembly->transaction = packet->transaction;
        assembly->received_mask = 0;
        assembly->route = *route;
    }
    memcpy(assembly->bytes + packet->offset, packet->data, packet->length);
    assembly->received_mask |= packet->offset ? 0x02 : 0x01;
    if (assembly->received_mask != 0x03) {
        return false;
    }
    memcpy(bytes, assembly->bytes, sizeof(assembly->bytes));
    assembly->received_mask = 0;
    return true;
}
size_t nemu_link_studio_frame_encode(uint8_t *b, size_t size,
                                     const struct nemu_link_studio_route *r,
                                     const struct nemu_link_studio_packet *p,
                                     nemu_link_link_crc_fn crc) {
    if (!b || size != NEMU_LINK_STUDIO_FRAME_SIZE || !route_valid(r) ||
        !nemu_link_studio_packet_valid(p)) {
        return false;
    }
    memset(b, 0, size);
    memcpy(b, "ZmKe", 4);
    b[4] = 27 + p->length;
    b[5] = r->source;
    b[6] = NEMU_LINK_STUDIO_FRAME_TAG;
    b[7] = NEMU_LINK_STUDIO_WIRE_VERSION;
    b[8] = (uint8_t)p->kind;
    b[9] = p->length;
    put(b + 10, r->boot, 4);
    put(b + 14, r->session, 4);
    put(b + 18, r->nonce, 4);
    put(b + 22, r->channel_epoch, 4);
    b[26] = r->channel_index;
    put(b + 27, p->transaction, 2);
    put(b + 29, p->offset, 2);
    b[31] = (uint8_t)r->local_side;
    memcpy(b + 32, p->data, p->length);
    size = 36 + p->length;
    put(b + size - 4, crc ? crc(b, size - 4) : 0, 4);
    return size;
}
bool nemu_link_studio_frame_decode(struct nemu_link_studio_packet *p,
                                   const struct nemu_link_studio_route *r, uint8_t pipe,
                                   const uint8_t *b, size_t size, nemu_link_link_crc_fn crc) {
    if (!p || !b || size < 36 || size > 60 || !route_valid(r) || pipe != r->source ||
        memcmp(b, "ZmKe", 4) || b[4] != size - 9 || b[5] != r->source ||
        b[6] != NEMU_LINK_STUDIO_FRAME_TAG || b[7] != NEMU_LINK_STUDIO_WIRE_VERSION ||
        size != 36u + b[9] || b[9] > NEMU_LINK_STUDIO_CHUNK_SIZE ||
        b[31] != (uint8_t)(1 - r->local_side) || get(b + 10, 4) != r->boot ||
        get(b + 14, 4) != r->session || get(b + 18, 4) != r->nonce ||
        get(b + 22, 4) != r->channel_epoch || b[26] != r->channel_index ||
        (crc && get(b + size - 4, 4) != crc(b, size - 4))) {
        return false;
    }
    struct nemu_link_studio_packet decoded = {.kind = (enum nemu_link_studio_packet_kind)b[8],
                                              .length = b[9],
                                              .transaction = (uint16_t)get(b + 27, 2),
                                              .offset = (uint16_t)get(b + 29, 2)};
    memcpy(decoded.data, b + 32, decoded.length);
    if (!nemu_link_studio_packet_valid(&decoded)) {
        return false;
    }
    *p = decoded;
    return true;
}
