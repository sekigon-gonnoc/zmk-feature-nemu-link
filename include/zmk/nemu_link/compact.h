#pragma once
#include <zmk/nemu_link/wire.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NEMU_LINK_COMPACT_WIRE_VERSION NEMU_LINK_WIRE_VERSION

/* Lossless wire-v1 bodies. Session identifiers are supplied only by the
 * authenticated current link, never by a compact packet. */
bool nemu_link_compact_pack(uint8_t source, bool link_allowed,
                            const uint8_t native[NEMU_LINK_FRAME_MAX_SIZE], size_t native_size,
                            uint8_t *kind, uint8_t body[NEMU_LINK_COMPACT_BODY_MAX],
                            size_t *body_size);
bool nemu_link_compact_kind(uint8_t kind);
bool nemu_link_compact_unpack(uint8_t kind, uint8_t source, uint8_t dongle, const uint8_t *body,
                              size_t body_size, uint32_t security_sequence, uint32_t epoch,
                              uint32_t session, uint32_t nonce,
                              uint8_t native[NEMU_LINK_FRAME_MAX_SIZE], size_t *native_size);
