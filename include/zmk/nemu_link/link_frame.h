#pragma once
#include <zmk/nemu_link/link.h>

#define NEMU_LINK_LINK_FRAME_CORE_OFFSET 7
#define NEMU_LINK_LINK_FRAME_MODE_OFFSET                                                           \
    (NEMU_LINK_LINK_FRAME_CORE_OFFSET + NEMU_LINK_LINK_WIRE_SIZE)
#define NEMU_LINK_LINK_FRAME_CRC_OFFSET (NEMU_LINK_LINK_FRAME_MODE_OFFSET + 4)
#define NEMU_LINK_LINK_FRAME_SIZE (NEMU_LINK_LINK_FRAME_CRC_OFFSET + 4)
#define NEMU_LINK_LINK_FRAME_PAYLOAD_SIZE (NEMU_LINK_LINK_FRAME_SIZE - 9)
#define NEMU_LINK_LINK_FRAME_EXTENSION_SIZE 9
/* Pass NULL only for an envelope protected by AEAD; legacy frames use crc32_ieee. */
typedef uint32_t (*nemu_link_link_crc_fn)(const uint8_t *bytes, size_t size);
/* Encode returns transmitted byte count, or zero on error. Decode takes actual RX length. */
size_t nemu_link_link_frame_encode(uint8_t *bytes, size_t size,
                                   const struct nemu_link_link_message *message, uint32_t mode,
                                   nemu_link_link_crc_fn crc);
bool nemu_link_link_frame_decode(struct nemu_link_link_message *message, uint32_t *mode,
                                 uint8_t pipe, const uint8_t *bytes, size_t size,
                                 nemu_link_link_crc_fn crc);
/* Shared native envelope codec. A hop extension is appended directly to the
 * core instead of encoding and decoding a nested base envelope. */
size_t nemu_link_link_frame_encode_extension(
    uint8_t *bytes, size_t capacity, const struct nemu_link_link_message *message, uint32_t mode,
    const uint8_t extension[NEMU_LINK_LINK_FRAME_EXTENSION_SIZE], nemu_link_link_crc_fn crc);
bool nemu_link_link_frame_decode_extension(struct nemu_link_link_message *message, uint32_t *mode,
                                           uint8_t pipe, const uint8_t *bytes, size_t size,
                                           bool extended, nemu_link_link_crc_fn crc);
