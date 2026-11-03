#pragma once
#include <zmk/nemu_link/hop.h>
#include <zmk/nemu_link/link_frame.h>

/* Maximum frame size. Encoders return actual size: STATE 64, control 44. */
#define NEMU_LINK_HOP_FRAME_EXTENSION_SIZE 9
#define NEMU_LINK_HOP_FRAME_SIZE (NEMU_LINK_LINK_FRAME_SIZE + NEMU_LINK_HOP_FRAME_EXTENSION_SIZE)
size_t nemu_link_hop_frame_encode(uint8_t *bytes, size_t size,
                                  const struct nemu_link_link_message *message, uint32_t mode,
                                  const struct nemu_link_hop_notice *notice,
                                  nemu_link_link_crc_fn crc);
bool nemu_link_hop_frame_decode(struct nemu_link_link_message *message, uint32_t *mode,
                                struct nemu_link_hop_notice *notice, uint8_t pipe,
                                const uint8_t *bytes, size_t size, nemu_link_link_crc_fn crc);
