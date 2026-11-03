#pragma once
#include <zmk/nemu_link/wire.h>
#include <zmk/nemu_link/secure.h>
#include <zmk/nemu_link/hop_frame.h>

bool nemu_link_mini_wire(size_t size);
bool nemu_link_mini_forward(uint32_t previous, uint16_t low, uint32_t *full);
bool nemu_link_mini_down_forward(uint32_t previous, uint16_t wire, uint32_t *full);
bool nemu_link_mini_ack_high(uint32_t reference, uint16_t wire, uint32_t *high);
bool nemu_link_mini_ack_counter(uint32_t reference, uint16_t low, uint32_t *full);
void nemu_link_mini_note(struct nemu_link_secure_peer *peer, bool tx,
                         const struct nemu_link_link_message *message, uint32_t mode,
                         const struct nemu_link_hop_notice *notice);
bool nemu_link_mini_can_send(const struct nemu_link_secure_peer *peer,
                             const struct nemu_link_link_message *message, uint32_t mode,
                             const struct nemu_link_hop_notice *notice);
int nemu_link_mini_wrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                        const struct nemu_link_link_message *message,
                        uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *size);
int nemu_link_mini_unwrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                          const uint8_t *wire, size_t size, uint32_t epoch, uint32_t session,
                          uint32_t peer_nonce, uint32_t ack_reference,
                          struct nemu_link_link_message *message, uint32_t *mode,
                          struct nemu_link_hop_notice *notice);
