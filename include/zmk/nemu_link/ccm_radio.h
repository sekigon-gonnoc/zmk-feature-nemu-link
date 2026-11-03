#pragma once
#include <zmk/nemu_link/wire.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
struct nemu_link_secure_peer;

int nemu_link_ccm_radio_queue(uint8_t source, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                              const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *plain,
                              size_t size, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t header);
void nemu_link_ccm_radio_context(const struct nemu_link_secure_peer *peer, uint8_t source,
                                 uint32_t ack_reference);
void nemu_link_ccm_radio_retire(uint8_t source, const uint8_t *wire, size_t size);
void nemu_link_ccm_radio_stop(void);
void nemu_link_ccm_radio_reset(void);
int nemu_link_ccm_radio_tx_prepare(uint8_t *wire, uint8_t *fifo_wire, size_t size, uint8_t source);
int nemu_link_ccm_radio_tx_ready(void);
bool nemu_link_ccm_radio_can_tx(const uint8_t *wire, size_t size, uint8_t source);
void nemu_link_ccm_radio_rx_arm(uint8_t *pdu, uint8_t source);
void nemu_link_ccm_radio_rx_prefix(void);
void nemu_link_ccm_radio_rx_finish(bool crc_ok);
/* Returns -ENOENT when no completed hardware result matches the exact nonce,
 * context key and ciphertext. Success/error is consumed once. */
int nemu_link_ccm_radio_rx_take(uint8_t source, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                                const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *wire,
                                size_t size, uint8_t *plain, size_t capacity);
