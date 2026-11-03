#pragma once
#include <zmk/nemu_link/wire.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NEMU_LINK_NRF_CCM_MIC_SIZE NEMU_LINK_CCM_MIC_SIZE
#define NEMU_LINK_NRF_CCM_PLAIN_MAX NEMU_LINK_CCM_PLAIN_MAX

/* AES key in standard byte order, BLE nonce = counter/direction (5) + IV (8).
 * AAD is the fixed zero packet header. Output is written only after success.
 * Runs in thread context with BLE disabled; no RADIO/PPI connections are used. */
int nemu_link_nrf_ccm_crypt(bool decrypt, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                            const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *input,
                            size_t length, uint8_t *output, size_t capacity);
int nemu_link_nrf_ccm_self_test(void);

#if defined(CONFIG_ZMK_NEMU_LINK_CCM_RADIO) && CONFIG_ZMK_NEMU_LINK_CCM_RADIO
/* IRQ-safe, exclusive reservation. Caller supplies an aligned CCM-format
 * DMA input which remains live until finish/cancel. No thread wake or PPI. */
int nemu_link_nrf_ccm_radio_begin(bool decrypt, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                                  const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], uint8_t *input,
                                  bool crypt_now);
void nemu_link_nrf_ccm_radio_input(uint8_t *input);
int nemu_link_nrf_ccm_radio_crypt(void);
int nemu_link_nrf_ccm_radio_finish(uint8_t *out, size_t capacity);
void nemu_link_nrf_ccm_radio_cancel(void);
bool nemu_link_nrf_ccm_radio_available(void);
#endif
