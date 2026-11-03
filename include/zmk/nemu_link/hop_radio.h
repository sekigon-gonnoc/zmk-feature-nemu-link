#pragma once
#include <stdint.h>
#define NEMU_LINK_FULL_SCAN_CHANNEL_COUNT 80U
/* Under link mutex + IRQ lock. Busy means an actual PTX transaction or its
 * deferred completion is pending. Queued snapshots/ACKs are invalidated only
 * after that boundary. Reconstruct current STATE after a successful retune. */
int nemu_link_radio_channel(uint8_t index);
/* Retry a transient write/start failure without another input event. */
void nemu_link_radio_service(void);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
void nemu_link_radio_diag_log(void);
#endif
uint8_t nemu_link_radio_index(void);

/* Select a registered RF address while the radio is stopped. */
int nemu_link_radio_profile(uint8_t id);

/* Drop old PRX ACK payloads and software queue before accepting a returning set. */
int nemu_link_radio_clear_pending(void);

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
/* Under the link mutex and IRQ lock. Preserve ACK FIFO, PID and applied index. */
int nemu_link_radio_scan(uint8_t index, uint16_t *busy, uint16_t *samples);
/* Sample an arbitrary 2.4 GHz channel while preserving the active PRX state. */
int nemu_link_radio_scan_frequency(uint8_t channel, int8_t *minimum, int8_t *maximum, int8_t *mean,
                                   uint16_t *samples);
#endif
