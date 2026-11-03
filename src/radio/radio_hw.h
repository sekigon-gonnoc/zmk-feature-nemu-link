#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <esb.h>
#include "radio.h"

extern bool hf_clock_held;
int nemu_link_clock_acquire(uint32_t timeout_ms);
int nemu_link_clock_release(void);
int nemu_link_hw_start(app_esb_mode_t mode, bool hop_enabled, uint8_t channel,
                       const uint8_t *prefixes, size_t prefix_count, esb_event_handler handler);

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
bool nemu_link_esb_scan_ready(void);
void nemu_link_hw_scan(uint8_t channel, uint16_t *busy, uint16_t *samples);
void nemu_link_hw_scan_rssi(uint8_t channel, int8_t *minimum, int8_t *maximum, int8_t *mean,
                            uint16_t *samples);
#endif
