#pragma once
#include <stdbool.h>
#include <stdint.h>
void nemu_link_notify_source_lost(uint8_t source);
void nemu_link_notify_mode_requested(bool esb);
void nemu_link_notify_mode_changed(bool esb, int error);
void nemu_link_request_ble_peers(void);
int nemu_link_request_next_dongle(void);
void nemu_link_mode_source_lost(uint8_t source);
void nemu_link_mode_requested(bool esb);
int nemu_link_mode_command(uint32_t mode);
int nemu_link_recover_ble(void);
bool nemu_link_peripheral_transport_active(void);
int nemu_link_peripheral_transport_set_enabled(bool enabled);
