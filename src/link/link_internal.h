#pragma once
#include <zephyr/kernel.h>
#include <zmk/nemu_link/link.h>
#include <zmk/nemu_link/split_frame.h>
#include <zmk/nemu_link/secure.h>
#include "radio.h"

struct rx_frame {
    uint64_t received_ms;
    uint8_t pipe, size;
    int8_t rssi_dbm;
    uint8_t bytes[64];
};

extern struct k_mutex link_lock;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
extern const struct zmk_split_transport_central *active_transport;
#elif IS_ENABLED(CONFIG_ZMK_SPLIT)
const struct zmk_split_transport_peripheral *zmk_nemu_link_peripheral_transport(void);
#else
bool nemu_link_peripheral_transport_active(void);
#endif
extern bool enabled;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
extern struct nemu_link_link_central central;
#else
extern struct nemu_link_link_peripheral peripheral;
#endif

int send_frame(uint8_t source, uint8_t *bytes, size_t size);
struct nemu_link_secure_peer *secure_peer(uint8_t source);
int idle_wake(void);
void search_input(uint64_t now);
int nemu_link_split_native_send(app_esb_data_t *packet);
int nemu_link_split_receive(const struct rx_frame *rx);
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
int nemu_link_split_receive_standalone(const struct rx_frame *rx);
#endif
bool nemu_link_studio_get_peripheral_rssi(uint8_t source, int8_t *rssi_dbm);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
void nemu_link_split_diag_log(void);
#endif
void nemu_link_split_reset(void);
void nemu_link_split_discard_input(void);
void nemu_link_split_source_lost(uint8_t source);

bool nemu_link_link_input_allowed(uint8_t source);
