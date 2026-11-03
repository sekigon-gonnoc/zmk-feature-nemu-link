#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef enum { APP_ESB_EVT_TX_SUCCESS, APP_ESB_EVT_TX_FAIL, APP_ESB_EVT_RX } app_esb_event_type_t;
typedef enum { APP_ESB_MODE_PTX, APP_ESB_MODE_PRX } app_esb_mode_t;
typedef struct {
    app_esb_event_type_t evt_type;
    uint8_t pipe;
    int8_t rssi_dbm; /* RX events only; negative dBm. */
    uint8_t *buf;
    uint32_t data_length;
    uint16_t msg_id;
} app_esb_event_t;
typedef struct {
    uint8_t pipe;
    uint8_t *data;
    uint32_t len;
    uint16_t msg_id;
    uint8_t max_retry;
    bool replaceable;
} app_esb_data_t;
typedef void (*app_esb_callback_t)(app_esb_event_t *event);
struct esb_msg_prefix {
    uint8_t magic_prefix[4];
    uint8_t payload_size;
} __packed;
int nemu_link_radio_init(app_esb_mode_t mode, app_esb_callback_t callback);
int nemu_link_radio_enable(bool enabled);
uint8_t nemu_link_radio_debug_state(void);
int nemu_link_radio_send(app_esb_data_t *packet);
int nemu_link_radio_discard_pipe(uint8_t pipe);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
int nemu_link_radio_scan_frequency_locked(uint8_t channel, int8_t *minimum, int8_t *maximum,
                                          int8_t *mean, uint16_t *samples);
#endif
