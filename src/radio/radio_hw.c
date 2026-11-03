#include <errno.h>
#ifdef NEMU_LINK_CLOCK_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
#else
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>
#include <zmk/nemu_link/key_store.h>
#include "radio_hw.h"
#endif

static struct onoff_client clock_request;
static struct onoff_manager *clock_manager;
K_SEM_DEFINE(clock_ready_sem, 0, 1);
bool hf_clock_held;

static void clock_ready(struct onoff_manager *manager, struct onoff_client *client, uint32_t state,
                        int result) {
    (void)manager;
    (void)client;
    (void)state;
    (void)result;
    k_sem_give(&clock_ready_sem);
}

int nemu_link_clock_acquire(uint32_t timeout_ms) {
    if (hf_clock_held) {
        return 0;
    }
    struct onoff_manager *manager = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (manager == NULL) {
        return -ENXIO;
    }
    /* Requests are serialized by the link mutex. Keep the client and semaphore
     * alive across timeout cancellation, including completion at the deadline. */
    k_sem_reset(&clock_ready_sem);
    sys_notify_init_callback(&clock_request.notify, clock_ready);
    int status = onoff_request(manager, &clock_request);
    if (status < 0) {
        return status;
    }

    status = k_sem_take(&clock_ready_sem, K_MSEC(timeout_ms));
    if (status < 0) {
        onoff_cancel_or_release(manager, &clock_request);
        return status == -EAGAIN ? -ETIMEDOUT : status;
    }
    int result = 0;
    status = sys_notify_fetch_result(&clock_request.notify, &result);
    if (status < 0 || result < 0) {
        onoff_cancel_or_release(manager, &clock_request);
        return status < 0 ? status : result;
    }
    clock_manager = manager;
    hf_clock_held = true;
    return 0;
}

int nemu_link_clock_release(void) {
    if (!hf_clock_held) {
        return 0;
    }
    int status = onoff_release(clock_manager);
    if (status < 0) {
        return status;
    }
    hf_clock_held = false;
    clock_manager = NULL;
    return 0;
}

#ifndef NEMU_LINK_CLOCK_HOST_TEST
int nemu_link_hw_start(app_esb_mode_t mode, bool hop_enabled, uint8_t channel,
                       const uint8_t *prefixes, size_t prefix_count, esb_event_handler handler) {
    uint8_t base_address[NEMU_LINK_RADIO_ADDRESS_SIZE];
    uint8_t base_address_1[NEMU_LINK_RADIO_ADDRESS_SIZE];
    int address_status = nemu_link_key_store_read_radio_address(base_address);
    if (address_status) {
        return address_status;
    }
    memcpy(base_address_1, base_address, sizeof(base_address_1));
    struct esb_config config = ESB_DEFAULT_CONFIG;
    config.mode = mode == APP_ESB_MODE_PTX ? ESB_MODE_PTX : ESB_MODE_PRX;
    config.protocol = ESB_PROTOCOL_ESB_DPL;
#if defined(RADIO_MODE_MODE_Ble_1Mbit) && defined(RADIO_MODE_MODE_Ble_2Mbit)
    config.bitrate = IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_BLE_1M)   ? ESB_BITRATE_1MBPS_BLE
                     : IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_BLE_2M) ? ESB_BITRATE_2MBPS_BLE
                                                                     : ESB_BITRATE_2MBPS;
#else
    BUILD_ASSERT(!IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_BLE_1M) &&
                     !IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_BLE_2M),
                 "BLE RADIO modes are unavailable on this SoC");
    config.bitrate = ESB_BITRATE_2MBPS;
#endif
    config.data_whitening = IS_ENABLED(CONFIG_ZMK_NEMU_LINK_DATA_WHITENING);
    config.whitening_iv = CONFIG_ZMK_NEMU_LINK_DATA_WHITENING_IV;
    config.retransmit_delay = CONFIG_ZMK_NEMU_LINK_PROTO_TX_RETRANSMIT_DELAY;
    config.retransmit_count = CONFIG_ZMK_NEMU_LINK_PROTO_TX_RETRANSMIT_COUNT;
    config.tx_mode = ESB_TXMODE_MANUAL_START;
    config.selective_auto_ack = true;
    config.use_fast_ramp_up = true;
    config.event_handler = handler;
    config.tx_output_power = ESB_TX_POWER_0DBM;
#if defined(RADIO_TXPOWER_TXPOWER_Neg8dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_MINUS_8)) {
        config.tx_output_power = ESB_TX_POWER_NEG8DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg12dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_MINUS_12)) {
        config.tx_output_power = ESB_TX_POWER_NEG12DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_2)) {
        config.tx_output_power = ESB_TX_POWER_2DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_3)) {
        config.tx_output_power = ESB_TX_POWER_3DBM;
    }
#endif
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_4)) {
        config.tx_output_power = ESB_TX_POWER_4DBM;
    }
#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_5)) {
        config.tx_output_power = ESB_TX_POWER_5DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_6)) {
        config.tx_output_power = ESB_TX_POWER_6DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_7)) {
        config.tx_output_power = ESB_TX_POWER_7DBM;
    }
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TX_POWER_PLUS_8)) {
        config.tx_output_power = ESB_TX_POWER_8DBM;
    }
#endif

    int status = esb_init(&config);
    if (status == 0) {
        status = esb_set_base_address_0(base_address);
    }
    if (status == 0) {
        status = esb_set_base_address_1(base_address_1);
    }
    if (status == 0) {
        status = esb_set_prefixes(prefixes, prefix_count);
    }
    if (status == 0 && hop_enabled) {
        status = esb_set_rf_channel(channel);
    }
    if (status != 0) {
        return status;
    }
    NVIC_SetPriority(RADIO_IRQn, 0);
    return mode == APP_ESB_MODE_PRX ? esb_start_rx() : 0;
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
void nemu_link_hw_scan(uint8_t channel, uint16_t *busy, uint16_t *samples) {
    /* esb_stop_rx has removed shortcuts, interrupts and ESB PPI. Keep RX
     * addresses disabled so candidate traffic cannot reach the ESB DMA/FIFO. */
    NRF_RADIO->FREQUENCY = channel;
    NRF_RADIO->RXADDRESSES = 0;
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (!NRF_RADIO->EVENTS_READY) {
    }
    *busy = *samples = 0;
    uint32_t start = k_cycle_get_32();
    uint32_t window = k_us_to_cyc_ceil32(CONFIG_ZMK_NEMU_LINK_SCAN_WINDOW_US);
    uint32_t sample_time = k_us_to_cyc_ceil32(20);
    while ((uint32_t)(k_cycle_get_32() - start) + sample_time <= window) {
        NRF_RADIO->EVENTS_RSSIEND = 0;
        NRF_RADIO->TASKS_RSSISTART = 1;
        k_busy_wait(20);
        if (NRF_RADIO->EVENTS_RSSIEND) {
            (*samples)++;
            if (-(int)NRF_RADIO->RSSISAMPLE > CONFIG_ZMK_NEMU_LINK_BUSY_RSSI_DBM) {
                (*busy)++;
            }
        }
    }
    NRF_RADIO->TASKS_RSSISTOP = 1;
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (!NRF_RADIO->EVENTS_DISABLED) {
    }
}

void nemu_link_hw_scan_rssi(uint8_t channel, int8_t *minimum, int8_t *maximum, int8_t *mean,
                            uint16_t *samples) {
    /* Keep ESB shortcuts/addresses disabled while observing an arbitrary RF channel. */
    NRF_RADIO->FREQUENCY = channel;
    NRF_RADIO->RXADDRESSES = 0;
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (!NRF_RADIO->EVENTS_READY) {
    }

    int16_t min_rssi = INT8_MAX;
    int16_t max_rssi = INT8_MIN;
    int32_t sum = 0;
    *samples = 0;
    uint32_t start = k_cycle_get_32();
    uint32_t window = k_us_to_cyc_ceil32(CONFIG_ZMK_NEMU_LINK_SCAN_WINDOW_US);
    uint32_t sample_time = k_us_to_cyc_ceil32(20);
    while ((uint32_t)(k_cycle_get_32() - start) + sample_time <= window) {
        NRF_RADIO->EVENTS_RSSIEND = 0;
        NRF_RADIO->TASKS_RSSISTART = 1;
        k_busy_wait(20);
        if (NRF_RADIO->EVENTS_RSSIEND) {
            int16_t rssi = -(int16_t)NRF_RADIO->RSSISAMPLE;
            if (rssi < min_rssi) {
                min_rssi = rssi;
            }
            if (rssi > max_rssi) {
                max_rssi = rssi;
            }
            sum += rssi;
            (*samples)++;
        }
    }

    NRF_RADIO->TASKS_RSSISTOP = 1;
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (!NRF_RADIO->EVENTS_DISABLED) {
    }

    if (*samples) {
        *minimum = (int8_t)min_rssi;
        *maximum = (int8_t)max_rssi;
        *mean = (int8_t)(sum / *samples);
    }
}
#endif

#endif
