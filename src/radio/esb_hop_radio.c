#define AUDIO_QUEUE_RESERVE 4U
#if defined(NEMU_LINK_RADIO_HOST_TEST)
#include "app_esb.h"
#include <esb.h>
#include <esb_peripherals.h>
#include <zmk/nemu_link/hop.h>
static uint8_t radio_index;
#include "app_esb.c"
#else
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <esb.h>
#include <esb_peripherals.h>
#include "radio.h"
#include "radio_hw.h"
#include <zmk/nemu_link/hop.h>
#include <zmk/nemu_link/hop_radio.h>

LOG_MODULE_REGISTER(nemu_link_radio, LOG_LEVEL_INF);
#define RADIO_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(sekigon_nemu_link_radio)
BUILD_ASSERT(CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS > AUDIO_QUEUE_RESERVE,
             "Nemu-link audio queue reservation must leave room for protocol traffic");
static uint8_t esb_addr_prefix[] = DT_PROP(RADIO_NODE, addr_prefix);
static uint8_t radio_index;
static app_esb_mode_t m_mode;
static app_esb_callback_t m_callback;
static bool m_active, m_enabled;
#endif
#include <zmk/nemu_link/wire.h>
#include <zmk/nemu_link/hop.h>
#include <zmk/nemu_link/hop_radio.h>
#include <zmk/nemu_link/key_record.h>
#include <zmk/nemu_link/key_store.h>
#include <zmk/nemu_link/selection.h>
#include <zmk/nemu_link/compact.h>
#include <zmk/nemu_link/idle.h>
#include <zmk/nemu_link/power_profile.h>
#include <zmk/nemu_link/power_trace.h>
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
#include <zmk/nemu_link/ccm_radio.h>
#endif

#define NEMU_LINK_DONGLE_ADDR_PREFIX0_BASE UINT8_C(0x36)
#define NEMU_LINK_DONGLE_ADDR_PREFIX1_BASE UINT8_C(0xa9)

struct hop_tx {
    struct esb_payload payload;
    uint16_t id;
    uint8_t retries;
    bool replaceable;
};
K_MSGQ_DEFINE(hop_tx_queue, sizeof(struct hop_tx), CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS, 4);
static struct hop_tx inflight;
static bool tx_active;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE)
static uint32_t tx_profile_start;
#endif
/* esb_init/disable reset Nordic's 2-bit PID. Keep PTX identity progressing
 * across idle wake/retune so a fresh probe is not a hardware duplicate. */
static uint8_t ptx_pid[NEMU_LINK_HOP_SOURCES];
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
static uint32_t diag_tx_start, diag_tx_ok, diag_tx_fail, diag_rx, diag_write_error;
static uint32_t diag_rx_pipe[NEMU_LINK_HOP_SOURCES], diag_write_pipe[NEMU_LINK_HOP_SOURCES];
static uint32_t diag_ack_retired, diag_ack_reconciled;
#endif
#define ACK_FIFO_PER_PIPE_LIMIT (CONFIG_ESB_TX_FIFO_SIZE / NEMU_LINK_HOP_SOURCES)
BUILD_ASSERT(ACK_FIFO_PER_PIPE_LIMIT > 0,
             "ESB TX FIFO must reserve at least one ACK payload for each Nemu-link pipe");
/* PRX ACK payloads retire on the NEXT new packet from that pipe, not on their first use. */
static uint8_t ack_count[NEMU_LINK_HOP_SOURCES];
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
/* At most one Studio frame across software + hardware FIFO. */
static bool control_pending;
static bool ack_control[NEMU_LINK_HOP_SOURCES][CONFIG_ESB_TX_FIFO_SIZE];
static bool is_control(const struct esb_payload *p) {
    return p->length >= NEMU_LINK_COMPACT_MIN_SIZE &&
           p->data[0] == NEMU_LINK_COMPACT_WIRE_VERSION && p->data[1] == NEMU_LINK_WIRE_STUDIO;
}
#endif
static uint16_t ack_msg_id[NEMU_LINK_HOP_SOURCES][CONFIG_ESB_TX_FIFO_SIZE];
static void ack_retired(uint8_t pipe) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    bool control = ack_control[pipe][0];
    if (control) {
        control_pending = false;
    }
#endif
    uint16_t msg_id = ack_msg_id[pipe][0];
    ack_count[pipe]--;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    memmove(ack_control[pipe], ack_control[pipe] + 1, ack_count[pipe] * sizeof(bool));
#endif
    memmove(ack_msg_id[pipe], ack_msg_id[pipe] + 1, ack_count[pipe] * sizeof(uint16_t));
    app_esb_event_t event = {.evt_type = APP_ESB_EVT_TX_SUCCESS, .pipe = pipe, .msg_id = msg_id};
    m_callback(&event);
}
static void reconcile_ack_fifo(void) {
    for (uint8_t pipe = 0; pipe < NEMU_LINK_HOP_SOURCES; pipe++) {
        int actual = esb_ack_payload_count(pipe);
        if (actual < 0) {
            continue;
    }
        while (ack_count[pipe] > (unsigned int)actual) {
        ack_retired(pipe);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            diag_ack_reconciled++;
#endif
        }
    }
}
static bool is_audio(const struct esb_payload *p) {
    return p->length >= NEMU_LINK_COMPACT_MIN_SIZE && p->data[0] == NEMU_LINK_WIRE_VERSION &&
           p->data[1] == NEMU_LINK_WIRE_AUDIO;
}
/* ID 1 and 2 preserve the existing receiver addresses for firmware compatibility. */
int nemu_link_radio_profile(uint8_t id) {
    if (id < 1 || id > NEMU_LINK_MAX_DONGLES) {
        return -EINVAL;
    }
    if (m_enabled || m_active) {
        return -EBUSY;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    esb_addr_prefix[0] = (uint8_t)(NEMU_LINK_DONGLE_ADDR_PREFIX0_BASE + id - 1);
    esb_addr_prefix[1] = (uint8_t)(NEMU_LINK_DONGLE_ADDR_PREFIX1_BASE + id - 1);
    return 0;
#else
    return -ENOTSUP;
#endif
}
static bool repair_required;
static void hop_event(const struct esb_evt *event);
static void pump(void);

static void stop_radio(void) {
    m_active = false;
    esb_disable(); /* Stops RADIO, disables ESB IRQs and resets RX/TX FIFOs. */
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    nemu_link_ccm_radio_stop();
#endif
    NVIC_ClearPendingIRQ(ESB_RADIO_IRQ_NUMBER);
    NVIC_ClearPendingIRQ(ESB_EVT_IRQ_NUMBER);
    NVIC_ClearPendingIRQ(ESB_TIMER_IRQ);
}
static int start_radio(void) {
    int err = nemu_link_hw_start(m_mode, IS_ENABLED(CONFIG_ZMK_NEMU_LINK_HOP),
                                 nemu_link_hop_channels[radio_index], esb_addr_prefix,
                                 ARRAY_SIZE(esb_addr_prefix), hop_event);
    if (!err && m_mode == APP_ESB_MODE_PTX) {
        for (uint8_t pipe = 0; pipe < NEMU_LINK_HOP_SOURCES && !err; pipe++) {
            /* The public reuse API decrements the driver's PID modulo 4.
             * Starting from zero, these calls restore the previous PID. */
            for (uint8_t i = 0; i < (4 - ptx_pid[pipe]) % 4 && !err; i++) {
                err = esb_reuse_pid(pipe);
            }
        }
    }
    if (err) {
        stop_radio();
        return err;
    }
    m_active = true;
    return 0;
}
static int write_payload(const struct esb_payload *payload) {
    int err = esb_write_payload(payload);
    if (!err && m_mode == APP_ESB_MODE_PTX) {
        ptx_pid[payload->pipe] = (uint8_t)((ptx_pid[payload->pipe] + 1U) % 4U);
    }
    return err;
}
static void complete(bool success) {
    uint16_t completed_id = inflight.id;
    LOG_DBG("hop TX id=%u result=%u", inflight.id, success);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    if (is_control(&inflight.payload)) {
        control_pending = false;
    }
#endif
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    nemu_link_ccm_radio_retire(inflight.payload.pipe, inflight.payload.data,
                               inflight.payload.length);
#endif
    tx_active = false;
    inflight = (struct hop_tx){0};
    app_esb_event_t event = {
        .evt_type = success ? APP_ESB_EVT_TX_SUCCESS : APP_ESB_EVT_TX_FAIL,
        .msg_id = completed_id,
    };
    m_callback(&event);
}
static void pump_locked(void) {
    if (!m_active || tx_active) {
        return;
    }
    if (m_mode == APP_ESB_MODE_PRX) {
        reconcile_ack_fifo();
    }
    struct hop_tx next;
    /* Bounded by queue capacity. PRX uses ESB's pipe-specific ACK queues; PTX
     * has exactly one hardware packet, kept separate until deferred completion. */
    for (unsigned int i = 0; i < CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS; i++) {
        if (k_msgq_peek(&hop_tx_queue, &next)) {
            return;
        }
        if (m_mode == APP_ESB_MODE_PTX && !esb_is_idle()) {
            return;
        }
        if (m_mode == APP_ESB_MODE_PRX && ack_count[next.payload.pipe] >= ACK_FIFO_PER_PIPE_LIMIT) {
            k_msgq_get(&hop_tx_queue, &next, K_NO_WAIT);
            k_msgq_put(&hop_tx_queue, &next, K_NO_WAIT);
            continue;
        }
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
        if (m_mode == APP_ESB_MODE_PTX &&
            !nemu_link_ccm_radio_can_tx(next.payload.data, next.payload.length,
                                        next.payload.pipe)) {
            return;
        }
#endif
        /* Hardware is authoritative. A duplicate packet can first use an ACK
         * without an RX event, so the software ledger can overcount. */
        int err = write_payload(&next.payload);
        if (err) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            diag_write_error++;
#endif
            return; /* Retain entry; service retries boundedly. */
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        diag_write_pipe[next.payload.pipe]++;
#endif
        if (m_mode == APP_ESB_MODE_PTX) {
            inflight = next;
            tx_active = true;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE)
            tx_profile_start = nemu_link_profile_start();
#endif
            nemu_link_trace_mark(NEMU_LINK_TRACE_TX_START, next.payload.length);
            err = esb_start_tx();
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            if (!err) {
                diag_tx_start++;
            } else {
                diag_write_error++;
            }
#endif
            if (err) {
                tx_active = false;
                inflight = (struct hop_tx){0};
                esb_flush_tx();
                return;
            }
        }
        if (m_mode == APP_ESB_MODE_PRX) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
            ack_control[next.payload.pipe][ack_count[next.payload.pipe]] =
                is_control(&next.payload);
#endif
            ack_count[next.payload.pipe]++;
            ack_msg_id[next.payload.pipe][ack_count[next.payload.pipe] - 1] = next.id;
        }
        k_msgq_get(&hop_tx_queue, &next, K_NO_WAIT);
        if (m_mode == APP_ESB_MODE_PTX) {
            return;
        }
    }
}
static void pump(void) {
    unsigned int key = irq_lock();
    pump_locked();
    irq_unlock(key);
}
static void hop_event(const struct esb_evt *event) {
    if (!m_active) {
        return;
    }
    if (event->evt_id == ESB_EVENT_TX_SUCCESS || event->evt_id == ESB_EVENT_TX_FAILED) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        if (m_mode == APP_ESB_MODE_PRX && event->evt_id == ESB_EVENT_TX_SUCCESS) {
            diag_ack_retired++;
        }
#endif
        if (m_mode == APP_ESB_MODE_PTX && tx_active) {
            bool success = event->evt_id == ESB_EVENT_TX_SUCCESS;
            nemu_link_trace_mark(NEMU_LINK_TRACE_TX_DONE,
                                 (uint16_t)((success ? 0x8000U : 0U) | event->tx_attempts));
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE)
            nemu_link_profile_end(NEMU_LINK_PROFILE_TX, tx_profile_start);
            nemu_link_profile_attempts(event->tx_attempts);
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            if (success) {
                diag_tx_ok++;
            } else {
                diag_tx_fail++;
            }
#endif
            if (!success) {
                esb_flush_tx();
                if (inflight.retries) {
                    inflight.retries--;
                    /* Keep the SAME payload/id ahead of queued B/C. */
                    int err = write_payload(&inflight.payload);
                    if (!err) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE)
                        tx_profile_start = nemu_link_profile_start();
#endif
                        nemu_link_trace_mark(NEMU_LINK_TRACE_TX_START, inflight.payload.length);
                        err = esb_start_tx();
                    }
                    if (!err) {
                        return;
                    }
                    esb_flush_tx();
                }
            }
            complete(success);
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (m_mode == APP_ESB_MODE_PTX) {
            pump();
        }
#else
        pump();
#endif
    } else if (event->evt_id == ESB_EVENT_RX_RECEIVED) {
        struct esb_payload payload;
        while (!esb_read_rx_payload(&payload)) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            diag_rx++;
            if (payload.pipe < NEMU_LINK_HOP_SOURCES) {
                diag_rx_pipe[payload.pipe]++;
            }
#endif
            app_esb_event_t rx = {.evt_type = APP_ESB_EVT_RX,
                                  .pipe = payload.pipe,
                                  /* ESB stores RSSISAMPLE as a positive magnitude. */
                                  .rssi_dbm = (int8_t)-payload.rssi,
                                  .buf = payload.data,
                                  .data_length = payload.length};
            m_callback(&rx);
        }
        pump();
    }
}
static void discard_queue(void) {
    for (uint8_t pipe = 0; pipe < NEMU_LINK_HOP_SOURCES; pipe++) {
        for (uint8_t i = 0; i < ack_count[pipe]; i++) {
            app_esb_event_t event = {
                .evt_type = APP_ESB_EVT_TX_FAIL, .pipe = pipe, .msg_id = ack_msg_id[pipe][i]};
            m_callback(&event);
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        memset(ack_control[pipe], 0, sizeof(ack_control[pipe]));
#endif
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    control_pending = false;
#endif
    memset(ack_count, 0, sizeof(ack_count));
    memset(ack_msg_id, 0, sizeof(ack_msg_id));
    struct hop_tx entry;
    while (!k_msgq_get(&hop_tx_queue, &entry, K_NO_WAIT)) {
        if (m_mode == APP_ESB_MODE_PTX) {
            app_esb_event_t event = {.evt_type = APP_ESB_EVT_TX_FAIL};
            m_callback(&event); /* Balance adapter pending; never ACK delivery. */
        }
    }
}
int nemu_link_radio_discard_pipe(uint8_t pipe) {
    if (pipe >= NEMU_LINK_HOP_SOURCES || m_mode != APP_ESB_MODE_PRX) {
        return -EINVAL;
    }
    unsigned int key = irq_lock();
    if (!m_active) {
        irq_unlock(key);
        return -EACCES;
    }
    /* Retire only ACKs which the driver really consumed before flushing. */
    reconcile_ack_fifo();
    int err = esb_ack_payload_flush(pipe);
    if (err) {
        irq_unlock(key);
        return err;
    }
    for (uint8_t i = 0; i < ack_count[pipe]; i++) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (ack_control[pipe][i]) {
            control_pending = false;
        }
#endif
        app_esb_event_t event = {
            .evt_type = APP_ESB_EVT_TX_FAIL, .pipe = pipe, .msg_id = ack_msg_id[pipe][i]};
        m_callback(&event);
    }
    ack_count[pipe] = 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    memset(ack_control[pipe], 0, sizeof(ack_control[pipe]));
#endif
    memset(ack_msg_id[pipe], 0, sizeof(ack_msg_id[pipe]));
    unsigned int queued = k_msgq_num_used_get(&hop_tx_queue);
    for (unsigned int i = 0; i < queued; i++) {
        struct hop_tx entry;
        if (k_msgq_get(&hop_tx_queue, &entry, K_NO_WAIT)) {
            break;
        }
        if (entry.payload.pipe != pipe) {
            k_msgq_put(&hop_tx_queue, &entry, K_NO_WAIT);
            continue;
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (is_control(&entry.payload)) {
            control_pending = false;
        }
#endif
        app_esb_event_t event = {.evt_type = APP_ESB_EVT_TX_FAIL, .pipe = pipe, .msg_id = entry.id};
        m_callback(&event);
    }
    pump_locked();
    irq_unlock(key);
    return 0;
}
int nemu_link_radio_send(app_esb_data_t *packet) {
    if (!packet || !packet->data || !packet->len || packet->len > CONFIG_ESB_MAX_PAYLOAD_LENGTH ||
        packet->pipe >= NEMU_LINK_HOP_SOURCES) {
        return -EINVAL;
    }
    if (!m_enabled || !m_active) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        LOG_WRN("radio send rejected enabled=%u active=%u mode=%u pipe=%u len=%u", m_enabled,
                m_active, m_mode, packet->pipe, packet->len);
#endif
        return -EACCES;
    }
    struct hop_tx entry = {
        .payload = {.length = (uint8_t)packet->len, .pipe = packet->pipe, .noack = false},
        .id = packet->msg_id,
        .retries = packet->max_retry,
        .replaceable = packet->replaceable};
    memcpy(entry.payload.data, packet->data, packet->len);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    bool control = is_control(&entry.payload);
    if (control && (control_pending || k_msgq_num_used_get(&hop_tx_queue) >=
                                           CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS - 2)) {
        return -EBUSY;
    }
#endif
    if (is_audio(&entry.payload) &&
        k_msgq_num_used_get(&hop_tx_queue) + (tx_active ? 1U : 0U) >=
            CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS - AUDIO_QUEUE_RESERVE) {
        return -EAGAIN;
    }
    unsigned int key = irq_lock();
    if (m_mode == APP_ESB_MODE_PRX && entry.replaceable) {
        /* A newer ACK supersedes older queued ACKs for the same peer. Keep
         * control and handshake frames in order; never claim discarded ACKs
         * were delivered. Hardware-owned entries are not modified here. */
        unsigned int queued = k_msgq_num_used_get(&hop_tx_queue);
        for (unsigned int i = 0; i < queued; i++) {
            struct hop_tx old;
            if (k_msgq_get(&hop_tx_queue, &old, K_NO_WAIT)) {
                break;
            }
            if (old.payload.pipe == entry.payload.pipe && old.replaceable) {
                app_esb_event_t event = {
                    .evt_type = APP_ESB_EVT_TX_FAIL, .pipe = old.payload.pipe, .msg_id = old.id};
                m_callback(&event);
            } else {
                k_msgq_put(&hop_tx_queue, &old, K_NO_WAIT);
            }
        }
    }
    int err = k_msgq_put(&hop_tx_queue, &entry, K_NO_WAIT);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    if (!err && control) {
        control_pending = true;
    }
#endif
    if (!err) {
        pump_locked();
    }
    irq_unlock(key);
    return err;
}
void nemu_link_radio_service(void) {
    if (repair_required && m_enabled && !m_active && hf_clock_held) {
        repair_required = start_radio() != 0;
    }
    pump();
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
void nemu_link_radio_diag_log(void) {
    unsigned int key = irq_lock();
    LOG_INF("radio diag mode=%u enabled=%u active=%u clock=%u channel=%u tx=%u ok=%u fail=%u rx=%u "
            "write_err=%u queue=%u",
            m_mode, m_enabled, m_active, hf_clock_held, radio_index, diag_tx_start, diag_tx_ok,
            diag_tx_fail, diag_rx, diag_write_error, k_msgq_num_used_get(&hop_tx_queue));
    LOG_INF("radio pipes rx0=%u rx1=%u write0=%u write1=%u retired=%u reconciled=%u hw_full=%u",
            diag_rx_pipe[0], diag_rx_pipe[1], diag_write_pipe[0], diag_write_pipe[1],
            diag_ack_retired, diag_ack_reconciled, esb_tx_full());
    bool pending_control = false;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    pending_control = control_pending;
#endif
    LOG_INF("radio ACK ledger count0=%u count1=%u control=%u", ack_count[0],
            ack_count[1], pending_control);
    irq_unlock(key);
}
#endif
uint8_t nemu_link_radio_index(void) { return radio_index; }
int nemu_link_radio_channel(uint8_t index) {
    if (index >= nemu_link_hop_channel_count) {
        return -EINVAL;
    }
    if (!m_enabled || !m_active) {
        return -EACCES;
    }
    if (tx_active || (m_mode == APP_ESB_MODE_PTX && !esb_is_idle())) {
        return -EBUSY;
    }
    if (radio_index == index) {
        return 0;
    }
    uint8_t previous = radio_index;
    stop_radio();
    discard_queue();
    radio_index = index;
    int err = start_radio();
    if (err) {
        radio_index = previous;
        repair_required = start_radio() != 0;
    }
    return err;
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
int nemu_link_radio_scan(uint8_t index, uint16_t *busy, uint16_t *samples) {
    if (!m_enabled || !m_active || m_mode != APP_ESB_MODE_PRX) {
        return -EACCES;
    }
    if (k_msgq_num_used_get(&hop_tx_queue) || !nemu_link_esb_scan_ready()) {
        return -EBUSY;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    if (control_pending) {
        return -EBUSY;
    }
#endif
    int err = esb_stop_rx();
    if (err) {
        return err;
    }
    nemu_link_hw_scan(nemu_link_hop_channels[index], busy, samples);
    /* No esb_disable/init: retain hardware ACKs and per-pipe PID/CRC history. */
    err = esb_start_rx();
    if (err) {
        stop_radio();
        discard_queue();
        repair_required = true;
        return err;
    }
    return *samples ? 0 : -EIO;
}

int nemu_link_radio_scan_frequency_locked(uint8_t channel, int8_t *minimum, int8_t *maximum,
                                          int8_t *mean, uint16_t *samples) {
    if (channel >= NEMU_LINK_FULL_SCAN_CHANNEL_COUNT || !minimum || !maximum || !mean || !samples) {
        return -EINVAL;
    }
    if (!m_enabled || !m_active || m_mode != APP_ESB_MODE_PRX) {
        return -EACCES;
    }
    /* Full-band scans are explicitly requested and may pause PRX briefly.
     * Do not wait for the normal traffic queue to become empty: a steady
     * stream of ACK payloads would otherwise starve scan results. */
    int err = esb_stop_rx();
    if (err) {
        return err;
    }
    nemu_link_hw_scan_rssi(channel, minimum, maximum, mean, samples);
    /* RSSI scanning programs RADIO->FREQUENCY directly, outside ESB's cache.
     * Restore the applied Nemu-link channel before resuming PRX. */
    NRF_RADIO->FREQUENCY = nemu_link_hop_channels[radio_index];
    err = esb_start_rx();
    if (err) {
        stop_radio();
        discard_queue();
        repair_required = true;
        return err;
    }
    /* A quiet channel is a valid result with samples == 0. Keep transport
     * failures from esb_stop_rx()/esb_start_rx() distinguishable to callers. */
    return 0;
}
#endif
int nemu_link_radio_clear_pending(void) {
    if (!m_enabled || !m_active) {
        return -EACCES;
    }
    if (tx_active || (m_mode == APP_ESB_MODE_PTX && !esb_is_idle())) {
        return -EBUSY;
    }
    unsigned int key = irq_lock();
    stop_radio();
    discard_queue();
    int err = start_radio();
    repair_required = err != 0;
    irq_unlock(key);
    return err;
}
int nemu_link_radio_init(app_esb_mode_t mode, app_esb_callback_t callback) {
    /* Both roles defer RF until the adapter enables them. The upstream central
     * otherwise starts PRX from POST_KERNEL, before USB/application readiness. */
    m_mode = mode;
    m_callback = callback;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE) && IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    nemu_link_radio_profile(CONFIG_ZMK_NEMU_LINK_DONGLE_ID);
#endif
    return 0;
}
int nemu_link_radio_enable(bool value) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    LOG_INF("radio enable request value=%u enabled=%u active=%u mode=%u clock=%u", value, m_enabled,
            m_active, m_mode, hf_clock_held);
#endif
    if (value && m_enabled) {
        return 0;
    }
    if (!value && !m_enabled && !m_active && !hf_clock_held) {
        return 0;
    }
    if (!value) {
        unsigned int key = irq_lock();
        m_enabled = false;
        repair_required = false;
        if (m_active) {
            stop_radio();
        }
        if (tx_active) {
            complete(false);
        }
        discard_queue();
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
        nemu_link_ccm_radio_reset();
#endif
        irq_unlock(key);
        return nemu_link_clock_release();
    }
    uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX] = NEMU_LINK_DEFAULT_RADIO_CHANNELS;
    uint8_t channel_count = NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT;
    if (nemu_link_key_store_read_radio_channels(channels, &channel_count)) {
        channel_count = NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT;
    }
    if (!nemu_link_hop_set_channels(channels, channel_count)) {
        return -EINVAL;
    }
    int err = nemu_link_clock_acquire(2000);
    if (err) {
        return err;
    }
    unsigned int key = irq_lock();
    radio_index = 0;
    err = start_radio();
    m_enabled = !err;
    irq_unlock(key);
    if (err) {
        nemu_link_clock_release();
    }
    return err;
}
uint8_t nemu_link_radio_debug_state(void) {
    return (uint8_t)((m_enabled ? BIT(0) : 0) | (m_active ? BIT(1) : 0));
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
int nemu_link_radio_sleep(void) {
    if (!m_enabled || !m_active || m_mode != APP_ESB_MODE_PTX || tx_active || !esb_is_idle() ||
        k_msgq_num_used_get(&hop_tx_queue)) {
        return -EBUSY;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_STOP_BEGIN, 0);
    uint32_t profile_start = nemu_link_profile_start();
    stop_radio();
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_DISABLED, 0);
    int err = nemu_link_clock_release();
    nemu_link_trace_mark(NEMU_LINK_TRACE_CLOCK_RELEASED, (uint16_t)-err);
    nemu_link_profile_end(NEMU_LINK_PROFILE_STOP, profile_start);
    nemu_link_trace_mark(NEMU_LINK_TRACE_STOP_END, (uint16_t)-err);
    if (err) {
        start_radio();
    }
    return err;
}
int nemu_link_radio_wake(void) {
    if (!m_enabled || m_mode != APP_ESB_MODE_PTX) {
        return -EACCES;
    }
    if (m_active) {
        return 0;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_CLOCK_BEGIN, 0);
    uint32_t profile_start = nemu_link_profile_start();
    int clock_err = nemu_link_clock_acquire(CONFIG_ZMK_NEMU_LINK_IDLE_WAKE_TIMEOUT_MS);
    nemu_link_profile_end(NEMU_LINK_PROFILE_CLOCK, profile_start);
    nemu_link_trace_mark(NEMU_LINK_TRACE_CLOCK_READY, (uint16_t)-clock_err);
    if (clock_err) {
        return clock_err;
    }
    unsigned int key = irq_lock();
    nemu_link_trace_mark(NEMU_LINK_TRACE_INIT_BEGIN, 0);
    profile_start = nemu_link_profile_start();
    int err = start_radio();
    nemu_link_profile_end(NEMU_LINK_PROFILE_INIT, profile_start);
    nemu_link_trace_mark(NEMU_LINK_TRACE_INIT_END, (uint16_t)-err);
    irq_unlock(key);
    if (err) {
        nemu_link_clock_release();
    }
    return err;
}
#endif
