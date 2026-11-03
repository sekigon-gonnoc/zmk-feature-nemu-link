/* ZMK link adapter: authenticated sessions, snapshots, hopping and role service. */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/nemu_link/split_frame.h>
#include <zmk/nemu_link/secure.h>
#include <zmk/nemu_link/mini.h>
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
#include <zmk/nemu_link/ccm_radio.h>
#endif
#include <zmk/nemu_link/power_profile.h>
#include <zmk/nemu_link/power_trace.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/split/transport/peripheral.h>
#endif
#include <zmk/pointing/input_split.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
extern const struct zmk_split_transport_peripheral *zmk_nemu_link_peripheral_transport(void);
#endif
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
#include <zmk/battery.h>
#endif
#include <zmk/keymap.h>
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#include <zmk/endpoints.h>
#include <zmk/nemu_link/battery_status.h>
#elif !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/endpoints.h>
#endif
#include <zmk/matrix.h>
#include <zmk/physical_layouts.h>
#include <zmk/split/transport/central.h>
#include <zmk/nemu_link/link.h>
#include <zmk/nemu_link/link_timing.h>
#include <zmk/nemu_link/link_frame.h>
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/internal.h>
#include <zmk/nemu_link/hop_radio.h>
#include "radio.h"
#include "link_internal.h"
#include <zmk/nemu_link/selection.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
#include <zmk/nemu_link/studio_zmk.h>
#endif
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
#include <zmk/nemu_link/settings_relay.h>
#include <zmk/nemu_link/studio_transport.h>
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
#include <zmk/nemu_link/idle.h>
#endif

LOG_MODULE_REGISTER(nemu_link_link, LOG_LEVEL_INF);

#define CENTRAL IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#define FRAME_CRC NULL
#include <zmk/nemu_link/hop_frame.h>
#include <zmk/nemu_link/hop_link.h>
#include <zmk/nemu_link/hop_radio.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
#include <zmk/nemu_link/channel_quality.h>
static struct nemu_link_channel_policy channel_policy;
static bool adaptive_pending;
static uint32_t scan_attempts, scan_busy, scan_errors;
static const struct nemu_link_channel_config channel_config = {
    .scan_idle_ms = CONFIG_ZMK_NEMU_LINK_SCAN_IDLE_MS,
    .scan_interval_ms = CONFIG_ZMK_NEMU_LINK_SCAN_INTERVAL_MS,
    .min_scan_windows = CONFIG_ZMK_NEMU_LINK_MIN_SCAN_WINDOWS,
    .degrade_ms = CONFIG_ZMK_NEMU_LINK_MIGRATION_DEGRADE_MS,
    .validation_ms = 12 * NEMU_LINK_LINK_INTERVAL_MS + 100,
    .cooldown_ms = CONFIG_ZMK_NEMU_LINK_MIGRATION_COOLDOWN_MS,
    .penalty_ms = CONFIG_ZMK_NEMU_LINK_FAILED_CHANNEL_PENALTY_MS,
    .loss_threshold = CONFIG_ZMK_NEMU_LINK_MIGRATION_LOSS_PERMILLE,
    .min_link_samples = 8,
};
#endif
#define FRAME_SIZE NEMU_LINK_HOP_FRAME_SIZE
static const struct nemu_link_hop_timing hop_timing = {
    .loss_ms = MAX(2 * NEMU_LINK_LINK_INTERVAL_MS + 100, 1000),
    .announce_ms = NEMU_LINK_LINK_MODE_SETTLE_MS,
    .probe_ms = 20,
    .peer_dwell_ms = 100,
    .central_dwell_ms = 700};
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static struct nemu_link_hop_central hop;

static uint64_t next_hop_test;
#else
static struct nemu_link_hop_peripheral hop;
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ||                                                   \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
static bool start_pending;
static atomic_t receiver_ready;
void zmk_nemu_link_set_receiver_ready(bool ready) { atomic_set(&receiver_ready, ready); }
#endif
static uint16_t tx_id;
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
#define SETTINGS_FRAGMENT_TX_TIMEOUT_MS 4000
static atomic_t settings_tx_wait_id;
static atomic_t settings_tx_status;
static atomic_t settings_tx_done_id;
K_SEM_DEFINE(settings_tx_done, 0, 1);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static struct {
    bool pending;
    uint8_t source;
    uint16_t msg_id;
    struct nemu_link_studio_packet packet;
    struct nemu_link_studio_route route;
} settings_request_tx;
#endif
#endif
#define SERVICE_MS 10

#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#define NEMU_LINK_RADIO_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(sekigon_nemu_link_radio)
BUILD_ASSERT(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_COUNT == NEMU_LINK_LINK_SOURCES);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
BUILD_ASSERT(CONFIG_ZMK_NEMU_LINK_DONGLE_ID <= CONFIG_ZMK_NEMU_LINK_DONGLE_COUNT);
#endif
BUILD_ASSERT(DT_PROP_LEN(NEMU_LINK_RADIO_NODE, addr_prefix) == NEMU_LINK_LINK_SOURCES,
             "One ESB RX pipe per source required");
BUILD_ASSERT(DT_PROP_BY_IDX(NEMU_LINK_RADIO_NODE, addr_prefix, 0) !=
                 DT_PROP_BY_IDX(NEMU_LINK_RADIO_NODE, addr_prefix, 1),
             "ESB prefixes must be distinct");
BUILD_ASSERT(CONFIG_ESB_TX_FIFO_SIZE >= 2 * NEMU_LINK_LINK_SOURCES,
             "Reserve ACK slots for both sources");
#endif
BUILD_ASSERT(CONFIG_ESB_MAX_PAYLOAD_LENGTH >= MAX(FRAME_SIZE, NEMU_LINK_SPLIT_FRAME_SIZE));
BUILD_ASSERT(NEMU_LINK_LINK_RELEASE_MS >= 5 * NEMU_LINK_LINK_INTERVAL_MS);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
BUILD_ASSERT(CONFIG_ZMK_NEMU_LINK_IDLE_GRACE_MS + CONFIG_ZMK_NEMU_LINK_IDLE_WAKE_TIMEOUT_MS +
                     SERVICE_MS <
                 NEMU_LINK_LINK_INTERVAL_MS,
             "Idle grace and wake must fit inside the snapshot interval");
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
BUILD_ASSERT(IS_ENABLED(CONFIG_ZMK_NEMU_LINK_HOP));
#endif
#if !IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
BUILD_ASSERT(!IS_ENABLED(CONFIG_ZMK_STUDIO) && !IS_ENABLED(CONFIG_ZMK_KEYMAP_SETTINGS_STORAGE));

#endif
#define CHECK_LAYER(node)                                                                          \
    BUILD_ASSERT(DT_PROP_LEN(node, bindings) > 0 && DT_PROP_LEN(node, bindings) <= 128,            \
                 "Unsupported keymap position count");
ZMK_KEYMAP_LAYERS_FOREACH(CHECK_LAYER)

K_MSGQ_DEFINE(link_rx, sizeof(struct rx_frame), 8, 4);
K_MUTEX_DEFINE(link_lock);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
int nemu_link_radio_scan_frequency(uint8_t channel, int8_t *minimum, int8_t *maximum, int8_t *mean,
                                   uint16_t *samples) {
    k_mutex_lock(&link_lock, K_FOREVER);
    unsigned int key = irq_lock();
    int err = nemu_link_radio_scan_frequency_locked(channel, minimum, maximum, mean, samples);
    irq_unlock(key);
    k_mutex_unlock(&link_lock);
    return err;
}
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
/* Every power-on starts at dongle 1. Selection changes remain in RAM only. */
static uint8_t selected_dongle = 1;
static struct nemu_link_selection selection = {.target = 1};
#endif
bool enabled;
static uint64_t next_stats;
static struct nemu_link_secure_peer secure_peers[NEMU_LINK_LINK_SOURCES];
struct nemu_link_secure_peer *secure_peer(uint8_t source) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    return &secure_peers[source];
#else
    ARG_UNUSED(source);
    return &secure_peers[CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID];
#endif
}
static uint8_t secure_dongle_id(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    return selected_dongle;
#else
    return 1;
#endif
}
static atomic_t rx_dropped, bad_rx;
static uint32_t tx_errors;
enum bad_rx_reason {
    BAD_RX_BAD_LENGTH,
    BAD_RX_BAD_PIPE,
    BAD_RX_QUEUE_STALE,
    BAD_RX_AUTH,
    BAD_RX_SECURE,
    BAD_RX_MINI,
    BAD_RX_SPLIT,
    BAD_RX_HOP_FRAME,
    BAD_RX_STALE,
    BAD_RX_NOTICE,
    BAD_RX_REASON_COUNT,
};
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
static atomic_t bad_rx_source[NEMU_LINK_LINK_SOURCES][BAD_RX_REASON_COUNT];
static atomic_t bad_rx_global[BAD_RX_REASON_COUNT];
#endif
static void bad_rx_record(uint8_t source, enum bad_rx_reason reason) {
    atomic_inc(&bad_rx);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    if (source < NEMU_LINK_LINK_SOURCES) {
        atomic_inc(&bad_rx_source[source][reason]);
    } else {
        atomic_inc(&bad_rx_global[reason]);
    }
#else
    ARG_UNUSED(source);
    ARG_UNUSED(reason);
#endif
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
/* Metadata only: never log key material, auth randoms or frame contents. */
static struct {
    uint32_t auth_rx, auth_rejected, data_rx, data_rejected, accepted;
    int last_auth_error, last_data_error;
    uint8_t last_wire_kind;
    struct nemu_link_secure_auth_diag auth_detail;
} peer_diag[NEMU_LINK_LINK_SOURCES];
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
struct nemu_link_link_central central;
static uint32_t requested_mode[NEMU_LINK_LINK_SOURCES];
static uint32_t confirmed_mode[NEMU_LINK_LINK_SOURCES];
static atomic_t diag_rx_rssi_last_dbm[NEMU_LINK_LINK_SOURCES];
static atomic_t diag_rx_rssi_valid[NEMU_LINK_LINK_SOURCES];
static uint32_t mode_generation;
static bool ble_return_pending;
static uint8_t ble_return_sources;
static uint8_t ble_return_attempts[NEMU_LINK_LINK_SOURCES];
static uint64_t ble_return_deadline_ms;
#define BLE_RETURN_MAX_ATTEMPTS 3
#define BLE_RETURN_TIMEOUT_MS 5000
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
static uint32_t diag_decode_rejected, diag_notice_rejected, diag_stale;
static uint32_t diag_kind[NEMU_LINK_LINK_ACK + 1];
static uint32_t diag_hop_rejected, diag_link_rejected, diag_link_fresh;
static uint32_t diag_hop_rejected_source[NEMU_LINK_LINK_SOURCES];
static uint32_t diag_link_rejected_source[NEMU_LINK_LINK_SOURCES];
static uint32_t diag_link_fresh_source[NEMU_LINK_LINK_SOURCES];
struct rx_rssi_stats {
    /* Window counters reset after each log; last_dbm and avg_dbm_x8 persist. */
    bool have_last, have_avg;
    uint32_t samples;
    /* EWMA alpha is 1/8; retain eighth-dBm precision between updates. */
    int32_t avg_dbm_x8;
    int8_t last_dbm, min_dbm, max_dbm;
};
static struct rx_rssi_stats diag_rx_rssi[NEMU_LINK_LINK_SOURCES];
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
static struct nemu_link_selection peer_selection[NEMU_LINK_LINK_SOURCES];
#endif
static uint8_t battery[NEMU_LINK_LINK_SOURCES] = {255, 255};
#else
struct nemu_link_link_peripheral peripheral;
static struct nemu_link_link_keys physical_keys;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
static uint8_t profile_target;
static uint64_t selection_ignore_until;
static bool selection_fault_started;
static void profile_switch(struct k_work *work);
static K_WORK_DEFINE(profile_work, profile_switch);
#endif

static bool search_active;
static uint64_t search_started_ms;

/* Keep a short existing dwell/probe window to collect delayed ACK payloads.
 * An unsuccessful HELLO restart must not renew the fast-search period. */
static void search_update(uint64_t now) {
    bool searching = hop.searching || peripheral.phase != NEMU_LINK_LINK_PERIPHERAL_UP;
    if (searching && !search_active) {
        search_started_ms = now;
    }
    search_active = searching;
}
/* Local activity renews fast search; the existing 10ms worker handles probes. */
void search_input(uint64_t now) {
    if (!enabled) {
        return;
    }
    search_update(now);
    if (search_active) {
        search_started_ms = now;
    }
}
static uint64_t search_window_start(uint64_t slow_start, uint64_t cycle) {
    const uint64_t interval = CONFIG_ZMK_NEMU_LINK_SEARCH_INTERVAL_MS;
    /* Vary the window by at most 600ms so it cannot lock to dongle patrol. */
    uint64_t jitter = MIN(600U, (interval - hop_timing.peer_dwell_ms) / 2);
    return slow_start + cycle * interval + (cycle * 317U) % (jitter + 1);
}
static uint64_t search_next(uint64_t at) {
    uint64_t slow_start = search_started_ms + CONFIG_ZMK_NEMU_LINK_SEARCH_FAST_MS;
    if (!search_active || at < slow_start) {
        return at;
    }
    uint64_t cycle = (at - slow_start) / CONFIG_ZMK_NEMU_LINK_SEARCH_INTERVAL_MS;
    uint64_t window = search_window_start(slow_start, cycle);
    if (at < window) {
        return window;
    }
    if (at < window + hop_timing.peer_dwell_ms) {
        return at;
    }
    return search_window_start(slow_start, cycle + 1);
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
static uint64_t next_tx_ms(uint64_t now) {
    uint64_t next = peripheral.has_sent ? peripheral.last_sent_ms + peripheral.interval_ms : now;
    next = search_next(MAX(next, now));
    return next;
}
#endif
static uint32_t received_mode;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_AUTO_BLE)
static uint64_t dongle_confirmed_ms;
#endif
static bool auto_ble_due(uint64_t now) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_AUTO_BLE)
    return now - dongle_confirmed_ms >= CONFIG_ZMK_NEMU_LINK_AUTO_BLE_TIMEOUT_MS;
#else
    ARG_UNUSED(now);
    return false;
#endif
}
static bool release_dropped;
static struct nemu_link_link_keys fault_seen_press;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
/* A completion count also covers the gap between RADIO idle and deferred ESB
 * event delivery. Protected against send/IRQ races by irq_lock in send_message. */
static atomic_t tx_pending;
static struct dt_idle idle;
static uint64_t retry_wake_ms;
static uint32_t idle_sleeps, idle_wakes, idle_errors;
static uint64_t idle_off_since, idle_off_ms;
static uint32_t wake_max_ms;
#define WAKE_MARGIN_MS CONFIG_ZMK_NEMU_LINK_IDLE_WAKE_MARGIN_MS

int idle_wake(void) {
    if (idle.phase != NEMU_LINK_IDLE_OFF) {
        return 0;
    }
    if (k_uptime_get() < retry_wake_ms) {
        return -EAGAIN;
    }
    uint64_t wake_start = k_uptime_get();
    idle_off_ms += wake_start - idle_off_since;
    idle.phase = NEMU_LINK_IDLE_WAKING;
    int err = nemu_link_radio_wake();
    if (err) {
        idle.phase = NEMU_LINK_IDLE_OFF;
        idle_off_since = k_uptime_get();
        idle_errors++;
        retry_wake_ms = k_uptime_get() + CONFIG_ZMK_NEMU_LINK_IDLE_RETRY_MS;
        /* Never renew liveness or consider a failed wake a successful send.
         * The existing link LOST -> fresh HELLO path handles reconnection. */
        peripheral.phase = NEMU_LINK_LINK_PERIPHERAL_LOST;
        LOG_WRN("idle wake failed err=%d; link reconnect required", err);
        return err;
    }
    wake_max_ms = MAX(wake_max_ms, (uint32_t)(k_uptime_get() - wake_start));
    idle.phase = NEMU_LINK_IDLE_AWAKE;
    idle_wakes++;
    return 0;
}
#endif
#endif

bool nemu_link_studio_get_peripheral_rssi(uint8_t source, int8_t *rssi_dbm) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (!rssi_dbm || source >= NEMU_LINK_LINK_SOURCES || !atomic_get(&diag_rx_rssi_valid[source])) {
        return false;
    }
    *rssi_dbm = (int8_t)atomic_get(&diag_rx_rssi_last_dbm[source]);
    return true;
#else
    ARG_UNUSED(source);
    ARG_UNUSED(rssi_dbm);
    return false;
#endif
}

static void service(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(link_work, service);
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
void nemu_link_link_ccm_ready(void) {
    if (enabled)
        k_work_reschedule(&link_work, K_NO_WAIT);
}
#endif

#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
extern const struct zmk_split_transport_central *active_transport;
#endif

static uint32_t fresh_nonce(void) {
    uint32_t value = 0;
    if (sys_csrand_get(&value, sizeof(value)) != 0) {
        return 0;
    }
    return value;
}

bool nemu_link_link_input_allowed(uint8_t source) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    if (selection.target != selected_dongle) {
        return false;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    return source < NEMU_LINK_LINK_SOURCES &&
           nemu_link_selection_equal(selection, peer_selection[source]);
#else
    return source == CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID;
#endif
#else
    ARG_UNUSED(source);
    return true;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
static struct {
    uint32_t count, bytes;
    uint8_t min, max;
} wire_tx[NEMU_LINK_LINK_SOURCES][NEMU_LINK_WIRE_KIND_COUNT];
#endif
static uint8_t wire_kind(const uint8_t *bytes, size_t size) {
    if (size == NEMU_LINK_MINI_HEARTBEAT_SIZE)
        return NEMU_LINK_WIRE_EMPTY;
    if (size == NEMU_LINK_MINI_ACK_SIZE)
        return NEMU_LINK_WIRE_ACK;
    return size >= 2 ? bytes[1] : 0;
}
static const struct zmk_nemu_link_audio_callbacks *audio_callbacks;

int zmk_nemu_link_register_audio_callbacks(const struct zmk_nemu_link_audio_callbacks *callbacks) {
    if (!callbacks || (!callbacks->fragment && !callbacks->control && !callbacks->source_lost)) {
        return -EINVAL;
    }
    if (audio_callbacks) {
        return -EALREADY;
    }
    audio_callbacks = callbacks;
    return 0;
}

static int send_frame_internal(uint8_t source, uint8_t *bytes, size_t size, bool wait_for_tx,
                               uint16_t *wait_id) {
    if (!size || size > NEMU_LINK_FRAME_MAX_SIZE) {
        return -EINVAL;
    }
    app_esb_data_t packet = {.pipe = source,
                             .data = bytes,
                             .len = size,
                             .msg_id = 0,
                             .max_retry = 0,
                             .replaceable = CENTRAL && !wait_for_tx &&
                                            wire_kind(bytes, size) == NEMU_LINK_WIRE_ACK};
    /* Live audio is obsolete as soon as its playout deadline passes. Avoid
     * replaying stale fragments ahead of current input and Studio traffic. */
    if (wire_kind(bytes, size) != NEMU_LINK_WIRE_AUDIO) {
        packet.max_retry = CONFIG_ZMK_NEMU_LINK_DELIVERY_RETRIES;
    }
    unsigned int key = irq_lock();
    if (++tx_id == 0) {
        ++tx_id;
    }
    packet.msg_id = tx_id;
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    if (wait_for_tx) {
        k_sem_reset(&settings_tx_done);
        atomic_set(&settings_tx_status, 0);
        atomic_set(&settings_tx_done_id, 0);
        atomic_set(&settings_tx_wait_id, tx_id);
    }
#else
    ARG_UNUSED(wait_for_tx);
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    atomic_inc(&tx_pending);
#endif
    nemu_link_trace_mark(NEMU_LINK_TRACE_QUEUE_BEGIN,
                         (uint16_t)((wire_kind(bytes, size) << 8) | size));
    int err = nemu_link_radio_send(&packet);
    if (!err && wait_for_tx && wait_id) {
        *wait_id = packet.msg_id;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_QUEUE_END, (uint16_t)-err);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    if (err) {
        atomic_dec(&tx_pending);
    }
#endif
    irq_unlock(key);
    if (err) {
        tx_errors++;
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (wait_for_tx && atomic_get(&settings_tx_wait_id) == packet.msg_id) {
            atomic_set(&settings_tx_wait_id, 0);
        }
#endif
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    uint8_t kind = wire_kind(bytes, size);
    bool secured = size >= 11 && bytes[0] == NEMU_LINK_SECURE_WIRE_VERSION;
    secured |= size == NEMU_LINK_MINI_HEARTBEAT_SIZE || size == NEMU_LINK_MINI_ACK_SIZE;
    if (!err && source < NEMU_LINK_LINK_SOURCES && secured && kind >= NEMU_LINK_WIRE_LINK &&
        kind <= NEMU_LINK_WIRE_AUDIO_CONTROL) {
        unsigned k = kind - NEMU_LINK_WIRE_LINK;
        if (!wire_tx[source][k].count || size < wire_tx[source][k].min)
            wire_tx[source][k].min = size;
        wire_tx[source][k].max = MAX(wire_tx[source][k].max, size);
        wire_tx[source][k].count++;
        wire_tx[source][k].bytes += size;
    }
#endif
    return err;
}

int send_frame(uint8_t source, uint8_t *bytes, size_t size) {
    return send_frame_internal(source, bytes, size, false, NULL);
}

#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
static int wait_for_tx(uint16_t wait_id) {
    if (!wait_id) {
        return -EINVAL;
    }
    if (k_sem_take(&settings_tx_done, K_MSEC(SETTINGS_FRAGMENT_TX_TIMEOUT_MS))) {
        bool completed =
            atomic_get(&settings_tx_done_id) == wait_id && atomic_get(&settings_tx_status) != 0;
        if (!completed) {
            return -ETIMEDOUT;
        }
    }
    return atomic_get(&settings_tx_done_id) == wait_id && atomic_get(&settings_tx_status) == 1
               ? 0
               : -EIO;
}

#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static int send_frame_and_wait(uint8_t source, uint8_t *bytes, size_t size) {
    uint16_t wait_id = 0;
    int err = send_frame_internal(source, bytes, size, true, &wait_id);
    return err ? err : wait_for_tx(wait_id);
}
#endif
#endif

int zmk_nemu_link_audio_send(uint16_t block_sequence, uint8_t fragment, uint8_t fragment_count,
                             const uint8_t *pcm, size_t size) {
#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    if (!pcm || !size || size > NEMU_LINK_AUDIO_DATA_MAX || !fragment_count ||
        fragment >= fragment_count) {
        return -EINVAL;
    }
    uint8_t source = CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID;
    uint8_t native[NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE + NEMU_LINK_AUDIO_DATA_MAX] = {0};
    memcpy(native, "EsAu", 4);
    native[4] = source;
    native[6] = NEMU_LINK_WIRE_AUDIO;
    sys_put_le16(block_sequence, native + 7);
    native[9] = fragment;
    native[10] = fragment_count;
    native[11] = size;
    memcpy(native + NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE, pcm, size);

    k_mutex_lock(&link_lock, K_FOREVER);
    int err = -ENOTCONN;
    if (enabled && peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP &&
        nemu_link_secure_ready(secure_peer(source))) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
        if (idle_wake()) {
            err = -EAGAIN;
            goto audio_out;
        }
#endif
        uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
        size_t encrypted_size = 0;
        err = nemu_link_secure_wrap(secure_peer(source), CENTRAL, source, native,
                                    NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE + size, encrypted,
                                    &encrypted_size);
        if (!err) {
            err = send_frame(source, encrypted, encrypted_size);
        }
    }
audio_out:
    k_mutex_unlock(&link_lock);
    return err;
#else
    ARG_UNUSED(block_sequence);
    ARG_UNUSED(fragment);
    ARG_UNUSED(fragment_count);
    ARG_UNUSED(pcm);
    ARG_UNUSED(size);
    return -ENOTSUP;
#endif
}

int zmk_nemu_link_audio_control(bool mic_enabled, uint8_t origin) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    uint8_t source = CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE;
    uint8_t native[NEMU_LINK_NATIVE_AUDIO_CONTROL_SIZE] = {0};
    memcpy(native, "EsAc", 4);
    native[4] = source;
    native[5] = origin;
    native[6] = NEMU_LINK_WIRE_AUDIO_CONTROL;
    native[7] = mic_enabled ? 1U : 0U;

    k_mutex_lock(&link_lock, K_FOREVER);
    int err = -ENOTCONN;
    if (enabled && source < NEMU_LINK_LINK_SOURCES &&
        central.peers[source].phase == NEMU_LINK_LINK_UP && nemu_link_link_input_allowed(source) &&
        nemu_link_secure_ready(secure_peer(source))) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
        if (idle_wake()) {
            err = -EAGAIN;
            goto audio_control_out;
        }
#endif
        uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
        size_t encrypted_size = 0;
        err = nemu_link_secure_wrap(secure_peer(source), CENTRAL, source, native, sizeof(native),
                                    encrypted, &encrypted_size);
        if (!err) {
            err = send_frame(source, encrypted, encrypted_size);
        }
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
audio_control_out:
#endif
    k_mutex_unlock(&link_lock);
    return err;
#else
    ARG_UNUSED(mic_enabled);
    ARG_UNUSED(origin);
    return -ENOTSUP;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
static uint64_t last_full_state_ms;
static bool full_state_sent;
#endif
static int send_message(const struct nemu_link_link_message *message, uint32_t mode,
                        bool force_full) {
    uint8_t bytes[FRAME_SIZE];
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    struct nemu_link_hop_notice notice = nemu_link_hop_central_notice(&hop);
#else
    struct nemu_link_hop_notice notice = nemu_link_hop_link_notice(&peripheral, &hop);
#endif
    /* A single-dongle peripheral still talks to dongle 1. Keep its hop-frame
     * address compatible with a multi-dongle receiver enforcing the ID. */
    notice.dongle_id = secure_dongle_id();
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    if (!mode && (message->kind == NEMU_LINK_LINK_STATE || message->kind == NEMU_LINK_LINK_ACK)) {
        mode = nemu_link_selection_encode(selection);
    }
#endif
    struct nemu_link_secure_peer *mini_peer = secure_peer(message->source);
    struct nemu_link_link_message remapped = *message;
    ARG_UNUSED(remapped);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC) && IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (message->kind == NEMU_LINK_LINK_ACK &&
        message->source == CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE) {
        remapped.studio_pending = nemu_link_studio_pending();
        message = &remapped;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    if (message->kind == NEMU_LINK_LINK_STATE && nemu_link_secure_ready(mini_peer)) {
        /* Every STATE shares the secure counter, including periodic full.
         * Keep the existing ledger's original generation time. */
        if (mini_peer->tx_sequence < message->sequence - 1) {
            mini_peer->tx_sequence = message->sequence - 1;
        }
        remapped.sequence = mini_peer->tx_sequence + 1;
        for (unsigned i = 0; i < NEMU_LINK_LINK_ACK_SLOTS; i++) {
            if (peripheral.sent[i].valid && peripheral.sent[i].sequence == message->sequence) {
                peripheral.sent[i].sequence = remapped.sequence;
            }
        }
        peripheral.sequence = remapped.sequence;
        message = &remapped;
    }
    uint64_t mini_now = k_uptime_get();
    bool mini_due = peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP && full_state_sent &&
                    mini_now - last_full_state_ms < (uint64_t)NEMU_LINK_LINK_INTERVAL_MS *
                                                        CONFIG_ZMK_NEMU_LINK_FULL_STATE_INTERVALS;
#else
    bool mini_due = central.peers[message->source].phase == NEMU_LINK_LINK_UP;
#endif
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    nemu_link_ccm_radio_context(mini_peer, message->source, peripheral.sequence);
#endif
    if (!force_full && mini_due && nemu_link_mini_can_send(mini_peer, message, mode, &notice)) {
        uint8_t mini_wire[NEMU_LINK_FRAME_MAX_SIZE];
        size_t mini_size = 0;
        nemu_link_trace_mark(NEMU_LINK_TRACE_PACK_BEGIN, message->kind);
        nemu_link_trace_mark(NEMU_LINK_TRACE_PACK_END, CENTRAL ? 3 : 1);
        int mini_err = nemu_link_mini_wrap(mini_peer, CENTRAL, message->source, message, mini_wire,
                                           &mini_size);
        int sent = mini_err ? mini_err : send_frame(message->source, mini_wire, mini_size);
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
        if (sent && !mini_err)
            nemu_link_ccm_radio_retire(message->source, mini_wire, mini_size);
#endif
        return sent;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCODE_BEGIN, message->kind);
    size_t size =
        nemu_link_hop_frame_encode(bytes, sizeof(bytes), message, mode, &notice, FRAME_CRC);
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCODE_END, size);
    if (!size) {
        return -EINVAL;
    }
    struct nemu_link_secure_peer *peer = secure_peer(message->source);
    if (!nemu_link_secure_ready(peer)) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
        uint8_t auth[NEMU_LINK_FRAME_MAX_SIZE];
        size_t auth_size =
            nemu_link_secure_auth_next(peer, message->source, secure_dongle_id(), auth);
        return auth_size ? send_frame(message->source, auth, auth_size) : -ENOTCONN;
#else
        return -ENOTCONN;
#endif
    }
    uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
    size_t encrypted_size = 0;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    bool compact = !force_full && central.peers[message->source].phase == NEMU_LINK_LINK_UP;
#else
    uint64_t now = k_uptime_get();
    bool compact = !force_full && peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP &&
                   full_state_sent &&
                   now - last_full_state_ms < (uint64_t)NEMU_LINK_LINK_INTERVAL_MS *
                                                  CONFIG_ZMK_NEMU_LINK_FULL_STATE_INTERVALS;
#endif
    int secure_err = compact ? nemu_link_secure_wrap_compact(peer, CENTRAL, message->source, bytes,
                                                             size, encrypted, &encrypted_size)
                             : nemu_link_secure_wrap(peer, CENTRAL, message->source, bytes, size,
                                                     encrypted, &encrypted_size);
    int err = secure_err ? secure_err : send_frame(message->source, encrypted, encrypted_size);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    if (!err && message->kind == NEMU_LINK_LINK_STATE &&
        (!compact || encrypted[1] == NEMU_LINK_WIRE_LINK)) {
        last_full_state_ms = now;
        full_state_sent = true;
    }
#endif
    if (!err) {
        nemu_link_mini_note(peer, true, message, mode, &notice);
    }
    return err;
}

int nemu_link_native_send(app_esb_data_t *packet) {
    int err = nemu_link_split_native_send(packet);
    if (err == -ENOTSUP) {
        LOG_WRN("unsupported native split event/command");
    }
    return err;
}

/* Called in ESB IRQ context. Copy bounded frames only; process on system work
 * queue alongside timeout and keymap/coordinator work. Never run ZMK here. */
void nemu_link_radio_event(app_esb_event_t *event) {
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
    if (event->evt_type == APP_ESB_EVT_TX_SUCCESS || event->evt_type == APP_ESB_EVT_TX_FAIL) {
        if (atomic_get(&settings_tx_wait_id) == event->msg_id) {
            atomic_set(&settings_tx_status, event->evt_type == APP_ESB_EVT_TX_SUCCESS ? 1 : 2);
            atomic_set(&settings_tx_done_id, event->msg_id);
            atomic_set(&settings_tx_wait_id, 0);
            k_sem_give(&settings_tx_done);
        }
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    if (event->evt_type == APP_ESB_EVT_TX_SUCCESS || event->evt_type == APP_ESB_EVT_TX_FAIL) {
        if (atomic_get(&tx_pending) > 0) {
            atomic_dec(&tx_pending);
        }
        /* Re-evaluate quiet time immediately after the completed transaction,
         * rather than holding HFCLK for the next 10ms periodic service. */
        nemu_link_trace_mark(NEMU_LINK_TRACE_COMPLETE, event->evt_type);
        nemu_link_profile_complete();
        k_work_reschedule(&link_work, K_NO_WAIT);
        return;
    }
#endif
    if (event->evt_type != APP_ESB_EVT_RX) {
        return;
    }
    const size_t minimum = NEMU_LINK_MINI_HEARTBEAT_SIZE;
    if (event->data_length < minimum ||
        event->data_length > sizeof(((struct rx_frame *)0)->bytes)) {
        bad_rx_record(event->pipe, BAD_RX_BAD_LENGTH);
        return;
    }
    if (event->pipe >= NEMU_LINK_LINK_SOURCES) {
        bad_rx_record(UINT8_MAX, BAD_RX_BAD_PIPE);
        return;
    }
    nemu_link_trace_mark(
        NEMU_LINK_TRACE_RX_IRQ,
        (uint16_t)((wire_kind(event->buf, event->data_length) << 8) | event->data_length));
    struct rx_frame frame = {.pipe = event->pipe,
                             .size = event->data_length,
                             .received_ms = k_uptime_get(),
                             .rssi_dbm = event->rssi_dbm};
    memcpy(frame.bytes, event->buf, event->data_length);
    if (k_msgq_put(&link_rx, &frame, K_NO_WAIT)) {
        atomic_inc(&rx_dropped);
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_RX_QUEUED, event->pipe);
    k_work_reschedule(&link_work, K_NO_WAIT);
}

#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static void positions(uint8_t source, struct nemu_link_link_keys keys, bool pressed) {
    /* STATE uses stock positions. ZMK behaviors/combo expect selected-layout
     * positions; keymap.c applies selected->stock when resolving bindings. */
    const uint32_t *map;
    int count = zmk_physical_layouts_get_selected_to_stock_position_map(&map);
    for (uint32_t position = 0; count > 0 && position < MIN((uint32_t)count, ZMK_KEYMAP_LEN);
         position++) {
        if (map[position] < ZMK_KEYMAP_LEN && nemu_link_link_keys_test(keys, map[position])) {
            int err = raise_zmk_position_state_changed(
                (struct zmk_position_state_changed){.source = source,
                                                    .position = position,
                                                    .state = pressed,
                                                    .timestamp = k_uptime_get()});
            if (err) {
                LOG_ERR("link position dispatch failed source=%u position=%u err=%d", source,
                        position, err);
            }
        }
    }
}

static void apply_result(uint8_t source, const struct nemu_link_link_result *out) {
    if (out->disconnected) {
        nemu_link_radio_discard_pipe(source);
        nemu_link_secure_reset(secure_peer(source));
        nemu_link_battery_disconnect(source);
        nemu_link_notify_source_lost(source);
        if (audio_callbacks && audio_callbacks->source_lost) {
            audio_callbacks->source_lost(source);
        }
        nemu_link_split_source_lost(source);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
        peer_selection[source] = (struct nemu_link_selection){0};
#endif
        requested_mode[source] = 0;
        confirmed_mode[source] = 0;
        LOG_WRN("link DOWN source=%u cancel=0x%x", source, (uint32_t)out->cancel_keys.words[0]);
    }
    /* An absent other hand does not block input. Only an actual selection
     * mismatch suppresses this source; keys held during convergence stay blocked. */
    if (!nemu_link_link_input_allowed(source)) {
        positions(source, nemu_link_link_keys_or(out->cancel_keys, out->release_keys), false);
        central.peers[source].blocked_keys =
            nemu_link_link_keys_or(central.peers[source].blocked_keys, central.peers[source].keys);
        central.peers[source].keys = (struct nemu_link_link_keys){0};
        return;
    }
    /* Like BLE disconnect, release through the ordinary ZMK event pipeline. */
    positions(source, nemu_link_link_keys_or(out->cancel_keys, out->release_keys), false);
    positions(source, out->press_keys, true);
    if (out->connected) {
        LOG_INF("link UP source=%u session=%u suppressed=0x%x", source,
                central.peers[source].session,
                (uint32_t)central.peers[source].blocked_keys.words[0]);
    }
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
static void selection_freeze(void) {
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
        nemu_link_battery_disconnect(source);
        nemu_link_notify_source_lost(source);
        positions(source, central.peers[source].keys, false);
        central.peers[source].blocked_keys =
            nemu_link_link_keys_or(central.peers[source].blocked_keys, central.peers[source].keys);
        central.peers[source].keys = (struct nemu_link_link_keys){0};
        nemu_link_split_source_lost(source);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    nemu_link_studio_link_state(false, NULL);
    nemu_link_studio_release_all();
#endif
    zmk_endpoint_clear_reports();
#if IS_ENABLED(CONFIG_ZMK_POINTING)
    zmk_endpoint_send_mouse_report();
#endif
}
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
static void next_dongle_work_handler(struct k_work *work);
static K_WORK_DEFINE(next_dongle_work, next_dongle_work_handler);
static void next_dongle_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    k_mutex_lock(&link_lock, K_FOREVER);
    uint8_t next = selected_dongle % CONFIG_ZMK_NEMU_LINK_DONGLE_COUNT + 1;
    if (!enabled || selection.target != selected_dongle ||
        !nemu_link_selection_next(&selection, next)) {
        k_mutex_unlock(&link_lock);
        return;
    }
    selection_freeze();
    LOG_INF("selection request generation=%u %u -> %u", selection.generation, selected_dongle,
            next);
    k_work_reschedule(&link_work, K_NO_WAIT);
    k_mutex_unlock(&link_lock);
}
#endif
int nemu_link_request_next_dongle(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    k_work_submit(&next_dongle_work);
    return 0;
#else
    return -ENOTSUP;
#endif
}
void nemu_link_request_ble_peers(void) {
    k_mutex_lock(&link_lock, K_FOREVER);
    ble_return_pending = true;
    ble_return_sources = (1U << NEMU_LINK_LINK_SOURCES) - 1U;
    memset(ble_return_attempts, 0, sizeof(ble_return_attempts));
    memset(requested_mode, 0, sizeof(requested_mode));
    memset(confirmed_mode, 0, sizeof(confirmed_mode));
    ble_return_deadline_ms = 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    LOG_INF("BLE return queued: Studio synced=%d busy=%d", nemu_link_studio_synced(),
            nemu_link_studio_radio_busy());
#endif
    k_work_reschedule(&link_work, K_NO_WAIT);
    k_mutex_unlock(&link_lock);
}
static void request_ble_peers(void) {
    if (!ble_return_sources) {
        ble_return_pending = false;
        return;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    if (!nemu_link_studio_synced() || nemu_link_studio_radio_busy()) {
        return;
    }
#endif
    uint64_t now = k_uptime_get();
    if (!ble_return_deadline_ms) {
        ble_return_deadline_ms = now + BLE_RETURN_TIMEOUT_MS;
    }
    if (now >= ble_return_deadline_ms) {
        LOG_WRN("BLE return timed out pending=0x%x", ble_return_sources);
        ble_return_sources = 0;
        ble_return_pending = false;
        ble_return_deadline_ms = 0;
        return;
    }
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
        if (!(ble_return_sources & BIT(source)) || requested_mode[source]) {
            continue;
        }
        if (central.peers[source].phase != NEMU_LINK_LINK_UP) {
            continue;
        }
        if (ble_return_attempts[source] >= BLE_RETURN_MAX_ATTEMPTS) {
            ble_return_sources &= ~BIT(source);
            LOG_WRN("BLE return retries exhausted source=%u", source);
            continue;
        }
        if (++mode_generation == 0) {
            ++mode_generation;
        }
        requested_mode[source] = mode_generation;
        ++ble_return_attempts[source];
        LOG_INF("link BLE request source=%u token=%u attempt=%u", source, requested_mode[source],
                ble_return_attempts[source]);
    }
    ble_return_pending = ble_return_sources != 0;
    if (!ble_return_pending) {
        ble_return_deadline_ms = 0;
    }
}
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
static void transmit_state(uint64_t now) {
    if (search_next(now) != now) {
        return;
    }
    struct nemu_link_link_keys output_keys = physical_keys;
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB)
    /* A standalone keyboard has its own USB endpoint. When it is selected,
     * keep the ESB session alive but prevent the dongle from duplicating HID. */
    if (zmk_endpoint_get_selected().transport == ZMK_TRANSPORT_USB) {
        output_keys = (struct nemu_link_link_keys){0};
    }
#endif
    uint32_t message_start = nemu_link_profile_start();
    struct nemu_link_link_message message;
    struct nemu_link_link_keys previous = peripheral.last_keys;
    if (!nemu_link_link_peripheral_next(&peripheral, now, output_keys, 0, &message)) {
        return;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_TRACE)
    if (peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP && idle.phase == NEMU_LINK_IDLE_OFF) {
        nemu_link_trace_begin(message.sequence, message_start);
        nemu_link_trace_mark(NEMU_LINK_TRACE_PROBE, 0);
        nemu_link_trace_mark(NEMU_LINK_TRACE_PROBE, 1);
    }
#else
    ARG_UNUSED(message_start);
#endif
    if (message.kind == NEMU_LINK_LINK_STATE) {
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
        message.battery = MIN(zmk_battery_state_of_charge(), 100);
#else
        message.battery = NEMU_LINK_LINK_BATTERY_UNKNOWN;
#endif
        if (peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP) {
            fault_seen_press = nemu_link_link_keys_or(
                fault_seen_press, nemu_link_link_keys_without(output_keys, previous));
        }
        if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_TEST_DROP_RELEASE_ONCE) && !release_dropped &&
            nemu_link_link_keys_any(nemu_link_link_keys_and(
                nemu_link_link_keys_without(previous, output_keys), fault_seen_press))) {
            release_dropped = true;
            LOG_WRN("link FAULT dropped release snapshot sequence=%u", message.sequence);
            return;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    nemu_link_trace_mark(NEMU_LINK_TRACE_MESSAGE_READY, message.kind);
    /* message retains this physical transition while bounded wake completes. */
    if (idle_wake()) {
        nemu_link_trace_finish(1);
        return;
    }
    idle.phase = NEMU_LINK_IDLE_AWAKE;
#endif
    int err =
        send_message(&message, message.kind == NEMU_LINK_LINK_STATE ? received_mode : 0, false);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    if (message.kind == NEMU_LINK_LINK_STATE &&
        (nemu_link_link_keys_any(message.keys) || nemu_link_link_keys_any(previous))) {
        LOG_INF("key state tx source=%u seq=%u keys=%llx/%llx previous=%llx/%llx err=%d",
                message.source, message.sequence, (unsigned long long)message.keys.words[0],
                (unsigned long long)message.keys.words[1], (unsigned long long)previous.words[0],
                (unsigned long long)previous.words[1], err);
    }
#else
    ARG_UNUSED(err);
    ARG_UNUSED(message);
#endif
}

void zmk_nemu_link_position(const struct zmk_position_state_changed *pos) {
    if (pos->source != ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL || pos->position >= ZMK_KEYMAP_LEN) {
        return;
    }
    const uint32_t *map;
    int count = zmk_physical_layouts_get_selected_to_stock_position_map(&map);
    if (count <= 0 || pos->position >= (uint32_t)count || map[pos->position] >= ZMK_KEYMAP_LEN) {
        return;
    }
    k_mutex_lock(&link_lock, K_FOREVER);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    bool was_pressed = nemu_link_link_keys_test(physical_keys, map[pos->position]);
#endif
    nemu_link_link_keys_set(&physical_keys, map[pos->position], pos->state);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    LOG_INF("key state local source=%u selected=%u stock=%u before=%u state=%u enabled=%u "
            "phase=%u",
            CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, pos->position, map[pos->position], was_pressed,
            pos->state, enabled, peripheral.phase);
#endif
    if (enabled) {
        search_input(k_uptime_get());
        /* Emit each physical transition without coalescing a quick press and
         * release into an empty bitmap before the periodic worker runs. */
        transmit_state(k_uptime_get());
    }
    k_mutex_unlock(&link_lock);
}
#endif

int nemu_link_link_enable(bool value) {
    k_mutex_lock(&link_lock, K_FOREVER);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ||                                                   \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    LOG_INF("link enable request value=%u current=%u ready=%lu pending=%u", value, enabled,
            atomic_get(&receiver_ready), start_pending);
#else
    LOG_INF("link enable request value=%u current=%u", value, enabled);
#endif
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    if (!enabled) {
        full_state_sent = false;
    }
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ||                                                   \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    if (value && !enabled && !atomic_get(&receiver_ready)) {
        start_pending = true;
        k_work_reschedule(&link_work, K_MSEC(100));
        k_mutex_unlock(&link_lock);
        return 0;
    }
    start_pending = false;
#endif
    uint32_t initial_nonce = 0;
    if (value && !enabled) {
        initial_nonce = fresh_nonce();
        if (!initial_nonce) {
            k_mutex_unlock(&link_lock);
            return -EIO;
        }
    }
    if (value == enabled) {
        k_mutex_unlock(&link_lock);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
        return 0; /* Logical enable is unchanged while radio sleeps. */
#else
        return nemu_link_radio_enable(value);
#endif
    }
    if (value && !enabled) {
        int security_err = nemu_link_secure_init();
        if (!security_err) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
            for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
                nemu_link_secure_reset(secure_peer(source));
            }
#else
            security_err = nemu_link_secure_start(secure_peer(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID),
                                                  initial_nonce);
#endif
        }
        if (security_err) {
            k_mutex_unlock(&link_lock);
            return security_err;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    if (value && !enabled) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        selected_dongle = CONFIG_ZMK_NEMU_LINK_DONGLE_ID;
#endif
        int profile_err = nemu_link_radio_profile(selected_dongle);
        if (profile_err) {
            k_mutex_unlock(&link_lock);
            return profile_err;
        }
    }
#endif
    int err = nemu_link_radio_enable(value);
    if (err) {
        k_mutex_unlock(&link_lock);
        return err;
    }
    enabled = value;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
        nemu_link_battery_disconnect(source);
    }
#endif
    if (!value) {
        for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
            nemu_link_secure_reset(&secure_peers[source]);
        }
    }
    nemu_link_split_reset();
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    if (!value) {
        nemu_link_studio_link_state(false, NULL);
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    nemu_link_idle_init(&idle, CONFIG_ZMK_NEMU_LINK_IDLE_GRACE_MS, WAKE_MARGIN_MS);
    atomic_set(&tx_pending, 0);
    retry_wake_ms = 0;
#endif
    k_msgq_purge(&link_rx);
    if (value) {
        uint64_t now = k_uptime_get();
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        const uint32_t buttons[] = {0, 0};
        nemu_link_link_central_init(&central, initial_nonce, NEMU_LINK_LINK_RELEASE_MS, buttons);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
        selected_dongle = CONFIG_ZMK_NEMU_LINK_DONGLE_ID;
        memset(peer_selection, 0, sizeof(peer_selection));
#endif
        nemu_link_hop_central_init(&hop, central.epoch, now, &hop_timing);
        next_hop_test = now + CONFIG_ZMK_NEMU_LINK_HOP_TEST_INTERVAL_MS;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
        nemu_link_channel_init(&channel_policy, now);
        adaptive_pending = false;
        scan_attempts = scan_busy = scan_errors = 0;
#endif
#else
        nemu_link_link_peripheral_init(&peripheral, CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID,
                                       initial_nonce, now, NEMU_LINK_LINK_INTERVAL_MS,
                                       NEMU_LINK_LINK_RELEASE_MS, 0);
        nemu_link_hop_peripheral_init(&hop, now, &hop_timing);
        search_active = false;
        peripheral.interval_ms = hop_timing.probe_ms;
        received_mode = 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_AUTO_BLE)
        dongle_confirmed_ms = now;
#endif
        release_dropped = false;
        fault_seen_press = (struct nemu_link_link_keys){0};
#endif
        next_stats = now + 3000;
        LOG_INF("link start role=%s heartbeat=%u release=%u battery=in-snapshot wire=%u",
                CENTRAL ? "central" : "peripheral", NEMU_LINK_LINK_INTERVAL_MS,
                NEMU_LINK_LINK_RELEASE_MS, NEMU_LINK_LINK_VERSION);
        k_work_reschedule(&link_work, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&link_work);
    }
    k_mutex_unlock(&link_lock);
    return 0;
}

uint8_t zmk_nemu_link_debug_state(void) {
    uint8_t state = enabled ? BIT(0) : 0;
    uint8_t radio_state = nemu_link_radio_debug_state();

#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ||                                                   \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    state |= atomic_get(&receiver_ready) ? BIT(1) : 0;
    state |= start_pending ? BIT(2) : 0;
#else
    state |= BIT(1); /* No USB startup gate is configured. */
#endif
    state |= (radio_state & BIT(0)) ? BIT(3) : 0;
    state |= (radio_state & BIT(1)) ? BIT(4) : 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    state |= ((uint8_t)peripheral.phase & 0x03) << 5;
#endif
    return state;
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
static void adaptive_service(uint64_t now) {
    bool both_up = central.peers[0].phase == NEMU_LINK_LINK_UP &&
                   central.peers[1].phase == NEMU_LINK_LINK_UP && nemu_link_link_input_allowed(0) &&
                   nemu_link_link_input_allowed(1);
    enum nemu_link_migration_result result =
        nemu_link_channel_result(&channel_policy, &channel_config, &hop, now, both_up);
    if (result != NEMU_LINK_MIGRATION_WAIT) {
        LOG_INF("adaptive result=%s rf=%u epoch=%u loss=%u/%u",
                result == NEMU_LINK_MIGRATION_OK ? "ok" : "failed",
                nemu_link_hop_channels[channel_policy.validation_index], hop.epoch,
                channel_policy.peers[0].loss_ewma, channel_policy.peers[1].loss_ewma);
    }
    /* Fully lost links use patrol during PREPARE. COMMIT must finish the
     * promised move; peers may already have enabled short loss detection. */
    bool lost =
        now - hop.seen_ms[0] >= hop.timing.loss_ms && now - hop.seen_ms[1] >= hop.timing.loss_ms;
    if (adaptive_pending && lost && !hop.committing) {
        hop.pending = false;
        adaptive_pending = false;
        LOG_INF("adaptive cancelled: both links lost");
    }
    bool required = requested_mode[0] || requested_mode[1];
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    required |= selection.target != selected_dongle;
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
        required |= central.peers[source].phase == NEMU_LINK_LINK_UP &&
                    !nemu_link_selection_equal(selection, peer_selection[source]);
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    required |= nemu_link_studio_radio_busy();
#endif
    uint8_t index;
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ADAPTIVE_MIGRATION) && !required &&
        nemu_link_channel_candidate(&channel_policy, &channel_config, &hop, now, both_up, &index) &&
        nemu_link_hop_central_request_index(&hop, now, index)) {
        adaptive_pending = true;
        LOG_INF("adaptive PREPARE old=%u new=%u epoch=%u busy=%u loss=%u/%u",
                nemu_link_hop_channels[hop.index], nemu_link_hop_channels[index], hop.epoch + 1,
                channel_policy.channels[index].busy_ewma, channel_policy.peers[0].loss_ewma,
                channel_policy.peers[1].loss_ewma);
    }
    if (!required &&
        nemu_link_channel_scan_due(&channel_policy, &channel_config, &hop, now, &index)) {
        uint16_t busy, samples;
        unsigned int key = irq_lock();
        int err =
            k_msgq_num_used_get(&link_rx) ? -EBUSY : nemu_link_radio_scan(index, &busy, &samples);
        irq_unlock(key);
        scan_attempts++;
        if (!err) {
            nemu_link_channel_scan_record(&channel_policy, index, now, busy, samples);
        } else if (err == -EBUSY) {
            scan_busy++;
        } else {
            scan_errors++;
            LOG_WRN("channel scan failed err=%d", err);
        }
    }
}
#endif

static bool hop_channel_change_available(void) { return nemu_link_hop_channel_count > 1U; }

static void hop_service(uint64_t now) {
    /* Searching the only configured channel cannot find an alternative. Keep
     * the receiver in place instead of advancing the hop epoch and purging RX. */
    bool channel_hop_available = hop_channel_change_available();
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (CONFIG_ZMK_NEMU_LINK_HOP_TEST_INTERVAL_MS && now >= next_hop_test &&
        central.peers[0].phase == NEMU_LINK_LINK_UP &&
        central.peers[1].phase == NEMU_LINK_LINK_UP && nemu_link_hop_central_request(&hop, now)) {
        next_hop_test = now + CONFIG_ZMK_NEMU_LINK_HOP_TEST_INTERVAL_MS;
        LOG_INF("hop PREPARE epoch=%u next=%u", hop.epoch + 1,
                (hop.index + 1) % nemu_link_hop_channel_count);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
    adaptive_service(now);
#endif
    struct nemu_link_hop_notice target;
    bool change = channel_hop_available && nemu_link_hop_central_next(&hop, now, &target);
    uint8_t index = change ? target.index : hop.index;
#else
    uint8_t index;
    bool change = channel_hop_available && nemu_link_hop_peripheral_next(&hop, now, &index);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    if (change && idle_wake()) {
        return;
    }
#endif
#endif
    unsigned int key = irq_lock();
    /* Never discard a queued valid input just to change RF. RX is drained in
     * bounded batches; independent link expiry runs before every batch. */
    int err = 0;
    if (change && !k_msgq_num_used_get(&link_rx)) {
        err = nemu_link_radio_channel(index);
        if (!err) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
            nemu_link_hop_central_applied(&hop, now, &target);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
            nemu_link_channel_changed(&channel_policy);
            if (adaptive_pending) {
                nemu_link_channel_validate(&channel_policy, &channel_config, hop.index, now);
                adaptive_pending = false;
            }
#endif
#else
            nemu_link_split_discard_input();
            nemu_link_hop_peripheral_applied(&hop, now, index);
            peripheral.has_sent = false; /* Immediate full snapshot on new RF. */
#endif
            k_msgq_purge(&link_rx);
            LOG_INF("hop channel epoch=%u index=%u rf=%u search=%u", hop.epoch, hop.index,
                    nemu_link_hop_channels[hop.index], hop.searching);
        }
    }
    nemu_link_radio_service();
    irq_unlock(key);
    if (err && err != -EBUSY) {
        LOG_WRN("hop retune failed err=%d", err);
    }
}

/* Authentication is the only path into application decoders. */
static bool authenticate_rx_frame(struct rx_frame *frame, uint64_t now) {
    if (now - frame->received_ms >= NEMU_LINK_LINK_RELEASE_MS) {
        bad_rx_record(frame->pipe, BAD_RX_QUEUE_STALE);
        return false;
    }
    struct nemu_link_secure_peer *security = secure_peer(frame->pipe);
    if (frame->bytes[0] == NEMU_LINK_SECURE_WIRE_VERSION && frame->size >= 2 &&
        frame->bytes[1] >= 1 && frame->bytes[1] <= 4) {
        uint8_t reply[NEMU_LINK_FRAME_MAX_SIZE];
        size_t reply_size = 0;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        bool allow_new = central.peers[frame->pipe].phase != NEMU_LINK_LINK_UP;
        uint8_t previous_client_random[NEMU_LINK_AUTH_RANDOM_SIZE];
        memcpy(previous_client_random, security->client_random, sizeof(previous_client_random));
#else
        bool allow_new = false;
        enum nemu_link_secure_phase previous_auth = security->phase;
#endif
        int auth_err = nemu_link_secure_auth_receive(security, CENTRAL, allow_new, frame->pipe,
                                                     secure_dongle_id(), frame->bytes, frame->size,
                                                     reply, &reply_size);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        peer_diag[frame->pipe].auth_rx++;
        peer_diag[frame->pipe].last_wire_kind = frame->bytes[1];
        peer_diag[frame->pipe].last_auth_error = auth_err;
        peer_diag[frame->pipe].auth_rejected += auth_err != 0;
        peer_diag[frame->pipe].auth_detail = nemu_link_secure_auth_diagnostic();
        if (peer_diag[frame->pipe].auth_rx <= 4) {
            LOG_INF("auth detail v2 source=%u wire=%u phase_before=%u challenge_matches=%u "
                    "mac_checked=%u mac_status=%d key_fp=%08x err=%d",
                    frame->pipe, frame->bytes[1], peer_diag[frame->pipe].auth_detail.phase_before,
                    peer_diag[frame->pipe].auth_detail.challenge_matches,
                    peer_diag[frame->pipe].auth_detail.mac_checked,
                    peer_diag[frame->pipe].auth_detail.mac_status,
                    peer_diag[frame->pipe].auth_detail.loaded_key_fingerprint, auth_err);
        }
#endif
        if (auth_err) {
            bad_rx_record(frame->pipe, BAD_RX_AUTH);
        } else {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
            /* A valid new HELLO may replace a syncing (not UP) peer. Old
             * encrypted replies must not leak into the new key session. */
            if (frame->bytes[NEMU_LINK_AUTH_KIND_OFFSET] == 1 &&
                memcmp(previous_client_random, security->client_random,
                       sizeof(previous_client_random))) {
                nemu_link_radio_discard_pipe(frame->pipe);
            }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
            /* A reboot can spend its entire fast-search period waiting for
             * the old lease. Once authenticated discovery makes progress,
             * keep probing long enough to drain delayed ACKs and finish the
             * application handshake. Duplicate/rejected replies do not renew. */
            if (security->phase != previous_auth) {
                search_update(now);
                if (search_active) {
                    search_started_ms = now;
                }
            }
#endif
            if (reply_size) {
                send_frame(frame->pipe, reply, reply_size);
            }
        }
        return false;
    }
    uint32_t route_epoch = 0, route_session = 0;
    uint32_t route_nonce = sys_get_le32(security->client_random);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (central.peers[frame->pipe].phase == NEMU_LINK_LINK_UP) {
        route_epoch = central.epoch;
        route_session = central.peers[frame->pipe].session;
    }
#else
    if (peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP) {
        route_epoch = peripheral.epoch;
        route_session = peripheral.session;
    }
#endif
    uint8_t authenticated[NEMU_LINK_FRAME_MAX_SIZE];
    size_t authenticated_size = 0;
    int secure_err = nemu_link_secure_unwrap(security, CENTRAL, frame->pipe, frame->bytes,
                                             frame->size, route_epoch, route_session, route_nonce,
                                             authenticated, &authenticated_size);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    peer_diag[frame->pipe].data_rx++;
    peer_diag[frame->pipe].last_wire_kind = frame->bytes[1];
    peer_diag[frame->pipe].last_data_error = secure_err;
    peer_diag[frame->pipe].data_rejected += secure_err != 0;
    peer_diag[frame->pipe].accepted += secure_err == 0;
#endif
    if (secure_err) {
        bad_rx_record(frame->pipe, BAD_RX_SECURE);
        return false;
    }
    memcpy(frame->bytes, authenticated, authenticated_size);
    frame->size = authenticated_size;
    return true;
}

/* Frames are authenticated before split, Studio or link state sees their bytes. */
static void process_rx_frame(struct rx_frame *frame) {
    struct nemu_link_link_message message;
    uint32_t mode;
    uint64_t now = k_uptime_get();
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (frame->pipe < NEMU_LINK_LINK_SOURCES) {
        atomic_set(&diag_rx_rssi_last_dbm[frame->pipe], frame->rssi_dbm);
        atomic_set(&diag_rx_rssi_valid[frame->pipe], 1);
    }
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    struct rx_rssi_stats *rssi = &diag_rx_rssi[frame->pipe];
    if (!rssi->samples) {
        rssi->min_dbm = frame->rssi_dbm;
        rssi->max_dbm = frame->rssi_dbm;
    } else {
        rssi->min_dbm = MIN(rssi->min_dbm, frame->rssi_dbm);
        rssi->max_dbm = MAX(rssi->max_dbm, frame->rssi_dbm);
    }
    rssi->have_last = true;
    rssi->last_dbm = frame->rssi_dbm;
    if (rssi->have_avg) {
        rssi->avg_dbm_x8 = (7 * rssi->avg_dbm_x8 + 8 * frame->rssi_dbm) / 8;
    } else {
        rssi->avg_dbm_x8 = 8 * frame->rssi_dbm;
        rssi->have_avg = true;
    }
    rssi->samples++;
#endif
    struct nemu_link_hop_notice notice;
    bool direct = false;
    bool mini_request = nemu_link_mini_wire(frame->size);
    if (mini_request) {
        if (frame->pipe >= NEMU_LINK_LINK_SOURCES) {
            bad_rx_record(UINT8_MAX, BAD_RX_BAD_PIPE);
            return;
        }
        struct nemu_link_secure_peer *peer = secure_peer(frame->pipe);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        uint32_t epoch = central.epoch, session = central.peers[frame->pipe].session;
        uint32_t reference = 0;
#else
        uint32_t epoch = peripheral.epoch, session = peripheral.session;
        uint32_t reference = peripheral.sequence;
#endif
        int err = nemu_link_mini_unwrap(peer, CENTRAL, frame->pipe, frame->bytes, frame->size,
                                        epoch, session, sys_get_le32(peer->client_random),
                                        reference, &message, &mode, &notice);
        if (err) {
            bad_rx_record(frame->pipe, BAD_RX_MINI);
            return;
        }
        direct = true;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    /* A compact request proves the peripheral already learned its link session.
     * Full requests must still receive full ACKs if the first ACK was lost. */
    bool compact_request =
        frame->size >= NEMU_LINK_COMPACT_MIN_SIZE &&
        frame->bytes[0] == NEMU_LINK_SECURE_WIRE_VERSION &&
        (frame->bytes[1] == NEMU_LINK_WIRE_EMPTY || frame->bytes[1] == NEMU_LINK_WIRE_STATE);
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    compact_request |= mini_request;
#endif
    if (!direct && !authenticate_rx_frame(frame, now)) {
        return;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC) ||                                                \
    (IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL))
    if (!direct && frame->bytes[6] == NEMU_LINK_WIRE_STUDIO) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
        if (frame->pipe == CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE &&
            nemu_link_link_input_allowed(frame->pipe)) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
            nemu_link_channel_data(&channel_policy, frame->pipe, frame->received_ms,
                                   central.peers[frame->pipe].session);
#endif
            nemu_link_studio_receive(frame->bytes, frame->size, frame->received_ms);
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (frame->pipe < NEMU_LINK_LINK_SOURCES && frame->size >= 9 &&
            frame->bytes[6] == NEMU_LINK_STUDIO_FRAME_TAG &&
            frame->bytes[8] == NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE) {
            struct nemu_link_studio_route route;
            struct nemu_link_studio_packet packet;
            bool route_ok = nemu_link_studio_route_central_source(&route, &central, &hop,
                                                                  frame->pipe, frame->received_ms);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
            if (route_ok) {
                route.dongle_id = selected_dongle;
            }
#endif
            if (!route_ok || !nemu_link_link_input_allowed(frame->pipe)) {
                nemu_link_settings_esb_note_link_rx_drop(true,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_ROUTE);
            } else if (now < frame->received_ms ||
                       now - frame->received_ms >= NEMU_LINK_LINK_RELEASE_MS) {
                nemu_link_settings_esb_note_link_rx_drop(true,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_EXPIRED);
            } else if (!nemu_link_studio_frame_decode(&packet, &route, frame->pipe, frame->bytes,
                                                      frame->size, FRAME_CRC)) {
                nemu_link_settings_esb_note_link_rx_drop(true,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_DECODE);
            } else {
                (void)nemu_link_settings_esb_receive_response(&packet, frame->pipe);
            }
        }
#endif
#else
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
        /* The dongle's shared-settings Studio packets (base, request, and
         * response) use the same authenticated link envelope as owner RPC.
         * Forward them into the Studio coordinator on the keyboard side so
         * its shared-state checks and response path can run. Owner RPC has a
         * separate relay and must remain on that path below. */
        if (frame->pipe == CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID && frame->size >= 9 &&
            frame->bytes[6] == NEMU_LINK_STUDIO_FRAME_TAG &&
            frame->bytes[8] != NEMU_LINK_STUDIO_OWNER_RPC_REQUEST &&
            nemu_link_link_input_allowed(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID)) {
            nemu_link_studio_receive(frame->bytes, frame->size, frame->received_ms);
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        if (frame->pipe == CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID && frame->size >= 9 &&
            frame->bytes[6] == NEMU_LINK_STUDIO_FRAME_TAG &&
            frame->bytes[8] == NEMU_LINK_STUDIO_OWNER_RPC_REQUEST) {
            struct nemu_link_studio_route route;
            struct nemu_link_studio_packet packet;
            bool route_ok = nemu_link_studio_route_peripheral_source(
                &route, &peripheral, &hop, CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, frame->received_ms);
            if (!route_ok || !nemu_link_link_input_allowed(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID)) {
                nemu_link_settings_esb_note_link_rx_drop(false,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_ROUTE);
            } else if (now < frame->received_ms ||
                       now - frame->received_ms >= NEMU_LINK_LINK_RELEASE_MS) {
                nemu_link_settings_esb_note_link_rx_drop(false,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_EXPIRED);
            } else if (!nemu_link_studio_frame_decode(&packet, &route, frame->pipe, frame->bytes,
                                                      frame->size, FRAME_CRC)) {
                nemu_link_settings_esb_note_link_rx_drop(false,
                                                         NEMU_LINK_SETTINGS_ESB_RX_DROP_DECODE);
            } else if (packet.kind == NEMU_LINK_STUDIO_OWNER_RPC_REQUEST) {
                (void)nemu_link_settings_esb_receive_request(&packet, &route);
            }
        }
#endif
#endif
        return;
    }
#endif
    if (!direct && frame->size >= NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE &&
        !memcmp(frame->bytes, "EsAu", 4) && frame->bytes[6] == NEMU_LINK_WIRE_AUDIO &&
        frame->bytes[4] == frame->pipe && frame->bytes[10] > 0 &&
        frame->bytes[9] < frame->bytes[10] &&
        frame->bytes[11] == frame->size - NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE &&
        frame->bytes[11] <= NEMU_LINK_AUDIO_DATA_MAX) {
        if (audio_callbacks && audio_callbacks->fragment) {
            audio_callbacks->fragment(
                frame->pipe, sys_get_le16(frame->bytes + 7), frame->bytes[9], frame->bytes[10],
                frame->bytes + NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE, frame->bytes[11]);
        }
        return;
    }
    if (!direct && frame->size == NEMU_LINK_NATIVE_AUDIO_CONTROL_SIZE &&
        !memcmp(frame->bytes, "EsAc", 4) && frame->bytes[6] == NEMU_LINK_WIRE_AUDIO_CONTROL &&
        frame->bytes[4] == frame->pipe && frame->bytes[7] <= 1U) {
        if (audio_callbacks && audio_callbacks->control) {
            audio_callbacks->control(frame->bytes[5], frame->bytes[7] != 0);
        }
        return;
    }
    if (!direct && !memcmp(frame->bytes, "ErEv", 4)) {
        int err = (now - frame->received_ms >= NEMU_LINK_LINK_RELEASE_MS)
                      ? -EINVAL
                      : nemu_link_split_receive(frame);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
        if (!err) {
            nemu_link_channel_data(&channel_policy, frame->pipe, frame->received_ms,
                                   central.peers[frame->pipe].session);
        }
#endif
        if (err == -ENOTSUP) {
            LOG_WRN("unsupported received split event/command source=%u", frame->pipe);
        }
        if (err) {
            bad_rx_record(frame->pipe, BAD_RX_SPLIT);
        }
        return;
    }
    bool frame_valid = direct;
    if (!direct) {
        nemu_link_trace_mark(NEMU_LINK_TRACE_LINK_DECODE_BEGIN, frame->size);
        frame_valid = nemu_link_hop_frame_decode(&message, &mode, &notice, frame->pipe,
                                                 frame->bytes, frame->size, FRAME_CRC);
        nemu_link_trace_mark(NEMU_LINK_TRACE_LINK_DECODE_END, frame_valid);
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    if (frame_valid) {
        if (message.kind <= NEMU_LINK_LINK_ACK) {
            diag_kind[message.kind]++;
        }
    } else {
        diag_decode_rejected++;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    if (frame_valid && notice.dongle_id != selected_dongle) {
        diag_notice_rejected++;
    }
#endif
    bool notice_valid = !frame_valid || notice.dongle_id == selected_dongle;
#else
    bool notice_valid = true;
#endif
    if (now - frame->received_ms >= NEMU_LINK_LINK_RELEASE_MS) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        diag_stale++;
#endif
        bad_rx_record(frame->pipe, BAD_RX_STALE);
        return;
    }
    if (!frame_valid) {
        bad_rx_record(frame->pipe, BAD_RX_HOP_FRAME);
        return;
    }
    if (!notice_valid) {
        bad_rx_record(frame->pipe, BAD_RX_NOTICE);
        return;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    struct nemu_link_link_result out;
    uint8_t source = message.source;
    bool committing = hop.committing;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    if (message.kind == NEMU_LINK_LINK_HELLO || message.kind == NEMU_LINK_LINK_STATE) {
        struct nemu_link_hop_central hop_candidate = hop;
        bool hop_accept = nemu_link_hop_central_receive(&hop_candidate, source, now, &notice);
        struct nemu_link_link_central link_candidate = central;
        bool link_accept =
            hop_accept && nemu_link_link_central_receive(&link_candidate, source, &message, now,
                                                         &(struct nemu_link_link_result){0});
        if (!hop_accept) {
            diag_hop_rejected++;
            diag_hop_rejected_source[frame->pipe]++;
        } else if (!link_accept) {
            diag_link_rejected++;
            diag_link_rejected_source[frame->pipe]++;
        } else {
            diag_link_fresh++;
            diag_link_fresh_source[frame->pipe]++;
        }
    }
#endif
    bool fresh =
        nemu_link_hop_link_central_receive(&central, &hop, source, &message, &notice, now, &out);
    if (fresh && !committing && hop.committing) {
        LOG_INF("hop COMMIT epoch=%u index=%u rf=%u receipts=%u window_ms=%u", hop.epoch + 1,
                hop.pending_index, nemu_link_hop_channels[hop.pending_index], hop.receipt_mask,
                NEMU_LINK_HOP_COMMIT_PROBES * hop.timing.probe_ms);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    struct nemu_link_selection remote;
    if (fresh && message.kind == NEMU_LINK_LINK_STATE &&
        nemu_link_selection_decode(mode, &remote)) {
        peer_selection[source] = remote;
        if (nemu_link_selection_merge(&selection, remote)) {
            selection_freeze();
            out.press_keys = (struct nemu_link_link_keys){0};
            LOG_INF("selection learned generation=%u target=%u source=%u", selection.generation,
                    selection.target, source);
        }
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
    if (fresh && message.kind == NEMU_LINK_LINK_STATE) {
        nemu_link_channel_state(&channel_policy, source, frame->received_ms, message.session,
                                secure_peer(source)->rx_sequence);
    }
#endif
    apply_result(source, &out);
    if (nemu_link_link_keys_any(out.press_keys) || nemu_link_link_keys_any(out.release_keys)) {
        LOG_INF("link STATE source=%u seq=%u press=0x%x release=0x%x", source, message.sequence,
                (uint32_t)out.press_keys.words[0], (uint32_t)out.release_keys.words[0]);
    }
    if (fresh && message.kind == NEMU_LINK_LINK_STATE) {
        if (mode && mode == requested_mode[source] && mode != confirmed_mode[source]) {
            confirmed_mode[source] = mode;
            ble_return_sources &= ~BIT(source);
            ble_return_pending = ble_return_sources != 0;
            if (!ble_return_pending) {
                ble_return_deadline_ms = 0;
            }
            LOG_INF("link BLE received source=%u token=%u", source, mode);
        }
        if (message.battery != NEMU_LINK_LINK_BATTERY_UNKNOWN) {
            nemu_link_battery_update(source, message.battery, nemu_link_link_input_allowed(source));
        }
        if (message.battery != NEMU_LINK_LINK_BATTERY_UNKNOWN &&
            message.battery != battery[source]) {
            battery[source] = message.battery;
            struct zmk_split_transport_peripheral_event event = {
                .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                .data.battery_event.level = message.battery};
            zmk_split_transport_central_peripheral_event_handler(active_transport, source, event);
            LOG_INF("link battery source=%u percent=%u", source, message.battery);
        }
    }
    if (fresh && !direct) {
        nemu_link_mini_note(secure_peer(source), false, &message, mode, &notice);
    }
    if (out.has_reply) {
        bool force_full = out.connected;
        force_full |= !compact_request;
        send_message(&out.reply, out.reply.kind == NEMU_LINK_LINK_ACK ? requested_mode[source] : 0,
                     force_full);
    }
    if (out.connected) {
        /* Queue after the handshake ACK, so the peer can accept commands. */
        /* Initialization may have selected a layout before this peer joined. */
        int layout = zmk_physical_layouts_get_selected();
        if (layout >= 0 && active_transport && active_transport->api->send_command) {
            struct zmk_split_transport_central_command command = {
                .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT,
                .data.set_physical_layout.layout_idx = layout};
            active_transport->api->send_command(source, command);
        }
    }
#else
    enum nemu_link_link_peripheral_phase previous = peripheral.phase;
    bool committed = hop.committed;
    nemu_link_trace_mark(NEMU_LINK_TRACE_ACK_APPLY_BEGIN, message.kind);
    bool fresh = nemu_link_hop_link_peripheral_receive(&peripheral, &hop, &message, &notice, now);
    nemu_link_trace_mark(NEMU_LINK_TRACE_ACK_APPLY_END, fresh);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    if (fresh && message.kind == NEMU_LINK_LINK_ACK && message.studio_pending) {
        nemu_link_studio_notify();
    }
#endif
    if (fresh && !committed && hop.committed) {
        LOG_INF("hop COMMIT received epoch=%u index=%u follow_ms=%u", notice.epoch, notice.index,
                NEMU_LINK_HOP_FOLLOW_PROBES * hop.timing.probe_ms);
    }
    if (fresh && !direct) {
        nemu_link_mini_note(secure_peer(message.source), false, &message, mode, &notice);
    }
    if (fresh && message.kind == NEMU_LINK_LINK_CHALLENGE) {
        search_update(now);
        if (search_active) {
            search_started_ms = now;
        }
    }
    if (fresh && peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_UP && previous != peripheral.phase) {
        LOG_INF("link UP source=%u session=%u", message.source, peripheral.session);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_AUTO_BLE)
    if (fresh && message.kind == NEMU_LINK_LINK_ACK) {
        dongle_confirmed_ms = peripheral.confirmed_ms;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    struct nemu_link_selection remote;
    if (fresh && message.kind == NEMU_LINK_LINK_ACK && nemu_link_selection_decode(mode, &remote)) {
        /* Diagnostic: deliberately miss one switch for a bounded interval. */
        if (remote.target != selected_dongle && CONFIG_ZMK_NEMU_LINK_TEST_SELECTION_IGNORE_MS &&
            !selection_fault_started) {
            selection_fault_started = true;
            selection_ignore_until = now + CONFIG_ZMK_NEMU_LINK_TEST_SELECTION_IGNORE_MS;
            LOG_WRN("selection FAULT ignoring instructions until %llu",
                    (unsigned long long)selection_ignore_until);
        }
        if (now >= selection_ignore_until) {
            nemu_link_selection_merge(&selection, remote);
            if (selection.target != selected_dongle) {
                profile_target = selection.target;
                k_work_submit(&profile_work);
            }
        }
        return;
    }
#endif
    if (fresh && message.kind == NEMU_LINK_LINK_ACK && mode && mode != received_mode) {
        received_mode = mode;
        LOG_INF("link BLE command token=%u", mode);
        zmk_nemu_link_request_mode(false);
    }
#endif
}

static void service(struct k_work *work) {
    ARG_UNUSED(work);
    nemu_link_trace_mark(NEMU_LINK_TRACE_WORK_ENTRY, 0);
    k_mutex_lock(&link_lock, K_FOREVER);
    nemu_link_trace_mark(NEMU_LINK_TRACE_WORK_LOCKED, 0);
    nemu_link_profile_service();
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ||                                                   \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    if (start_pending && !enabled) {
        k_mutex_unlock(&link_lock);
        int err = nemu_link_link_enable(true);
        if (err) {
            LOG_WRN("hop USB-ready start failed err=%d", err);
            k_mutex_lock(&link_lock, K_FOREVER);
            start_pending = true;
            k_work_reschedule(&link_work, K_MSEC(100));
            k_mutex_unlock(&link_lock);
        }
        return;
    }
#endif
    if (!enabled) {
        nemu_link_trace_finish(3);
        k_mutex_unlock(&link_lock);
        return;
    }
    uint64_t now = k_uptime_get();
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    request_ble_peers();
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
        struct nemu_link_link_result out;
        nemu_link_link_central_tick(&central, source, now, &out);
        apply_result(source, &out);
    }
#endif
    struct rx_frame frame;
    /* Bounded batch so sustained RX cannot starve the deadline service. */
    for (unsigned int i = 0; i < 8 && !k_msgq_get(&link_rx, &frame, K_NO_WAIT); i++) {
        nemu_link_trace_mark(NEMU_LINK_TRACE_RX_BEGIN, frame.size);
        process_rx_frame(&frame);
        nemu_link_trace_mark(NEMU_LINK_TRACE_RX_END, frame.size);
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    hop_service(k_uptime_get());
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    nemu_link_trace_mark(NEMU_LINK_TRACE_MAINT_BEGIN, 0);
    now = k_uptime_get();
    if (auto_ble_due(now)) {
        LOG_WRN("dongle ACK timeout: return to BLE with last saved local Studio settings");
        /* The mode queue stops this link. No new lease from retries or RF ACKs. */
        nemu_link_recover_ble();
        nemu_link_trace_finish(2);
        k_mutex_unlock(&link_lock);
        k_work_reschedule(&link_work, K_MSEC(100));
        return;
    }
    nemu_link_link_peripheral_tick(&peripheral, now);
    if (peripheral.phase == NEMU_LINK_LINK_PERIPHERAL_LOST) {
        if (audio_callbacks && audio_callbacks->control) {
            audio_callbacks->control(UINT8_MAX, false);
        }
        LOG_WRN("link ACK timeout source=%u; starting new HELLO",
                CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID);
        nemu_link_split_discard_input();
        uint32_t restart_nonce = fresh_nonce();
        if (restart_nonce) {
            if (!nemu_link_secure_start(secure_peer(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID),
                                        restart_nonce)) {
                nemu_link_link_peripheral_restart(&peripheral, restart_nonce, now);
            }
        }
        received_mode = 0;
    }
    search_update(now);
    if (search_next(now) == now) {
        hop_service(now);
    }
    peripheral.interval_ms =
        hop.searching || hop.pending || peripheral.phase != NEMU_LINK_LINK_PERIPHERAL_UP
            ? hop_timing.probe_ms
            : NEMU_LINK_LINK_INTERVAL_MS;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    uint64_t deadline = next_tx_ms(now);
    bool must_wake = received_mode != 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    must_wake |= nemu_link_studio_radio_busy();
#endif
    must_wake |= !nemu_link_hop_idle_allowed(&hop, now);
    /* With zero lead, transmit_state prepares the due snapshot before waking.
     * Keep its heartbeat timestamp independent of the clock startup duration. */
    if ((must_wake || WAKE_MARGIN_MS) &&
        nemu_link_idle_should_wake(&idle, now, deadline, must_wake)) {
        idle_wake();
    }
#endif
    now = k_uptime_get();
    transmit_state(now);
    nemu_link_trace_mark(NEMU_LINK_TRACE_MAINT_END, 0);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    now = k_uptime_get();
    deadline = next_tx_ms(now);
    /* Application ACK remains outstanding across sleep, exactly as in link.
     * PTX can only receive it in the NEXT ordinary transaction. Keep the link
     * ledger/deadlines running; never add a transmission just to enter idle. */
    unsigned int irq_key = irq_lock();
    bool busy = received_mode || atomic_get(&tx_pending) || k_msgq_num_used_get(&link_rx);
    busy |= !nemu_link_hop_idle_allowed(&hop, now);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    busy |= nemu_link_studio_radio_busy();
#endif
    if (nemu_link_idle_should_sleep(&idle, now, deadline, busy)) {
        int err = nemu_link_radio_sleep();
        if (!err) {
            idle.phase = NEMU_LINK_IDLE_OFF;
            idle_off_since = k_uptime_get();
            idle_sleeps++;
        } else if (err != -EBUSY) {
            /* Shutdown/release failed. Retry the radio at the next send even
             * if restoring the awake radio also failed; do not claim sleep. */
            idle.phase = NEMU_LINK_IDLE_OFF;
            idle_off_since = k_uptime_get();
            idle_errors++;
        } else {
            idle.phase = NEMU_LINK_IDLE_AWAKE;
        }
    }
    irq_unlock(irq_key);
#endif
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
    struct nemu_link_studio_route route;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    bool studio_route = nemu_link_studio_route_central(&route, &central, &hop, k_uptime_get());
#else
    bool studio_route =
        nemu_link_studio_route_peripheral(&route, &peripheral, &hop, k_uptime_get());
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    studio_route &= nemu_link_link_input_allowed(CONFIG_ZMK_NEMU_LINK_STUDIO_SOURCE);
    if (studio_route) {
        route.dongle_id = selected_dongle;
    }
#endif
    nemu_link_studio_link_state(studio_route, studio_route ? &route : NULL);
#endif
    if (now >= next_stats) {
        nemu_link_trace_mark(NEMU_LINK_TRACE_DIAG_BEGIN, 0);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
        LOG_INF("selection status generation=%u target=%u receiver=%u", selection.generation,
                selection.target, selected_dongle);
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        bool commit = hop.committing;
#else
        bool commit = hop.committed;
#endif
        LOG_INF("hop status boot=%u epoch=%u index=%u search=%u pending=%u commit=%u", hop.boot,
                hop.epoch, hop.index, hop.searching, hop.pending, commit);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        LOG_INF("hop receipts mask=%u", hop.receipt_mask);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_CHANNEL_SCAN)
        LOG_INF("scan attempts=%u busy=%u errors=%u loss=%u/%u samples=%u/%u validate=%u",
                scan_attempts, scan_busy, scan_errors, channel_policy.peers[0].loss_ewma,
                channel_policy.peers[1].loss_ewma, channel_policy.peers[0].samples,
                channel_policy.peers[1].samples, channel_policy.validating);
        for (unsigned int i = 0; i < nemu_link_hop_channel_count; i++) {
            const struct nemu_link_channel_scan *q = &channel_policy.channels[i];
            LOG_INF("channel rf=%u windows=%u busy=%u last=%llu penalty=%u until=%llu",
                    nemu_link_hop_channels[i], q->scan_windows, q->busy_ewma,
                    (unsigned long long)q->last_scan_ms,
                    now < q->penalty_expiry ? q->failure_penalty : 0,
                    (unsigned long long)q->penalty_expiry);
        }
#endif
#endif
        LOG_INF("link stats bad_rx=%ld rx_full=%ld tx_error=%u", (long)atomic_get(&bad_rx),
                (long)atomic_get(&rx_dropped), tx_errors);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        LOG_INF("link receive diag decode_rejected=%u notice_rejected=%u stale=%u "
                "kind_hello=%u kind_challenge=%u kind_state=%u kind_ack=%u "
                "hop_rejected=%u link_rejected=%u link_fresh=%u",
                diag_decode_rejected, diag_notice_rejected, diag_stale, diag_kind[1], diag_kind[2],
                diag_kind[3], diag_kind[4], diag_hop_rejected, diag_link_rejected, diag_link_fresh);
        for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
            LOG_INF("rx reject source=%u len=%u queue_stale=%u auth=%u secure=%u "
                    "mini=%u split=%u frame=%u stale=%u notice=%u hop_state=%u link_state=%u "
                    "fresh_state=%u",
                    source, (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_BAD_LENGTH]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_QUEUE_STALE]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_AUTH]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_SECURE]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_MINI]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_SPLIT]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_HOP_FRAME]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_STALE]),
                    (uint32_t)atomic_get(&bad_rx_source[source][BAD_RX_NOTICE]),
                    diag_hop_rejected_source[source], diag_link_rejected_source[source],
                    diag_link_fresh_source[source]);
        }
        LOG_INF("rx reject global len=%u pipe=%u",
                (uint32_t)atomic_get(&bad_rx_global[BAD_RX_BAD_LENGTH]),
                (uint32_t)atomic_get(&bad_rx_global[BAD_RX_BAD_PIPE]));
        for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
            const struct rx_rssi_stats *rssi = &diag_rx_rssi[source];
            if (rssi->samples) {
                LOG_INF("rx rssi source=%u samples=%u last_dbm=%d min_dbm=%d avg_dbm=%d "
                        "max_dbm=%d",
                        source, rssi->samples, rssi->last_dbm, rssi->min_dbm,
                        (int)((rssi->avg_dbm_x8 - 4) / 8), rssi->max_dbm);
            } else if (rssi->have_last) {
                LOG_INF("rx rssi source=%u samples=0 last_dbm=%d", source, rssi->last_dbm);
            } else {
                LOG_INF("rx rssi source=%u samples=0 last_dbm=na", source);
            }
            diag_rx_rssi[source].samples = 0;
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
        for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
            if (source != CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID) {
                continue;
            }
            unsigned int phase = peripheral.phase;
            uint64_t age = now - peripheral.confirmed_ms;
#else
            unsigned int phase = central.peers[source].phase;
            uint64_t age = now - central.peers[source].last_seen_ms;
#endif
            LOG_INF("peer diag source=%u link=%u auth=%u input=%u age_ms=%llu wire=%u "
                    "auth_rx=%u auth_bad=%u auth_err=%d data_rx=%u data_bad=%u data_err=%d "
                    "accepted=%u",
                    source, phase, secure_peer(source)->phase, nemu_link_link_input_allowed(source),
                    (unsigned long long)age, peer_diag[source].last_wire_kind,
                    peer_diag[source].auth_rx, peer_diag[source].auth_rejected,
                    peer_diag[source].last_auth_error, peer_diag[source].data_rx,
                    peer_diag[source].data_rejected, peer_diag[source].last_data_error,
                    peer_diag[source].accepted);
            LOG_INF("auth detail v2 source=%u wire=%u phase_before=%u challenge_matches=%u "
                    "mac_checked=%u mac_status=%d key_fp=%08x err=%d",
                    source, peer_diag[source].last_wire_kind,
                    peer_diag[source].auth_detail.phase_before,
                    peer_diag[source].auth_detail.challenge_matches,
                    peer_diag[source].auth_detail.mac_checked,
                    peer_diag[source].auth_detail.mac_status,
                    peer_diag[source].auth_detail.loaded_key_fingerprint,
                    peer_diag[source].last_auth_error);
        }
        for (unsigned source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
            for (unsigned k = 0; k < NEMU_LINK_WIRE_KIND_COUNT; k++) {
                if (wire_tx[source][k].count)
                    LOG_INF("compact tx source=%u kind=%02x count=%u bytes=%u min=%u max=%u",
                            source, k + NEMU_LINK_WIRE_LINK, wire_tx[source][k].count,
                            wire_tx[source][k].bytes, wire_tx[source][k].min,
                            wire_tx[source][k].max);
            }
        }
        nemu_link_split_diag_log();
        nemu_link_radio_diag_log();
#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
        nemu_link_settings_esb_diag_log();
#endif
        nemu_link_profile_log();
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
        LOG_INF("idle idle phase=%u sleep=%u wake=%u error=%u pending=%ld", idle.phase, idle_sleeps,
                idle_wakes, idle_errors, (long)atomic_get(&tx_pending));
        LOG_INF("idle idle off_ms=%llu wake_max_ms=%u",
                (unsigned long long)(idle_off_ms + (idle.phase == NEMU_LINK_IDLE_OFF
                                                        ? (uint64_t)k_uptime_get() - idle_off_since
                                                        : 0)),
                wake_max_ms);
#endif
        next_stats = now + 3000;
        nemu_link_trace_mark(NEMU_LINK_TRACE_DIAG_END, 0);
    }
    uint32_t service_delay = SERVICE_MS;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    /* Clock start may consume part of this tick. Do not add that duration to
     * every subsequent heartbeat interval. */
    uint64_t schedule_now = k_uptime_get();
    uint64_t next_tx = next_tx_ms(schedule_now);
    /* OFF needs no 10ms polling: TX IRQ and local input reschedule this work.
     * Keep the short cadence when auto-BLE needs its own timeout check. */
    uint32_t maximum =
        idle.phase == NEMU_LINK_IDLE_OFF && !IS_ENABLED(CONFIG_ZMK_NEMU_LINK_AUTO_BLE)
            ? UINT32_MAX
            : service_delay;
    service_delay = nemu_link_idle_next_service(&idle, schedule_now, next_tx, maximum);
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    /* Keep diagnostic visibility during a long RF-off discovery gap. This
     * schedules CPU work only; idle policy still decides whether RF wakes. */
    uint64_t diagnostic_now = k_uptime_get();
    service_delay =
        MIN(service_delay, next_stats > diagnostic_now ? next_stats - diagnostic_now : 0);
#endif
    k_work_schedule(&link_work, K_MSEC(service_delay));
    nemu_link_trace_mark(NEMU_LINK_TRACE_WORK_END, 0);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_POWER_TRACE)
    if (idle.phase == NEMU_LINK_IDLE_OFF) {
        nemu_link_trace_finish(0);
    }
#endif
    k_mutex_unlock(&link_lock);
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE) &&                                               \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
static void profile_switch(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t target = profile_target;
    if (target == selected_dongle || target < 1 || target > CONFIG_ZMK_NEMU_LINK_DONGLE_COUNT) {
        return;
    }
    int err = nemu_link_link_enable(false);
    if (err) {
        LOG_ERR("dongle switch stop failed %d", err);
        return;
    }
    selected_dongle = target;
    err = nemu_link_link_enable(true);
    if (err) {
        LOG_ERR("dongle switch start failed %d", err);
    } else {
        LOG_INF("selected dongle %u", target);
    }
}
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
int nemu_link_link_studio_send(const struct nemu_link_studio_route *route, const uint8_t *bytes,
                               size_t size) {
    k_mutex_lock(&link_lock, K_FOREVER);
    int err = -EAGAIN;
    struct nemu_link_studio_route current;
    if (!enabled) {
        goto out;
    }
    if (!nemu_link_link_input_allowed(route->source)) {
        goto out;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    if (idle_wake()) {
        goto out;
    }
#endif
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    bool valid = nemu_link_studio_route_central(&current, &central, &hop, k_uptime_get());
#else
    bool valid = nemu_link_studio_route_peripheral(&current, &peripheral, &hop, k_uptime_get());
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    current.dongle_id = selected_dongle;
#endif
    if (!valid || current.dongle_id != route->dongle_id || current.boot != route->boot ||
        current.session != route->session || current.nonce != route->nonce ||
        current.channel_epoch != route->channel_epoch ||
        current.channel_index != route->channel_index) {
        goto out;
    }
    uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
    size_t encrypted_size = 0;
    err = nemu_link_secure_wrap(secure_peer(route->source), CENTRAL, route->source, bytes, size,
                                encrypted, &encrypted_size);
    if (!err) {
        err = send_frame(route->source, encrypted, encrypted_size);
    }
out:
    k_mutex_unlock(&link_lock);
    return err;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_PROTOCOL)
int nemu_link_link_settings_esb_send(const struct nemu_link_studio_packet *packet, uint8_t source) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (!packet || !nemu_link_studio_packet_valid(packet)) {
        return -EINVAL;
    }
    k_mutex_lock(&link_lock, K_FOREVER);
    int err = -EHOSTUNREACH;
    struct nemu_link_studio_route route;
    uint8_t bytes[NEMU_LINK_STUDIO_FRAME_SIZE];
    uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
    size_t encrypted_size = 0;
    if (!enabled || source >= NEMU_LINK_LINK_SOURCES ||
        !nemu_link_studio_route_central_source(&route, &central, &hop, source, k_uptime_get()) ||
        !nemu_link_link_input_allowed(source)) {
        goto out;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    err = idle_wake();
    if (err) {
        goto out;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    route.dongle_id = selected_dongle;
#endif
    bool owner_request = packet->kind == NEMU_LINK_STUDIO_OWNER_RPC_REQUEST;
    if (owner_request && settings_request_tx.pending) {
        bool same_packet = settings_request_tx.source == source &&
                           settings_request_tx.packet.transaction == packet->transaction &&
                           settings_request_tx.packet.offset == packet->offset &&
                           settings_request_tx.packet.length == packet->length &&
                           settings_request_tx.packet.kind == packet->kind &&
                           !memcmp(settings_request_tx.packet.data, packet->data, packet->length) &&
                           nemu_link_studio_route_same_session(&route, &settings_request_tx.route);
        uint16_t pending_id = settings_request_tx.msg_id;
        k_mutex_unlock(&link_lock);
        int wait_err = wait_for_tx(pending_id);
        if (wait_err == -ETIMEDOUT) {
            return same_packet ? wait_err : -EBUSY;
        }
        k_mutex_lock(&link_lock, K_FOREVER);
        if (settings_request_tx.pending && settings_request_tx.msg_id == pending_id) {
            settings_request_tx.pending = false;
        }
        k_mutex_unlock(&link_lock);
        if (!same_packet) {
            return nemu_link_link_settings_esb_send(packet, source);
        }
        return wait_err;
    }
    size_t size = nemu_link_studio_frame_encode(bytes, sizeof(bytes), &route, packet, FRAME_CRC);
    if (!size) {
        err = -EINVAL;
        goto out;
    }
    err = nemu_link_secure_wrap(secure_peer(source), CENTRAL, source, bytes, size, encrypted,
                                &encrypted_size);
    uint16_t wait_id = 0;
    if (!err) {
        /* Enqueue against the route selected under link_lock, then release the
         * lock before waiting so link service can process hop and ACK state. */
        err = send_frame_internal(source, encrypted, encrypted_size, true, &wait_id);
        if (!err && owner_request) {
            settings_request_tx.pending = true;
            settings_request_tx.source = source;
            settings_request_tx.msg_id = wait_id;
            settings_request_tx.packet = *packet;
            settings_request_tx.route = route;
        }
    }
    k_mutex_unlock(&link_lock);
    if (!err) {
        err = wait_for_tx(wait_id);
        if (err != -ETIMEDOUT && owner_request) {
            k_mutex_lock(&link_lock, K_FOREVER);
            if (settings_request_tx.pending && settings_request_tx.msg_id == wait_id) {
                settings_request_tx.pending = false;
            }
            k_mutex_unlock(&link_lock);
        }
    }
    return err;
out:
    k_mutex_unlock(&link_lock);
    return err;
#else
    ARG_UNUSED(packet);
    ARG_UNUSED(source);
    return -ENOTSUP;
#endif
}

int nemu_link_link_settings_esb_reply(const struct nemu_link_studio_route *route,
                                      const struct nemu_link_studio_packet *packet) {
#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (!route || !packet || route->local_side != NEMU_LINK_STUDIO_KEYBOARD ||
        route->source != CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID ||
        !nemu_link_studio_packet_valid(packet) ||
        packet->kind != NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE) {
        return -EINVAL;
    }
    k_mutex_lock(&link_lock, K_FOREVER);
    int err = -EHOSTUNREACH;
    struct nemu_link_studio_route current;
    if (!enabled || !nemu_link_link_input_allowed(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID)) {
        goto reply_out;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    err = idle_wake();
    if (err) {
        goto reply_out;
    }
#endif
    if (!nemu_link_studio_route_peripheral_source(
            &current, &peripheral, &hop, CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, k_uptime_get()) ||
        !nemu_link_studio_route_same_session(&current, route)) {
        goto reply_out;
    }
    uint8_t bytes[NEMU_LINK_STUDIO_FRAME_SIZE];
    uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
    size_t encrypted_size = 0;
    size_t size = nemu_link_studio_frame_encode(bytes, sizeof(bytes), &current, packet, FRAME_CRC);
    if (!size) {
        err = -EINVAL;
        goto reply_out;
    }
    err = nemu_link_secure_wrap(secure_peer(current.source), CENTRAL, current.source, bytes, size,
                                encrypted, &encrypted_size);
reply_out:
    k_mutex_unlock(&link_lock);
    if (!err) {
        uint64_t deadline = k_uptime_get() + 5000;
        do {
            err = send_frame_and_wait(current.source, encrypted, encrypted_size);
            if (err != -EBUSY && err != -EAGAIN && err != -ENOMSG) {
                break;
            }
            if (k_uptime_get() >= deadline) {
                break;
            }
            k_sleep(K_MSEC(5));
        } while (true);
    }
    return err;
#else
    ARG_UNUSED(route);
    ARG_UNUSED(packet);
    return -ENOTSUP;
#endif
}
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
BUILD_ASSERT(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID >= 0 && CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID < 2);
#endif
#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) &&                                                  \
    !IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
void zmk_nemu_link_set_receiver_ready(bool ready) { ARG_UNUSED(ready); }
#endif
