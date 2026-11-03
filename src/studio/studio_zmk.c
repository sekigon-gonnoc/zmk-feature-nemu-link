#include <errno.h>
#include <zmk/physical_layouts.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#if IS_ENABLED(CONFIG_SETTINGS_NVS)
#include <zephyr/fs/nvs.h>
#endif
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <zmk/nemu_link/studio_zmk.h>
#include <zmk/nemu_link/studio_group_stream.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
#include <zmk/dynamic_settings/schema.h>
#include <zmk/dynamic_settings/store.h>
#include <zmk/nemu_link/settings_relay.h>
#include "settings_schema_generated.h"
#endif
LOG_MODULE_REGISTER(dongle_studio, LOG_LEVEL_INF);
BUILD_ASSERT(IS_ENABLED(CONFIG_ZMK_NEMU_LINK_HOP) && IS_ENABLED(CONFIG_ZMK_STUDIO));
#define FRAME_CRC NULL
#define ACTIVE_SERVICE_MS 5
#define SIDE                                                                                       \
    (IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE) ? NEMU_LINK_STUDIO_DONGLE                           \
                                               : NEMU_LINK_STUDIO_KEYBOARD)
static K_MUTEX_DEFINE(store_lock);
static atomic_t synchronized, discovery_pending, remote_notice, worker_started;
static atomic_t editing_busy, radio_busy, loaded, received_frames, dropped_frames;
static bool ready, have_metadata;
static struct nemu_link_studio_version live;
static struct nemu_link_studio_tx tx;
static struct nemu_link_studio_stream_rx rx;
static uint64_t next_offer;
static bool peer_schema_matches;
static uint8_t last_keymap_receive_error;
static uint32_t last_keymap_receive_error_code;
static uint32_t last_keymap_receive_error_index;
static bool tx_active(void);
static bool rx_active(void);
static bool waiting_for_keyboard;
static uint8_t last_keymap_offer_decision;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static struct nemu_link_studio_version shared_live;
static struct nemu_link_studio_tx shared_tx;
static struct nemu_link_studio_group_rx shared_rx;
static atomic_t shared_save_pending;
static bool have_shared_metadata, shared_ready, shared_synchronized;
static uint64_t shared_next_offer;
static uint8_t last_shared_offer_decision;
static uint32_t shared_generation;
static atomic_t shared_last_stale_reason;
static uint8_t peer_shared_contract_hash[32], peer_shared_contract_received;
static uint8_t local_shared_contract_sent;
static bool peer_shared_contract_known, peer_shared_contract_matches;
static uint32_t local_shared_session, local_shared_sequence;
static uint32_t local_preview_session;
static uint64_t local_preview_base[2];
static bool shared_tx_active(void);
static struct {
    uint32_t offset;
    size_t index, used, size;
    uint8_t header[6];
} shared_reader;
static bool remote_sequence_newer(uint32_t candidate, uint32_t current);
static bool shared_rx_active(void);
static bool shared_control_packet(const struct nemu_link_studio_packet *packet) {
    switch (packet->kind) {
    case NEMU_LINK_STUDIO_REMOTE_REQUEST:
    case NEMU_LINK_STUDIO_REMOTE_RESPONSE:
    case NEMU_LINK_STUDIO_REMOTE_BASE:
    case NEMU_LINK_STUDIO_REMOTE_BASE_ACK:
    case NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH:
    case NEMU_LINK_STUDIO_SCHEMA_HASH:
    case NEMU_LINK_STUDIO_GROUP_META:
    case NEMU_LINK_STUDIO_GROUP_DATA:
    case NEMU_LINK_STUDIO_GROUP_COMMIT:
    case NEMU_LINK_STUDIO_GROUP_META_ACK:
    case NEMU_LINK_STUDIO_GROUP_DATA_ACK:
    case NEMU_LINK_STUDIO_GROUP_COMMIT_ACK:
        return true;
    case NEMU_LINK_STUDIO_ERROR:
        return packet->length >= 2 && (packet->data[1] == NEMU_LINK_STUDIO_GROUP_META ||
                                       packet->data[1] == NEMU_LINK_STUDIO_GROUP_DATA ||
                                       packet->data[1] == NEMU_LINK_STUDIO_GROUP_COMMIT);
    default:
        return false;
    }
}
#else
static bool shared_control_packet(const struct nemu_link_studio_packet *packet) {
    ARG_UNUSED(packet);
    return false;
}
#endif

enum {
    SHARED_STALE_NONE,
    SHARED_STALE_SESSION_SEQUENCE,
    SHARED_STALE_NOT_READY,
    SHARED_STALE_NOT_SYNCHRONIZED,
    SHARED_STALE_SAVE_PENDING,
    SHARED_STALE_RECEIVE_ACTIVE,
    SHARED_STALE_REVISION_MISMATCH,
    SHARED_STALE_GENERATION_CHANGED,
    SHARED_STALE_CONTRACT_MISMATCH,
    SHARED_STALE_REMOTE_REJECTED,
};

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static uint8_t shared_stale_state_reason(void) {
    if (!shared_ready)
        return SHARED_STALE_NOT_READY;
    if (!shared_synchronized)
        return SHARED_STALE_NOT_SYNCHRONIZED;
    if (atomic_get(&shared_save_pending))
        return SHARED_STALE_SAVE_PENDING;
    if (shared_rx_active())
        return SHARED_STALE_RECEIVE_ACTIVE;
    if (dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED) != shared_generation)
        return SHARED_STALE_GENERATION_CHANGED;
    if (memcmp(shared_live.schema, dynamic_settings_shared_contract_sha256, 32))
        return SHARED_STALE_CONTRACT_MISMATCH;
    return SHARED_STALE_NONE;
}
#endif

void nemu_link_studio_get_shared_diagnostics(
    struct nemu_link_studio_shared_diagnostics *diagnostics) {
    if (!diagnostics)
        return;
    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->peripheral_0_rssi = -128;
    diagnostics->peripheral_1_rssi = -128;
    int8_t rssi_dbm;
    if (nemu_link_studio_get_peripheral_rssi(0, &rssi_dbm)) {
        diagnostics->peripheral_0_rssi = rssi_dbm;
    }
    if (nemu_link_studio_get_peripheral_rssi(1, &rssi_dbm)) {
        diagnostics->peripheral_1_rssi = rssi_dbm;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    diagnostics->ready = shared_ready;
    diagnostics->synchronized = shared_synchronized;
    diagnostics->save_pending = atomic_get(&shared_save_pending) != 0;
    diagnostics->receive_active = shared_rx_active();
    diagnostics->contract_matches =
        !memcmp(shared_live.schema, dynamic_settings_shared_contract_sha256, 32);
    diagnostics->peer_shared_contract_known = peer_shared_contract_known;
    diagnostics->peer_shared_contract_matches = peer_shared_contract_matches;
    diagnostics->keymap_synchronized = atomic_get(&synchronized) != 0;
    diagnostics->keymap_schema_matches = peer_schema_matches;
    diagnostics->keymap_dirty = nemu_link_studio_dirty() != 0;
    diagnostics->keymap_transfer_active = tx_active();
    diagnostics->keymap_offer_pending = next_offer != UINT64_MAX;
    diagnostics->keymap_receive_active = rx_active();
    diagnostics->keymap_waiting_for_keyboard = waiting_for_keyboard;
    diagnostics->keymap_offer_decision = last_keymap_offer_decision;
    diagnostics->keymap_transfer_phase = tx.phase;
    diagnostics->keymap_transfer_error = tx.error;
    diagnostics->keymap_receive_error = last_keymap_receive_error;
    diagnostics->keymap_receive_error_code = last_keymap_receive_error_code;
    diagnostics->keymap_receive_error_index = last_keymap_receive_error_index;
    diagnostics->keymap_transfer_offset = tx.offset;
    diagnostics->keymap_transfer_size = tx.transfer_size;
    uint64_t now = k_uptime_get();
    diagnostics->keymap_transfer_age_ms =
        tx.last_progress && now >= tx.last_progress ? MIN(now - tx.last_progress, UINT32_MAX) : 0;
    diagnostics->keymap_receive_offset = rx.received;
    diagnostics->keymap_receive_size = rx.transfer_size;
    diagnostics->keymap_receive_metadata = rx.metadata_received;
    diagnostics->shared_dirty = dynamic_settings_scope_is_dirty(DYNAMIC_SETTING_SCOPE_SHARED);
    diagnostics->shared_transfer_active = shared_tx_active();
    diagnostics->shared_offer_pending = shared_next_offer != UINT64_MAX;
    diagnostics->shared_offer_decision = last_shared_offer_decision;
    diagnostics->shared_transfer_phase = shared_tx.phase;
    diagnostics->shared_transfer_error = shared_tx.error;
    diagnostics->shared_transfer_offset = shared_tx.offset;
    diagnostics->shared_receive_metadata = shared_rx.metadata_received;
    diagnostics->store_generation = dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED);
    diagnostics->synchronized_generation = shared_generation;
    diagnostics->keyboard_revision = shared_live.clock[NEMU_LINK_STUDIO_KEYBOARD];
    diagnostics->dongle_revision = shared_live.clock[NEMU_LINK_STUDIO_DONGLE];
    diagnostics->last_stale_reason = atomic_get(&shared_last_stale_reason);
#endif
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
enum nvs_diagnostic_state {
    NVS_DIAGNOSTIC_UNSUPPORTED,
    NVS_DIAGNOSTIC_NOT_READY,
    NVS_DIAGNOSTIC_READY,
    NVS_DIAGNOSTIC_STATS_ERROR,
};

struct nvs_diagnostic_snapshot {
    enum nvs_diagnostic_state state;
    uint32_t total_bytes;
    uint32_t free_bytes;
};

static K_MUTEX_DEFINE(nvs_diagnostic_lock);
static struct nvs_diagnostic_snapshot nvs_diagnostic_cache;
static int64_t nvs_diagnostic_cache_time;
static bool nvs_diagnostic_cache_valid;

static struct nvs_diagnostic_snapshot get_nvs_diagnostic_snapshot(void) {
    k_mutex_lock(&nvs_diagnostic_lock, K_FOREVER);
    int64_t now = k_uptime_get();
    if (nvs_diagnostic_cache_valid && now >= nvs_diagnostic_cache_time &&
        now - nvs_diagnostic_cache_time < 1000) {
        struct nvs_diagnostic_snapshot snapshot = nvs_diagnostic_cache;
        k_mutex_unlock(&nvs_diagnostic_lock);
        return snapshot;
    }

    struct nvs_diagnostic_snapshot snapshot = {.state = NVS_DIAGNOSTIC_UNSUPPORTED};
#if IS_ENABLED(CONFIG_SETTINGS_NVS)
    void *storage = NULL;
    if (settings_storage_get(&storage) == 0 && storage != NULL) {
        struct nvs_fs *fs = storage;
        snapshot.total_bytes = (uint32_t)fs->sector_size * fs->sector_count;
        if (!fs->ready) {
            snapshot.state = NVS_DIAGNOSTIC_NOT_READY;
        } else {
            k_mutex_lock(&fs->nvs_lock, K_FOREVER);
            ssize_t free_bytes = nvs_calc_free_space(fs);
            k_mutex_unlock(&fs->nvs_lock);
            if (free_bytes < 0) {
                snapshot.state = NVS_DIAGNOSTIC_STATS_ERROR;
            } else {
                snapshot.state = NVS_DIAGNOSTIC_READY;
                snapshot.free_bytes = MIN((uint64_t)free_bytes, UINT32_MAX);
            }
        }
    }
#endif
    nvs_diagnostic_cache = snapshot;
    nvs_diagnostic_cache_time = now;
    nvs_diagnostic_cache_valid = true;
    k_mutex_unlock(&nvs_diagnostic_lock);
    return snapshot;
}

int nemu_link_studio_settings_diagnostic_default(
    const struct dynamic_setting_descriptor *descriptor, struct dynamic_setting_value *value,
    const void *context) {
    ARG_UNUSED(context);
    if (!descriptor || !value)
        return -EINVAL;
    struct nemu_link_studio_shared_diagnostics diagnostics;
    nemu_link_studio_get_shared_diagnostics(&diagnostics);
    struct nvs_diagnostic_snapshot nvs = {0};
    uint32_t number;
    /* Keep the historical ID namespace so the Studio Settings schema stays stable. */
    switch (descriptor->id) {
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_READY_LINK:
        value->data[0] = diagnostics.ready;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SYNCHRONIZED_LINK:
        value->data[0] = diagnostics.synchronized;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SAVE_PENDING_LINK:
        value->data[0] = diagnostics.save_pending;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_RECEIVE_ACTIVE_LINK:
        value->data[0] = diagnostics.receive_active;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_CONTRACT_MATCHES_LINK:
        value->data[0] = diagnostics.contract_matches;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_PEER_SHARED_CONTRACT_KNOWN_LINK:
        value->data[0] = diagnostics.peer_shared_contract_known;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_PEER_SHARED_CONTRACT_MATCHES_LINK:
        value->data[0] = diagnostics.peer_shared_contract_matches;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_PERIPHERAL_0_RSSI_LINK:
        sys_put_le16((uint16_t)diagnostics.peripheral_0_rssi, value->data);
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_PERIPHERAL_1_RSSI_LINK:
        sys_put_le16((uint16_t)diagnostics.peripheral_1_rssi, value->data);
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_SYNCHRONIZED_LINK:
        value->data[0] = diagnostics.keymap_synchronized;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_SCHEMA_MATCHES_LINK:
        value->data[0] = diagnostics.keymap_schema_matches;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_DIRTY_LINK:
        value->data[0] = diagnostics.keymap_dirty;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_ACTIVE_LINK:
        value->data[0] = diagnostics.keymap_transfer_active;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_OFFER_PENDING_LINK:
        value->data[0] = diagnostics.keymap_offer_pending;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_ACTIVE_LINK:
        value->data[0] = diagnostics.keymap_receive_active;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_WAITING_FOR_KEYBOARD_LINK:
        value->data[0] = diagnostics.keymap_waiting_for_keyboard;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_OFFER_DECISION_LINK:
        value->data[0] = diagnostics.keymap_offer_decision;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_PHASE_LINK:
        value->data[0] = diagnostics.keymap_transfer_phase;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_ERROR_LINK:
        value->data[0] = diagnostics.keymap_transfer_error;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_ERROR_LINK:
        value->data[0] = diagnostics.keymap_receive_error;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_DIRTY_LINK:
        value->data[0] = diagnostics.shared_dirty;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_TRANSFER_ACTIVE_LINK:
        value->data[0] = diagnostics.shared_transfer_active;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_OFFER_PENDING_LINK:
        value->data[0] = diagnostics.shared_offer_pending;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_OFFER_DECISION_LINK:
        value->data[0] = diagnostics.shared_offer_decision;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_TRANSFER_PHASE_LINK:
        value->data[0] = diagnostics.shared_transfer_phase;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_TRANSFER_ERROR_LINK:
        value->data[0] = diagnostics.shared_transfer_error;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_STORE_GENERATION_LINK:
        number = diagnostics.store_generation;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SYNC_GENERATION_LINK:
        number = diagnostics.synchronized_generation;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_NVS_STATE_LINK:
        nvs = get_nvs_diagnostic_snapshot();
        value->data[0] = nvs.state;
        return 0;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_NVS_TOTAL_BYTES_LINK:
        nvs = get_nvs_diagnostic_snapshot();
        number = nvs.total_bytes;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_NVS_FREE_BYTES_LINK:
        nvs = get_nvs_diagnostic_snapshot();
        number = nvs.free_bytes;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_OFFSET_LINK:
        number = diagnostics.keymap_transfer_offset;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_SIZE_LINK:
        number = diagnostics.keymap_transfer_size;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_TRANSFER_AGE_MS_LINK:
        number = diagnostics.keymap_transfer_age_ms;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_OFFSET_LINK:
        number = diagnostics.keymap_receive_offset;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_SIZE_LINK:
        number = diagnostics.keymap_receive_size;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_METADATA_LINK:
        number = diagnostics.keymap_receive_metadata;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_ERROR_CODE_LINK:
        number = diagnostics.keymap_receive_error_code;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYMAP_RECEIVE_ERROR_INDEX_LINK:
        number = diagnostics.keymap_receive_error_index;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_TRANSFER_OFFSET_LINK:
        number = diagnostics.shared_transfer_offset;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_SHARED_RECEIVE_METADATA_LINK:
        number = diagnostics.shared_receive_metadata;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYBOARD_REVISION_LOW_LINK:
        number = (uint32_t)diagnostics.keyboard_revision;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_KEYBOARD_REVISION_HIGH_LINK:
        number = (uint32_t)(diagnostics.keyboard_revision >> 32);
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_DONGLE_REVISION_LOW_LINK:
        number = (uint32_t)diagnostics.dongle_revision;
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_DONGLE_REVISION_HIGH_LINK:
        number = (uint32_t)(diagnostics.dongle_revision >> 32);
        break;
    case DYNAMIC_SETTING_ID_NEMU_LINK_SHARED_DIAGNOSTICS_LAST_STALE_REASON_LINK:
        value->data[0] = diagnostics.last_stale_reason;
        return 0;
    default:
        return -ENOENT;
    }
    sys_put_le32(number, value->data);
    return 0;
}
#endif
/* Native pending bits are captured before save; only one synchronized-base save
 * is eligible. Repeated/offline saves and interrupted imports use the full map. */
#define DELTA_RECORDS (1 + NEMU_LINK_STUDIO_MAX_LAYERS * (1 + NEMU_LINK_STUDIO_MAX_POSITIONS))
static uint8_t delta_records[(DELTA_RECORDS + 7) / 8];
static struct nemu_link_studio_version delta_base;
static bool delta_valid;
static uint16_t delta_size;
static struct {
    uint32_t offset;
    uint16_t index;
    size_t used, size;
    uint8_t record[NEMU_LINK_STUDIO_RECORD_MAX + 3];
    bool delta;
} reader;
static bool delta_selected(uint16_t index) {
    return !!(delta_records[index / 8] & (1u << (index % 8)));
}
static struct k_spinlock route_lock;
static struct nemu_link_studio_route link_route, session_route;
static bool route_valid, session_valid;
struct rx_item {
    uint64_t time;
    size_t size;
    uint8_t bytes[64];
};
K_MSGQ_DEFINE(control_rx, sizeof(struct rx_item), 6, 4);
static struct k_work_q worker;
K_THREAD_STACK_DEFINE(worker_stack, 2048);
static void service(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sync_work, service);
static struct nemu_link_studio_packet response;
static bool response_pending;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static struct nemu_link_studio_packet owner_schema_reply[2];
static uint8_t owner_schema_reply_pending;
static uint8_t owner_schema_reply_next;
#endif
static uint64_t last_poll, poll_until;
static uint32_t save_count, applied_count, sent_frames, rejected_frames, send_errors, send_waits;
static int last_send_error;
static uint64_t next_status;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static K_MUTEX_DEFINE(remote_lock);
static K_MUTEX_DEFINE(owner_esb_call_lock);
static K_SEM_DEFINE(remote_done_sem, 0, 1);
static struct dynamic_setting_remote_request remote_request;
static struct dynamic_setting_remote_response remote_response;
static bool remote_inflight, remote_sent, remote_done;
static bool remote_base_needed, remote_base_sent, remote_base_acked;
static uint64_t remote_shared_base[2];
static int remote_error;
static uint64_t remote_last_sent;
static uint16_t remote_transaction;
static bool remote_response_received, remote_owner_schema_expected;
static uint8_t remote_owner_schema_received, remote_owner_schema_hash[32];
static uint32_t owner_last_session, owner_last_sequence;
static uint16_t owner_last_transaction;
static bool owner_base_valid;
static uint16_t owner_base_transaction;
static uint8_t owner_base_request[NEMU_LINK_STUDIO_CHUNK_SIZE];
static uint64_t owner_base_clock[2];
static uint8_t owner_last_request[NEMU_LINK_STUDIO_CHUNK_SIZE];
static struct nemu_link_studio_packet owner_last_response;
#endif
static bool same_session(const struct nemu_link_studio_route *a,
                         const struct nemu_link_studio_route *b);
static void wake_service(void) {
    if (atomic_get(&worker_started)) {
        k_work_reschedule_for_queue(&worker, &sync_work, K_NO_WAIT);
    }
}
bool nemu_link_studio_pending(void) { return atomic_get(&discovery_pending); }
void nemu_link_studio_notify(void) {
    /* A level notice starts a collection window only while idle. Repeated ACKs
     * during a transfer/drain must not push its deadline forward indefinitely.
     * No map/store mutex under the link mutex. */
    if (atomic_get(&radio_busy)) {
        return;
    }
    atomic_set(&remote_notice, 1);
    atomic_set(&radio_busy, 1);
    wake_service();
}

bool nemu_link_studio_edit_allowed(void) { return !atomic_get(&editing_busy); }
bool nemu_link_studio_synced(void) { return atomic_get(&synchronized); }
bool nemu_link_studio_radio_busy(void) { return atomic_get(&radio_busy); }
static bool tx_active(void) {
    return tx.phase >= NEMU_LINK_STUDIO_TX_META && tx.phase <= NEMU_LINK_STUDIO_TX_COMMIT;
}
static bool rx_active(void) {
    /* META is part of the transfer too: ACK payloads require left-side polls
     * before enough metadata has arrived to set rx.started. */
    return rx.transaction && !rx.expired && !rx.receipt;
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static bool shared_tx_active(void) {
    return shared_tx.phase >= NEMU_LINK_STUDIO_TX_META &&
           shared_tx.phase <= NEMU_LINK_STUDIO_TX_COMMIT;
}

static bool shared_rx_active(void) {
    /* A partial META is already an in-flight saved sync and must exclude
     * STATUS/preview/save-base requests just like a started DATA transfer. */
    return shared_rx.transaction && !shared_rx.expired && !shared_rx.receipt;
}

static void discard_shared_edits(void) {
    for (size_t i = 0; i < dynamic_settings_shared_group_count(); i++) {
        const struct dynamic_setting_group_descriptor *group = dynamic_settings_shared_group_at(i);
        if (group && dynamic_settings_is_dirty(group->id)) {
            int err = dynamic_settings_discard(group->id);
            if (err) {
                LOG_WRN("Shared session discard failed id=%08x err=%d", group->id, err);
            }
        }
    }
}

static int describe_shared(struct nemu_link_studio_version *version) {
    uint32_t checksum = 0;
    uint32_t size = 0;
    size_t count = dynamic_settings_shared_group_count();
    uint32_t generation = dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED);
    memcpy(version->schema, dynamic_settings_shared_contract_sha256, sizeof(version->schema));
    for (size_t i = 0; i < count; i++) {
        const struct dynamic_setting_group_descriptor *group = dynamic_settings_shared_group_at(i);
        size_t record_size;
        if (!group ||
            dynamic_settings_read_shared_group_record(group->id, 0, NULL, 0, &record_size) ||
            record_size > UINT16_MAX || size + 6 + record_size > UINT16_MAX) {
            return -EBUSY;
        }
        uint8_t header[6] = {
            (uint8_t)group->id,         (uint8_t)(group->id >> 8), (uint8_t)(group->id >> 16),
            (uint8_t)(group->id >> 24), (uint8_t)record_size,      (uint8_t)(record_size >> 8),
        };
        checksum = nemu_link_studio_checksum(checksum, size, header, sizeof(header));
        size += sizeof(header);
        for (size_t offset = 0; offset < record_size;) {
            uint8_t chunk[NEMU_LINK_STUDIO_CHUNK_SIZE];
            size_t chunk_size = MIN(sizeof(chunk), record_size - offset);
            size_t actual_size;
            int err = dynamic_settings_read_shared_group_record(group->id, offset, chunk,
                                                                chunk_size, &actual_size);
            if (err || actual_size != record_size) {
                return err ? err : -ESTALE;
            }
            checksum = nemu_link_studio_checksum(checksum, size, chunk, chunk_size);
            size += chunk_size;
            offset += chunk_size;
        }
    }
    if (!size) {
        return count ? -EIO : -ENOENT;
    }
    if (generation != dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED)) {
        return -ESTALE;
    }
    nemu_link_studio_checksum_bytes(checksum, version->checksum);
    version->size = size;
    return 0;
}

static int persist_shared_version(const struct nemu_link_studio_version *version) {
    uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (!nemu_link_studio_version_encode(bytes, version)) {
        return -EINVAL;
    }
    return settings_save_one("esbc1/v/shared_config", bytes, sizeof(bytes));
}

static uint32_t shared_group_id(void *context, size_t index) {
    ARG_UNUSED(context);
    const struct dynamic_setting_group_descriptor *group = dynamic_settings_shared_group_at(index);
    return group ? group->id : 0;
}

static bool apply_shared_group(void *context, uint32_t group_id, const uint8_t *record,
                               size_t size) {
    ARG_UNUSED(context);
    int err = dynamic_settings_is_shared_preview(group_id)
                  ? dynamic_settings_replace_shared_preview(group_id, record, size)
                  : dynamic_settings_apply_shared_group(group_id, record, size);
    if (err) {
        LOG_WRN("Shared group apply failed id=%08x err=%d", group_id, err);
    }
    return !err;
}

static bool commit_shared_received(void *context, struct nemu_link_studio_version *version) {
    ARG_UNUSED(context);
    struct nemu_link_studio_version actual = *version;
    int err = describe_shared(&actual);
    if (!err && (actual.size != version->size || memcmp(actual.checksum, version->checksum, 32))) {
        err = -EIO;
    }
    if (!err) {
        err = persist_shared_version(version);
    }
    if (err) {
        LOG_ERR("Shared settings commit failed %d", err);
        return false;
    }
    shared_live = *version;
    shared_synchronized = true;
    atomic_set(&shared_last_stale_reason, SHARED_STALE_NONE);
    shared_next_offer = UINT64_MAX;
    shared_generation = dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED);
    LOG_INF("Shared settings applied K=%llu D=%llu", version->clock[0], version->clock[1]);
    return true;
}

static bool read_shared_image(void *context, uint32_t offset, uint8_t *bytes, size_t size) {
    ARG_UNUSED(context);
    if (offset != shared_reader.offset) {
        return false;
    }
    while (size) {
        if (shared_reader.used == shared_reader.size) {
            const struct dynamic_setting_group_descriptor *group =
                dynamic_settings_shared_group_at(shared_reader.index);
            size_t record_size;
            if (!group ||
                dynamic_settings_read_shared_group_record(group->id, 0, NULL, 0, &record_size) ||
                record_size > UINT16_MAX) {
                return false;
            }
            shared_reader.header[0] = (uint8_t)group->id;
            shared_reader.header[1] = (uint8_t)(group->id >> 8);
            shared_reader.header[2] = (uint8_t)(group->id >> 16);
            shared_reader.header[3] = (uint8_t)(group->id >> 24);
            shared_reader.header[4] = (uint8_t)record_size;
            shared_reader.header[5] = (uint8_t)(record_size >> 8);
            shared_reader.used = 0;
            shared_reader.size = sizeof(shared_reader.header) + record_size;
            shared_reader.index++;
        }
        size_t count = MIN(size, shared_reader.size - shared_reader.used);
        size_t cursor = shared_reader.used;
        if (cursor < sizeof(shared_reader.header)) {
            size_t header_count = MIN(count, sizeof(shared_reader.header) - cursor);
            memcpy(bytes, shared_reader.header + cursor, header_count);
            shared_reader.used += header_count;
            shared_reader.offset += header_count;
            bytes += header_count;
            size -= header_count;
            count -= header_count;
            cursor += header_count;
        }
        if (count) {
            size_t record_offset = cursor - sizeof(shared_reader.header);
            size_t record_size;
            const struct dynamic_setting_group_descriptor *group =
                dynamic_settings_shared_group_at(shared_reader.index - 1);
            if (!group ||
                dynamic_settings_read_shared_group_record(group->id, record_offset, bytes, count,
                                                          &record_size) ||
                record_size + sizeof(shared_reader.header) != shared_reader.size) {
                return false;
            }
            shared_reader.used += count;
            shared_reader.offset += count;
            bytes += count;
            size -= count;
        }
    }
    return true;
}

static void shared_scope_saved(enum dynamic_setting_scope scope, void *context) {
    ARG_UNUSED(context);
    if (scope == DYNAMIC_SETTING_SCOPE_SHARED) {
        atomic_set(&shared_save_pending, 1);
        wake_service();
    }
}

static int validate_shared_scope_save(enum dynamic_setting_scope scope, uint32_t group_id,
                                      uint32_t session_id, uint32_t sequence, void *context) {
    ARG_UNUSED(context);
    if (scope != DYNAMIC_SETTING_SCOPE_SHARED || !session_id || !sequence ||
        !dynamic_settings_find_group(group_id)) {
        return -EINVAL;
    }
    if (!dynamic_settings_scope_is_dirty(scope)) {
        return 0;
    }
    k_spinlock_key_t route_key = k_spin_lock(&route_lock);
    bool connected = route_valid;
    k_spin_unlock(&route_lock, route_key);

    bool has_confirmed_preview = false;
    k_mutex_lock(&store_lock, K_FOREVER);
    if (session_id != local_shared_session ||
        !remote_sequence_newer(sequence, local_shared_sequence)) {
        k_mutex_unlock(&store_lock);
        return -ESTALE;
    }
    for (size_t i = 0; i < dynamic_settings_shared_group_count(); i++) {
        const struct dynamic_setting_group_descriptor *group = dynamic_settings_shared_group_at(i);
        if (group && dynamic_settings_is_dirty(group->id) &&
            dynamic_settings_is_shared_preview(group->id)) {
            has_confirmed_preview = true;
            break;
        }
    }
    if (has_confirmed_preview &&
        (local_preview_session != session_id || local_preview_base[0] != shared_live.clock[0] ||
         local_preview_base[1] != shared_live.clock[1] || !shared_ready || !shared_synchronized ||
         atomic_get(&shared_save_pending) || shared_tx_active() || shared_rx_active())) {
        k_mutex_unlock(&store_lock);
        return -ESTALE;
    }
    if (!connected || !has_confirmed_preview) {
        local_shared_sequence = sequence;
        k_mutex_unlock(&store_lock);
        return 0;
    }
    k_mutex_unlock(&store_lock);

    struct dynamic_setting_remote_request request = {
        .operation = 7,
        .target = DYNAMIC_SETTING_SCOPE_SHARED,
        .session_id = session_id,
        .sequence = sequence,
        .group_id = group_id,
    };
    struct dynamic_setting_remote_response response;
    return dynamic_settings_remote_call(&request, &response);
}

static int remote_settings_call(const struct dynamic_setting_remote_request *request,
                                struct dynamic_setting_remote_response *response, void *context) {
    ARG_UNUSED(context);
    k_spinlock_key_t route_key = k_spin_lock(&route_lock);
    bool connected = route_valid;
    k_spin_unlock(&route_lock, route_key);
    if (!request || !response ||
        (request->target != DYNAMIC_SETTING_SCOPE_SHARED &&
         !(request->target == DYNAMIC_SETTING_SCOPE_OWNER &&
           (SIDE == NEMU_LINK_STUDIO_DONGLE || SIDE == NEMU_LINK_STUDIO_KEYBOARD))) ||
        request->value_length > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX || !request->session_id ||
        !request->sequence) {
        return -EINVAL;
    }
#if IS_ENABLED(CONFIG_BT)
    if (SIDE == NEMU_LINK_STUDIO_KEYBOARD && request->target == DYNAMIC_SETTING_SCOPE_OWNER) {
        return nemu_link_settings_ble_relay_call(request, response);
    }
#else
    if (SIDE == NEMU_LINK_STUDIO_KEYBOARD && request->target == DYNAMIC_SETTING_SCOPE_OWNER) {
        return -EHOSTUNREACH;
    }
#endif
    if (!atomic_get(&loaded)) {
        return -EHOSTUNREACH;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (request->target == DYNAMIC_SETTING_SCOPE_OWNER) {
        k_mutex_lock(&owner_esb_call_lock, K_FOREVER);
        int err = nemu_link_settings_esb_relay_call(request, response);
        k_mutex_unlock(&owner_esb_call_lock);
        return err;
    }
#endif
    k_mutex_lock(&store_lock, K_FOREVER);
    if (request->target == DYNAMIC_SETTING_SCOPE_SHARED) {
        bool new_session = request->session_id != local_shared_session;
        if (!new_session && !remote_sequence_newer(request->sequence, local_shared_sequence)) {
            LOG_WRN("Settings local sequence rejected side=%u op=%u sid=%u seq=%u lastSeq=%u", SIDE,
                    request->operation, request->session_id, request->sequence,
                    local_shared_sequence);
            k_mutex_unlock(&store_lock);
            return -ESTALE;
        }
    }
    k_mutex_lock(&remote_lock, K_FOREVER);
    if (remote_inflight) {
        k_mutex_unlock(&remote_lock);
        k_mutex_unlock(&store_lock);
        return -EBUSY;
    }
    if (request->target == DYNAMIC_SETTING_SCOPE_SHARED) {
        if (request->session_id != local_shared_session) {
            bool has_confirmed_preview = false;
            for (size_t i = 0; i < dynamic_settings_shared_group_count(); i++) {
                const struct dynamic_setting_group_descriptor *group =
                    dynamic_settings_shared_group_at(i);
                if (group && dynamic_settings_is_dirty(group->id) &&
                    dynamic_settings_is_shared_preview(group->id)) {
                    has_confirmed_preview = true;
                    break;
                }
            }
            local_shared_session = request->session_id;
            local_shared_sequence = 0;
            if (has_confirmed_preview) {
                /* Keep an applied preview across Studio page/session reloads.
                 * Its base clock still guards SAVE against peer-side changes. */
                local_preview_session = request->session_id;
            } else {
                local_preview_session = 0;
                local_preview_base[0] = 0;
                local_preview_base[1] = 0;
            }
        }
        local_shared_sequence = request->sequence;
    }
    /* Track local-only shared requests too. SAVE validates its session and
     * sequence after an offline WRITE_TEMP has already been applied locally. */
    if (!connected) {
        k_mutex_unlock(&remote_lock);
        k_mutex_unlock(&store_lock);
        return -EHOSTUNREACH;
    }
    remote_request = *request;
    const struct dynamic_setting_group_descriptor *request_group =
        request->setting_id ? dynamic_settings_find_setting_group(request->setting_id)
                            : dynamic_settings_find_group(request->group_id);
    bool is_record_request =
        request_group && (request_group->flags & DYNAMIC_SETTING_GROUP_FLAG_RECORD);
    /* STATUS is read-only metadata. It can be served while the replicated
     * shared value image is still converging; only writes/resets/discards
     * need a synchronized clock snapshot. */
    remote_base_needed = request->target == DYNAMIC_SETTING_SCOPE_SHARED &&
                         (request->operation == 3 || request->operation == 4 ||
                          request->operation == 6 || is_record_request);
    remote_base_acked = !remote_base_needed;
    remote_base_sent = false;
    remote_shared_base[0] = shared_live.clock[0];
    remote_shared_base[1] = shared_live.clock[1];
    remote_transaction++;
    if (!remote_transaction) {
        remote_transaction = 1;
    }
    remote_inflight = true;
    remote_sent = false;
    remote_done = false;
    remote_response_received = false;
    remote_owner_schema_expected = false;
    remote_owner_schema_received = 0;
    memset(remote_owner_schema_hash, 0, sizeof(remote_owner_schema_hash));
    k_sem_reset(&remote_done_sem);
    k_mutex_unlock(&remote_lock);
    k_mutex_unlock(&store_lock);
    wake_service();

    if (k_sem_take(&remote_done_sem, K_SECONDS(10))) {
        k_mutex_lock(&remote_lock, K_FOREVER);
        remote_inflight = false;
        remote_sent = false;
        remote_base_sent = false;
        k_mutex_unlock(&remote_lock);
        return -ETIMEDOUT;
    }
    k_mutex_lock(&remote_lock, K_FOREVER);
    int err = remote_done ? remote_error : -ETIMEDOUT;
    if (err) {
        LOG_WRN("Settings relay failed side=%u op=%u target=%u sid=%u seq=%u err=%d ready=%u "
                "sync=%u save=%u rx=%u K=%llu D=%llu",
                SIDE, request->operation, request->target, request->session_id, request->sequence,
                err, shared_ready, shared_synchronized,
                (unsigned int)atomic_get(&shared_save_pending), shared_rx_active(),
                shared_live.clock[0], shared_live.clock[1]);
    }
    if (!err) {
        *response = remote_response;
        if (request->target == DYNAMIC_SETTING_SCOPE_SHARED) {
            const struct dynamic_setting_group_descriptor *group =
                request->setting_id ? dynamic_settings_find_setting_group(request->setting_id)
                                    : dynamic_settings_find_group(request->group_id);
            if (group) {
                if (request->operation == 3 || request->operation == 4) {
                    (void)dynamic_settings_mark_shared_preview(group->id, true);
                    local_preview_session = request->session_id;
                    local_preview_base[0] = remote_shared_base[0];
                    local_preview_base[1] = remote_shared_base[1];
                } else if (request->operation == 6) {
                    (void)dynamic_settings_mark_shared_preview(group->id, false);
                }
            }
        }
    }
    remote_inflight = false;
    remote_sent = false;
    remote_base_sent = false;
    remote_base_needed = false;
    remote_base_acked = false;
    remote_done = false;
    k_mutex_unlock(&remote_lock);
    return err;
}

static void encode_remote_request(uint8_t *bytes,
                                  const struct dynamic_setting_remote_request *request) {
    const struct dynamic_setting_group_descriptor *group =
        request->setting_id ? dynamic_settings_find_setting_group(request->setting_id)
                            : dynamic_settings_find_group(request->group_id);
    if (group && (group->flags & DYNAMIC_SETTING_GROUP_FLAG_RECORD) &&
        (request->operation == 2 || request->operation == 3)) {
        bytes[0] = request->operation;
        bytes[1] = request->target;
        bytes[2] = 0;
        bytes[3] = request->value_length;
        sys_put_le32(request->group_id, bytes + 4);
        sys_put_le16(request->offset, bytes + 8);
        sys_put_le16(request->total_length, bytes + 10);
        memcpy(bytes + 12, request->value, DYNAMIC_SETTINGS_REMOTE_VALUE_MAX);
        return;
    }
    bytes[0] = request->operation;
    bytes[1] = request->target;
    bytes[2] = request->value_type;
    bytes[3] = request->value_length;
    sys_put_le32(request->session_id, bytes + 4);
    sys_put_le32(request->sequence, bytes + 8);
    sys_put_le32(request->group_id, bytes + 12);
    sys_put_le32(request->setting_id, bytes + 16);
    memcpy(bytes + 20, request->value, DYNAMIC_SETTINGS_VALUE_MAX);
}

static void encode_remote_base(uint8_t *bytes, uint32_t session, uint32_t sequence,
                               const uint64_t clock[2]) {
    sys_put_le32(session, bytes);
    sys_put_le32(sequence, bytes + 4);
    sys_put_le64(clock[0], bytes + 8);
    sys_put_le64(clock[1], bytes + 16);
}

static bool remote_sequence_newer(uint32_t candidate, uint32_t current) {
    /* Compare request sequence numbers within one opaque session only. Session IDs
     * are tokens and may come from unrelated clocks or UI instances. */
    uint32_t distance = candidate - current;
    return distance && distance < 0x80000000u;
}

static struct nemu_link_studio_packet
remote_base_packet(const struct nemu_link_studio_packet *packet) {
    uint32_t session = sys_get_le32(packet->data);
    uint32_t sequence = sys_get_le32(packet->data + 4);
    uint64_t keyboard_clock = sys_get_le64(packet->data + 8);
    uint64_t dongle_clock = sys_get_le64(packet->data + 16);
    int err = 0;
    if (!packet->transaction || !session || !sequence) {
        err = -EINVAL;
    } else if (owner_base_valid && packet->transaction == owner_base_transaction &&
               !memcmp(packet->data, owner_base_request, sizeof(owner_base_request))) {
        err = 0;
    } else if (owner_base_valid) {
        err = -EBUSY;
    } else if ((owner_last_transaction &&
                !nemu_link_studio_transaction_newer(packet->transaction, owner_last_transaction)) ||
               (session == owner_last_session &&
                !remote_sequence_newer(sequence, owner_last_sequence))) {
        err = -ESTALE;
    } else if (!shared_ready || !shared_synchronized || atomic_get(&shared_save_pending) ||
               shared_rx_active() || keyboard_clock != shared_live.clock[0] ||
               dongle_clock != shared_live.clock[1]) {
        err = -ESTALE;
    } else {
        if (owner_last_session && session != owner_last_session) {
            owner_last_session = session;
            owner_last_sequence = 0;
        }
        owner_base_valid = true;
        owner_base_transaction = packet->transaction;
        memcpy(owner_base_request, packet->data, sizeof(owner_base_request));
        owner_base_clock[0] = keyboard_clock;
        owner_base_clock[1] = dongle_clock;
    }
    if (err == -ESTALE) {
        uint8_t reason = shared_stale_state_reason();
        atomic_set(&shared_last_stale_reason,
                   reason == SHARED_STALE_NONE ? SHARED_STALE_SESSION_SEQUENCE : reason);
        if (keyboard_clock != shared_live.clock[0] || dongle_clock != shared_live.clock[1]) {
            atomic_set(&shared_last_stale_reason, SHARED_STALE_REVISION_MISMATCH);
        }
        LOG_WRN("Settings base rejected side=%u txn=%u sid=%u seq=%u ready=%u sync=%u save=%u "
                "rx=%u reqK=%llu reqD=%llu liveK=%llu liveD=%llu lastTxn=%u lastSid=%u lastSeq=%u",
                SIDE, packet->transaction, session, sequence, shared_ready, shared_synchronized,
                (unsigned int)atomic_get(&shared_save_pending), shared_rx_active(), keyboard_clock,
                dongle_clock, shared_live.clock[0], shared_live.clock[1], owner_last_transaction,
                owner_last_session, owner_last_sequence);
    }
    uint8_t data[2];
    sys_put_le16((uint16_t)(int16_t)err, data);
    struct nemu_link_studio_packet reply = {
        .kind = NEMU_LINK_STUDIO_REMOTE_BASE_ACK,
        .transaction = packet->transaction,
        .length = sizeof(data),
    };
    memcpy(reply.data, data, sizeof(data));
    return reply;
}

static struct nemu_link_studio_packet
remote_request_packet(const struct nemu_link_studio_packet *packet) {
    bool record_request = packet->data[2] == 0 && (packet->data[0] == 2 || packet->data[0] == 3);
    uint32_t session =
        record_request ? sys_get_le32(owner_base_request) : sys_get_le32(packet->data + 4);
    uint32_t sequence =
        record_request ? sys_get_le32(owner_base_request + 4) : sys_get_le32(packet->data + 8);
    if (packet->transaction == owner_last_transaction &&
        !memcmp(packet->data, owner_last_request, sizeof(owner_last_request))) {
        return owner_last_response;
    }
    struct dynamic_setting_remote_request request = {
        .operation = packet->data[0],
        .target = packet->data[1],
        .value_type = packet->data[2],
        .value_length = packet->data[3],
        .session_id = session,
        .sequence = sequence,
        .group_id =
            record_request ? sys_get_le32(packet->data + 4) : sys_get_le32(packet->data + 12),
        .setting_id = record_request ? 0 : sys_get_le32(packet->data + 16),
        .offset = record_request ? sys_get_le16(packet->data + 8) : 0,
        .total_length = record_request ? sys_get_le16(packet->data + 10) : 0,
    };
    memcpy(request.value, packet->data + (record_request ? 12 : 20),
           record_request ? DYNAMIC_SETTINGS_REMOTE_VALUE_MAX : DYNAMIC_SETTINGS_VALUE_MAX);
    if (owner_base_valid && owner_base_transaction != packet->transaction) {
        uint8_t data[24] = {0};
        sys_put_le16((uint16_t)(int16_t)-EBUSY, data);
        struct nemu_link_studio_packet reply = {
            .kind = NEMU_LINK_STUDIO_REMOTE_RESPONSE,
            .transaction = packet->transaction,
            .length = record_request ? sizeof(data) : 16,
        };
        memcpy(reply.data, data, sizeof(data));
        return reply;
    }
    int err = 0;
    bool accepted = false;
    struct dynamic_setting_remote_response result = {0};
    if (owner_last_transaction &&
        !nemu_link_studio_transaction_newer(packet->transaction, owner_last_transaction)) {
        err = -ESTALE;
    } else if (!packet->transaction || !session || !sequence ||
               request.value_length > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX) {
        err = -EINVAL;
    } else if (request.target == DYNAMIC_SETTING_SCOPE_SHARED && request.operation == 7 &&
               (atomic_get(&shared_save_pending) || shared_rx_active())) {
        err = -ESTALE;
    } else if (request.target == DYNAMIC_SETTING_SCOPE_SHARED &&
               (request.operation == 3 || request.operation == 4 || request.operation == 6) &&
               (!owner_base_valid || owner_base_transaction != packet->transaction ||
                sys_get_le32(owner_base_request) != session ||
                sys_get_le32(owner_base_request + 4) != sequence || !shared_synchronized ||
                atomic_get(&shared_save_pending) || shared_rx_active() ||
                owner_base_clock[0] != shared_live.clock[0] ||
                owner_base_clock[1] != shared_live.clock[1])) {
        err = -ESTALE;
    } else if (session == owner_last_session &&
               !remote_sequence_newer(sequence, owner_last_sequence)) {
        err = -ESTALE;
    } else {
        if (owner_base_valid && owner_base_transaction != packet->transaction) {
            owner_base_valid = false;
        }
        if (session != owner_last_session) {
            owner_last_session = session;
            owner_last_sequence = 0;
        }
        err = nemu_link_settings_relay_dispatch(&request, &result);
        accepted = !err;
    }
    bool record_response = record_request;
    uint8_t data[24] = {0};
    sys_put_le16((uint16_t)(int16_t)err, data);
    data[2] = result.value_type;
    data[3] = result.value_length;
    data[4] = result.origin;
    data[5] = result.dirty;
    data[6] = result.persisted;
    data[7] = request.target == DYNAMIC_SETTING_SCOPE_OWNER && result.schema_known;
    sys_put_le32(result.generation, data + 8);
    if (record_response) {
        sys_put_le16(result.total_length, data + 12);
        memcpy(data + 14, result.value, 10);
    } else {
        memcpy(data + 12, result.value, DYNAMIC_SETTINGS_VALUE_MAX);
    }
    struct nemu_link_studio_packet reply = {
        .kind = NEMU_LINK_STUDIO_REMOTE_RESPONSE,
        .transaction = packet->transaction,
        .length = record_response ? sizeof(data) : 16,
    };
    memcpy(reply.data, data, sizeof(data));
    if (data[7]) {
        owner_schema_reply[0] = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH,
            .transaction = packet->transaction,
            .offset = 0,
            .length = 24,
        };
        memcpy(owner_schema_reply[0].data, result.schema_sha256, 24);
        owner_schema_reply[1] = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH,
            .transaction = packet->transaction,
            .offset = 24,
            .length = 8,
        };
        memcpy(owner_schema_reply[1].data, result.schema_sha256 + 24, 8);
        owner_schema_reply_pending = BIT(0) | BIT(1);
        owner_schema_reply_next = 0;
    }
    if (accepted) {
        owner_last_session = session;
        owner_last_sequence = sequence;
        if (request.target == DYNAMIC_SETTING_SCOPE_SHARED) {
            const struct dynamic_setting_group_descriptor *group =
                request.setting_id ? dynamic_settings_find_setting_group(request.setting_id)
                                   : dynamic_settings_find_group(request.group_id);
            if (group) {
                if (request.operation == 3 || request.operation == 4) {
                    (void)dynamic_settings_mark_shared_preview(group->id, true);
                } else if (request.operation == 6) {
                    (void)dynamic_settings_mark_shared_preview(group->id, false);
                }
            }
        }
    }
    if (err == -ESTALE) {
        uint8_t reason = shared_stale_state_reason();
        atomic_set(&shared_last_stale_reason,
                   reason == SHARED_STALE_NONE ? SHARED_STALE_SESSION_SEQUENCE : reason);
        LOG_WRN("Settings request rejected side=%u txn=%u op=%u target=%u sid=%u seq=%u ready=%u "
                "sync=%u save=%u rx=%u base=%u baseTxn=%u lastTxn=%u lastSid=%u lastSeq=%u",
                SIDE, packet->transaction, request.operation, request.target, session, sequence,
                shared_ready, shared_synchronized, (unsigned int)atomic_get(&shared_save_pending),
                shared_rx_active(), owner_base_valid, owner_base_transaction,
                owner_last_transaction, owner_last_session, owner_last_sequence);
    }
    if (owner_base_valid && owner_base_transaction == packet->transaction) {
        owner_base_valid = false;
    }
    if (nemu_link_studio_transaction_newer(packet->transaction, owner_last_transaction) ||
        !owner_last_transaction) {
        owner_last_transaction = packet->transaction;
        memcpy(owner_last_request, packet->data, sizeof(owner_last_request));
        owner_last_response = reply;
    }
    return reply;
}
#endif
static void set_busy(bool busy) {
    /* Pair with editor wrappers: no editor can pass its guard during capture. */
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    atomic_set(&editing_busy, busy);
    k_mutex_unlock(&nemu_link_studio_map_lock);
}
static int describe(struct nemu_link_studio_version *v) {
    uint8_t record[NEMU_LINK_STUDIO_RECORD_MAX];
    uint32_t checksum = 0, size = 0;
    for (uint16_t i = 0; i < nemu_link_studio_record_count(nemu_link_studio_schema()); i++) {
        size_t n;
        int err = nemu_link_studio_record_read(i, false, record, &n);
        if (err) {
            return err;
        }
        checksum = nemu_link_studio_checksum(checksum, size, record, n);
        size += n;
    }
    if (!size || size > UINT16_MAX) {
        return -E2BIG;
    }
    nemu_link_studio_checksum_bytes(checksum, v->checksum);
    v->size = size;
    return 0;
}
static int persist_version(const struct nemu_link_studio_version *v) {
    uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (!nemu_link_studio_version_encode(bytes, v)) {
        return -EINVAL;
    }
    return settings_save_one("m6/version", bytes, sizeof(bytes));
}
static void abort_transfer(void) {
    delta_valid = false;
    waiting_for_keyboard = false;
    if (tx_active()) {
        tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
    }
    rx.expired = true;
    rx.started = false;
    response_pending = false;
    /* Direct receive writes already applied remain the current keymap. */
    if (!nemu_link_studio_native_dirty()) {
        (void)describe(&live);
    }
    next_offer = 0;
}
int nemu_link_studio_select_layout(uint8_t index) {
    k_mutex_lock(&store_lock, K_FOREVER);
    if (!ready || !nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&store_lock);
        return -EBUSY;
    }
    set_busy(true);
    int err = zmk_physical_layouts_select(index);
    set_busy(false);
    k_mutex_unlock(&store_lock);
    return err;
}
int nemu_link_studio_dirty(void) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    int ret = nemu_link_studio_native_dirty();
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int nemu_link_studio_save(bool stock) {
    k_mutex_lock(&store_lock, K_FOREVER);
    set_busy(true);
    if (!ready) {
        set_busy(false);
        k_mutex_unlock(&store_lock);
        return -EAGAIN;
    }
    bool capture_delta =
        !stock && peer_schema_matches && atomic_get(&synchronized) && !tx_active() && !rx_active();
    struct nemu_link_studio_version base = live;
    memset(delta_records, 0, sizeof(delta_records));
    if (capture_delta) {
        for (uint16_t i = 0; i < nemu_link_studio_record_count(nemu_link_studio_schema()); i++) {
            if (nemu_link_studio_record_changed(i)) {
                delta_records[i / 8] |= (uint8_t)(1u << (i % 8));
            }
        }
    }
    abort_transfer();
    int err = 0;
    if (live.clock[SIDE] == UINT64_MAX) {
        err = -EOVERFLOW;
    }
    if (!err && stock) {
        /* Discard pending edits first; then compare stock against persisted RAM. */
        err = nemu_link_studio_native_discard();
        for (uint16_t i = 0; !err && i < nemu_link_studio_record_count(nemu_link_studio_schema());
             i++) {
            uint8_t record[NEMU_LINK_STUDIO_RECORD_MAX];
            size_t n;
            err = nemu_link_studio_record_read(i, true, record, &n);
            if (!err) {
                err = nemu_link_studio_record_write(i, record, n);
            }
        }
    } else if (!err) {
        err = nemu_link_studio_native_save();
    }
    struct nemu_link_studio_version next = live;
    if (!err) {
        err = describe(&next);
    }
    if (!err) {
        next.clock[SIDE]++;
        err = persist_version(&next);
    }
    if (!err) {
        live = next;
        delta_base = base;
        delta_valid = capture_delta;
        delta_size = 0;
        for (uint16_t i = 0;
             delta_valid && i < nemu_link_studio_record_count(nemu_link_studio_schema()); i++) {
            if (delta_selected(i)) {
                uint8_t record[NEMU_LINK_STUDIO_RECORD_MAX];
                size_t n;
                if (nemu_link_studio_record_read(i, false, record, &n) ||
                    delta_size + n + 3 > UINT16_MAX) {
                    delta_valid = false;
                } else {
                    delta_size = (uint16_t)(delta_size + n + 3);
                }
            }
        }
        delta_valid = delta_valid && delta_size && delta_size < live.size;
        save_count++;
        atomic_set(&synchronized, 0);
        next_offer = 0;
        atomic_set(&discovery_pending, 1);
        wake_service();
        LOG_INF("Studio saved K=%llu D=%llu checksum=%02x%02x%02x%02x saves=%u", live.clock[0],
                live.clock[1], live.checksum[3], live.checksum[2], live.checksum[1],
                live.checksum[0], save_count);
    } else {
        LOG_ERR("Studio save failed %d", err);
    }
    set_busy(false);
    k_mutex_unlock(&store_lock);
    return err;
}
int nemu_link_studio_discard(void) {
    k_mutex_lock(&store_lock, K_FOREVER);
    set_busy(true);
    abort_transfer();
    int err = nemu_link_studio_native_discard();
    if (!err) {
        err = describe(&live);
    }
    set_busy(false);
    k_mutex_unlock(&store_lock);
    wake_service();
    return err;
}
/* TX caches each proposal until its matching ACK, so this reader only moves
 * forwards. Each record is fetched once even if it spans several chunks. */
static bool read_image(void *unused, uint32_t offset, uint8_t *bytes, size_t size) {
    ARG_UNUSED(unused);
    if (offset != reader.offset) {
        return false;
    }
    while (size) {
        if (reader.used == reader.size) {
            uint16_t count = nemu_link_studio_record_count(nemu_link_studio_schema());
            while (reader.index < count && reader.delta && !delta_selected(reader.index)) {
                reader.index++;
            }
            if (reader.index >= count) {
                return false;
            }
            size_t n;
            unsigned int prefix = reader.delta ? 3 : 0;
            if (nemu_link_studio_record_read(reader.index, false, reader.record + prefix, &n)) {
                return false;
            }
            if (reader.delta) {
                reader.record[0] = (uint8_t)reader.index;
                reader.record[1] = (uint8_t)(reader.index >> 8);
                reader.record[2] = (uint8_t)n;
            }
            reader.index++;
            reader.used = 0;
            reader.size = n + prefix;
        }
        size_t count = MIN(size, reader.size - reader.used);
        memcpy(bytes, reader.record + reader.used, count);
        reader.used += count;
        reader.offset += count;
        bytes += count;
        size -= count;
    }
    return true;
}
static bool apply_record(void *unused, uint16_t index, const uint8_t *bytes, size_t size) {
    ARG_UNUSED(unused);
    int err = nemu_link_studio_record_write(index, bytes, size);
    if (!err) {
        uint8_t actual[NEMU_LINK_STUDIO_RECORD_MAX];
        size_t n;
        err = nemu_link_studio_record_read(index, false, actual, &n);
        if (!err && (n != size || memcmp(actual, bytes, size))) {
            err = -EIO;
        }
    }
    if (err) {
        last_keymap_receive_error_code = (uint32_t)-err;
        last_keymap_receive_error_index = index;
        LOG_ERR("Studio record %u write failed %d", index, err);
    }
    return !err;
}
static bool commit_received(void *unused, struct nemu_link_studio_version *v) {
    ARG_UNUSED(unused);
    /* Re-read one record at a time: confirm the RAM representation actually
     * matches the received checksum before acknowledging its revision. */
    struct nemu_link_studio_version actual = *v;
    int err = describe(&actual);
    if (!err && !memcmp(live.schema, v->schema, 32) &&
        (actual.size != v->size || memcmp(actual.checksum, v->checksum, 32))) {
        err = -EINVAL;
    }
    /* Wire XOR checksum was checked over ALL records. Each kept record was read back;
     * retain our build identity and checksum of our possibly larger/smaller map. */
    memcpy(actual.schema, live.schema, 32);
    if (!err) {
        err = persist_version(&actual);
    }
    if (!err) {
        *v = actual;
    }
    if (err) {
        last_keymap_receive_error_code = (uint32_t)-err;
        last_keymap_receive_error_index = UINT32_MAX;
        LOG_ERR("Studio receive commit failed %d", err);
        return false;
    }
    delta_valid = false;
    applied_count++;
    LOG_INF("Studio applied K=%llu D=%llu commits=%u", v->clock[0], v->clock[1], applied_count);
    next_offer = 0;
    return true;
}
static void consider_offer(void) {
    peer_schema_matches = !memcmp(live.schema, rx.target.schema, 32);
    enum nemu_link_studio_plan plan = nemu_link_studio_plan(SIDE, &live, &rx.target);
    last_keymap_offer_decision = (uint8_t)plan + 1;

    /* A dirty preview is not a saved generation. Once saved revisions are
     * compared, discard the preview if either side has the winning saved map,
     * so the winning map can actually be received or sent. On equal/diverged
     * generations the dongle yields to the keyboard's saved map. */
    bool dirty = nemu_link_studio_dirty();
    bool discard_preview =
        dirty && (plan == NEMU_LINK_STUDIO_SEND || plan == NEMU_LINK_STUDIO_RECEIVE ||
                  (SIDE == NEMU_LINK_STUDIO_DONGLE &&
                   (plan == NEMU_LINK_STUDIO_EQUAL || plan == NEMU_LINK_STUDIO_DIVERGED ||
                    plan == NEMU_LINK_STUDIO_WAIT_KEYBOARD)));
    if (discard_preview) {
        int err = nemu_link_studio_native_discard();
        if (err) {
            atomic_set(&synchronized, 0);
            next_offer = 0;
            LOG_ERR("Studio discard preview before keymap arbitration failed %d", err);
            return;
        }
    }
    atomic_set(&synchronized, plan == NEMU_LINK_STUDIO_EQUAL);
    if (plan == NEMU_LINK_STUDIO_EQUAL) {
        /* Equal persisted generations are already converged. Do not start a
         * redundant full image just because a session reset left an offer due. */
        if (!nemu_link_studio_dirty()) {
            next_offer = UINT64_MAX;
        } else {
            /* Keyboard-side preview wins the equal-generation tie, but remains
             * unsaved until Studio explicitly saves it. */
            atomic_set(&synchronized, 0);
            next_offer = 0;
        }
    } else if (plan != NEMU_LINK_STUDIO_WAIT_KEYBOARD && plan != NEMU_LINK_STUDIO_DIVERGED) {
        waiting_for_keyboard = false;
    }
    if ((plan == NEMU_LINK_STUDIO_RESOLVE_KEYBOARD || plan == NEMU_LINK_STUDIO_DIVERGED) &&
        SIDE == NEMU_LINK_STUDIO_KEYBOARD) {
        struct nemu_link_studio_version next = live;
        uint64_t clock = MAX(live.clock[0], rx.target.clock[0]);
        if (clock == UINT64_MAX) {
            return;
        }
        next.clock[0] = clock + 1;
        next.clock[1] = MAX(live.clock[1], rx.target.clock[1]);
        if (!persist_version(&next)) {
            LOG_WRN("Studio conflict: keyboard wins K=%llu D=%llu", next.clock[0], next.clock[1]);
            delta_valid = false;
            live = next;
            tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
            next_offer = 0;
        }
    } else if (plan == NEMU_LINK_STUDIO_RECEIVE) {
        delta_valid = false;
        next_offer = UINT64_MAX;
        /* The outgoing reader must never serve a map being changed by RX. */
        if (tx_active()) {
            tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
        }
    } else if (plan == NEMU_LINK_STUDIO_SEND && !tx_active()) {
        next_offer = 0;
    } else if (plan == NEMU_LINK_STUDIO_WAIT_KEYBOARD ||
               (plan == NEMU_LINK_STUDIO_DIVERGED && SIDE == NEMU_LINK_STUDIO_DONGLE)) {
        /* The keyboard can arbitrate only after receiving the dongle's revision.
         * Reject the conflicting keyboard offer, then advertise our version. */
        if (tx_active()) {
            tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
            tx.error = NEMU_LINK_STUDIO_OK;
        }
        waiting_for_keyboard = true;
        next_offer = 0;
    } else if (plan == NEMU_LINK_STUDIO_INCOMPATIBLE) {
        waiting_for_keyboard = false;
        LOG_WRN("Studio incompatible peer schema");
        LOG_HEXDUMP_WRN(live.schema, sizeof(live.schema), "local schema");
        LOG_HEXDUMP_WRN(rx.target.schema, sizeof(rx.target.schema), "peer schema");
    }
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static void consider_shared_offer(void) {
    enum nemu_link_studio_plan plan =
        !memcmp(shared_live.schema, dynamic_settings_shared_contract_sha256,
                sizeof(shared_live.schema)) &&
                !memcmp(shared_rx.target.schema, dynamic_settings_shared_contract_sha256,
                        sizeof(shared_rx.target.schema))
            ? nemu_link_studio_plan(SIDE, &shared_live, &shared_rx.target)
            : NEMU_LINK_STUDIO_INCOMPATIBLE;
    last_shared_offer_decision = (uint8_t)plan + 1;
    shared_synchronized = plan == NEMU_LINK_STUDIO_EQUAL;
    if (plan == NEMU_LINK_STUDIO_EQUAL) {
        shared_next_offer = UINT64_MAX;
    } else if ((plan == NEMU_LINK_STUDIO_RESOLVE_KEYBOARD || plan == NEMU_LINK_STUDIO_DIVERGED) &&
               SIDE == NEMU_LINK_STUDIO_KEYBOARD) {
        struct nemu_link_studio_version next = shared_live;
        uint64_t clock = MAX(shared_live.clock[0], shared_rx.target.clock[0]);
        if (clock == UINT64_MAX) {
            shared_next_offer = UINT64_MAX;
            return;
        }
        next.clock[0] = clock + 1;
        next.clock[1] = MAX(shared_live.clock[1], shared_rx.target.clock[1]);
        if (!persist_shared_version(&next)) {
            LOG_WRN("Shared settings conflict: keyboard wins K=%llu D=%llu", next.clock[0],
                    next.clock[1]);
            shared_live = next;
            shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
            shared_tx.error = NEMU_LINK_STUDIO_OK;
            shared_next_offer = 0;
        }
    } else if (plan == NEMU_LINK_STUDIO_RECEIVE) {
        if (shared_tx_active()) {
            shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
            shared_tx.error = NEMU_LINK_STUDIO_OK;
        }
        shared_next_offer = UINT64_MAX;
    } else if (plan == NEMU_LINK_STUDIO_SEND && !shared_tx_active()) {
        shared_next_offer = 0;
    } else if (plan == NEMU_LINK_STUDIO_INCOMPATIBLE) {
        LOG_WRN("Shared settings execution contract mismatch; sync held");
        shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
        shared_tx.error = NEMU_LINK_STUDIO_INVALID;
        shared_next_offer = UINT64_MAX;
    } else if (plan == NEMU_LINK_STUDIO_WAIT_KEYBOARD) {
        shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
        shared_tx.error = NEMU_LINK_STUDIO_OK;
        /* The keyboard needs this revision to create a dominating version. */
        shared_next_offer = 0;
    }
}
#endif
void nemu_link_studio_link_state(bool up, const struct nemu_link_studio_route *route) {
    k_spinlock_key_t key = k_spin_lock(&route_lock);
    bool valid = up && route;
    bool changed =
        valid != route_valid || (valid && (!same_session(&link_route, route) ||
                                           link_route.channel_epoch != route->channel_epoch ||
                                           link_route.channel_index != route->channel_index));
    route_valid = valid;
    /* A hop/search pause does not undo an acknowledged saved revision.
     * Completed transfers have next_offer=UINT64_MAX, so clearing this flag
     * here would leave manual BLE return waiting forever in the same session.
     * Save and a changed peer session invalidate it separately. */
    if (route_valid) {
        if (link_route.dongle_id != route->dongle_id || link_route.boot != route->boot ||
            link_route.session != route->session || link_route.nonce != route->nonce) {
            atomic_set(&synchronized, 0);
            atomic_set(&remote_notice, 0);
        }
        link_route = *route;
    }
    k_spin_unlock(&route_lock, key);
    if (changed) {
        wake_service();
    }
}
void nemu_link_studio_receive(const uint8_t *bytes, size_t size, uint64_t received_ms) {
    /* POLL only clocks ACK delivery. Keep it out of the coordinator queue
     * so 200 Hz empty requests cannot starve DATA_ACK during slow map reads.
     * Link has authenticated the frame; validate its route before consuming. */
    struct nemu_link_studio_route route;
    k_spinlock_key_t key = k_spin_lock(&route_lock);
    bool valid = route_valid;
    route = link_route;
    k_spin_unlock(&route_lock, key);
    struct nemu_link_studio_packet p;
    if (valid && bytes && size > 8 && bytes[8] == NEMU_LINK_STUDIO_POLL &&
        nemu_link_studio_frame_decode(&p, &route, route.source, bytes, size, FRAME_CRC) &&
        p.kind == NEMU_LINK_STUDIO_POLL) {
        return;
    }
    struct rx_item item = {.time = received_ms, .size = size};
    memcpy(item.bytes, bytes, size);
    atomic_inc(&received_frames);
    if (k_msgq_put(&control_rx, &item, K_NO_WAIT)) {
        atomic_inc(&dropped_frames);
    } else {
        /* Pending normal traffic must block scanning before the worker runs. */
        atomic_set(&radio_busy, 1);
        k_work_reschedule_for_queue(&worker, &sync_work, K_NO_WAIT);
    }
}
static bool same_session(const struct nemu_link_studio_route *a,
                         const struct nemu_link_studio_route *b) {
    return a->dongle_id == b->dongle_id && a->boot == b->boot && a->session == b->session &&
           a->nonce == b->nonce;
}
static void service(struct k_work *work) {
    ARG_UNUSED(work);
    struct nemu_link_studio_route route;
    k_spinlock_key_t key = k_spin_lock(&route_lock);
    bool valid = route_valid;
    route = link_route;
    k_spin_unlock(&route_lock, key);
    struct nemu_link_studio_packet outgoing = {0};
    enum {
        SEND_NONE,
        SEND_RESPONSE,
        SEND_KEYMAP_TX,
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
        SEND_SHARED_TX,
        SEND_REMOTE_TX,
        SEND_REMOTE_BASE,
        SEND_SCHEMA,
        SEND_OWNER_SCHEMA,
#endif
        SEND_POLL,
    } send_action = SEND_NONE;
    uint64_t delay_ms = UINT64_MAX;
    uint64_t now = k_uptime_get();
    k_mutex_lock(&store_lock, K_FOREVER);
    if (!ready && atomic_get(&loaded)) {
        set_busy(true);
        uint8_t schema[32];
        nemu_link_studio_schema_digest(schema);
        if (have_metadata && memcmp(schema, live.schema, 32)) {
            LOG_WRN("Studio schema changed; preserving clocks, using loaded keymap");
        }
        memcpy(live.schema, schema, 32);
        if (!describe(&live)) {
            ready = true;
            LOG_INF("Studio ready side=%u K=%llu D=%llu bytes=%u record_buffer=%u Studio=unlocked",
                    SIDE, live.clock[0], live.clock[1], live.size, NEMU_LINK_STUDIO_RECORD_MAX);
        }
        set_busy(false);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    if (!shared_ready && atomic_get(&loaded)) {
        struct nemu_link_studio_version current = shared_live;
        bool contract_changed =
            have_shared_metadata &&
            memcmp(current.schema, dynamic_settings_shared_contract_sha256, sizeof(current.schema));
        memcpy(current.schema, dynamic_settings_shared_contract_sha256, sizeof(current.schema));
        int err = describe_shared(&current);
        if (!err) {
            bool content_changed = !have_shared_metadata || contract_changed ||
                                   current.size != shared_live.size ||
                                   memcmp(current.checksum, shared_live.checksum, 32);
            if (content_changed) {
                if (shared_live.clock[SIDE] == UINT64_MAX) {
                    err = -EOVERFLOW;
                } else {
                    current.clock[SIDE] = shared_live.clock[SIDE] + 1;
                    err = persist_shared_version(&current);
                }
            }
            if (!err) {
                shared_live = current;
                shared_ready = true;
                shared_next_offer = 0;
                shared_generation = dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED);
                if (contract_changed) {
                    LOG_WRN("Shared settings contract changed; incompatible groups use defaults");
                }
            }
        } else if (err == -ENOENT) {
            shared_ready = true;
            shared_next_offer = UINT64_MAX;
        }
    }
    if (shared_ready && atomic_cas(&shared_save_pending, 1, 0)) {
        if (shared_tx_active()) {
            shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
        }
        shared_rx.expired = true;
        shared_rx.started = false;
        struct nemu_link_studio_version next = shared_live;
        int err = shared_live.clock[SIDE] == UINT64_MAX ? -EOVERFLOW : 0;
        if (!err) {
            err = describe_shared(&next);
        }
        if (!err) {
            next.clock[SIDE]++;
            err = persist_shared_version(&next);
        }
        if (!err) {
            shared_live = next;
            shared_synchronized = false;
            shared_next_offer = 0;
            shared_generation = dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED);
            LOG_INF("Shared settings saved K=%llu D=%llu", next.clock[0], next.clock[1]);
        } else {
            LOG_ERR("Shared settings version save failed %d", err);
            shared_synchronized = false;
            shared_next_offer = now + 1000;
        }
    }
#endif
    if (!atomic_get(&loaded)) {
        goto done;
    }
    if ((rx_active() && now - rx.last_progress >= 10000) ||
        (tx_active() && now - tx.last_progress >= 10000)) {
        abort_transfer();
        set_busy(false);
        atomic_set(&radio_busy, 0);
        next_offer = now + 1000;
        LOG_WRN("Studio retry after timeout; input continues");
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    if ((shared_rx_active() &&
         (now < shared_rx.last_progress || now - shared_rx.last_progress >= 10000)) ||
        (shared_tx_active() &&
         (now < shared_tx.last_progress || now - shared_tx.last_progress >= 10000))) {
        shared_rx.expired = true;
        shared_rx.started = false;
        if (shared_tx_active()) {
            shared_tx.phase = NEMU_LINK_STUDIO_TX_FAILED;
        }
        shared_next_offer = now + 1000;
        LOG_WRN("Shared settings retry after timeout");
    }
#endif
    if (!valid) {
        goto done;
    }
    if (!session_valid || !same_session(&route, &session_route)) {
        abort_transfer();
        tx = (struct nemu_link_studio_tx){0};
        rx = (struct nemu_link_studio_stream_rx){0};
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
        if (shared_ready) {
            discard_shared_edits();
        }
        local_preview_session = 0;
        local_preview_base[0] = 0;
        local_preview_base[1] = 0;
        shared_tx = (struct nemu_link_studio_tx){0};
        (void)nemu_link_studio_group_rx_init(&shared_rx, dynamic_settings_storage_scratch,
                                             dynamic_settings_storage_scratch_size);
        shared_synchronized = false;
        shared_next_offer = 0;
#endif
        peer_schema_matches = false;
        session_route = route;
        session_valid = true;
        next_offer = 0;
        poll_until = 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
        owner_last_session = 0;
        owner_last_sequence = 0;
        owner_last_transaction = 0;
        owner_base_valid = false;
        owner_schema_reply_pending = 0;
        owner_schema_reply_next = 0;
        k_mutex_lock(&remote_lock, K_FOREVER);
        remote_transaction = 0;
        if (remote_inflight && !remote_done) {
            remote_error = -ESTALE;
            remote_done = true;
            k_sem_give(&remote_done_sem);
        }
        remote_sent = false;
        remote_base_sent = false;
        remote_base_acked = false;
        remote_response_received = false;
        remote_owner_schema_expected = false;
        remote_owner_schema_received = 0;
        memset(remote_owner_schema_hash, 0, sizeof(remote_owner_schema_hash));
        k_mutex_unlock(&remote_lock);
        memset(peer_shared_contract_hash, 0, sizeof(peer_shared_contract_hash));
        peer_shared_contract_received = 0;
        local_shared_contract_sent = 0;
        peer_shared_contract_known = false;
        peer_shared_contract_matches = false;
#endif
        set_busy(false);
        LOG_INF("Studio session %u", route.session);
    }
    if (atomic_set(&remote_notice, 0)) {
        poll_until = now + 200;
    }
    /* Acquire editor exclusion before inspecting its dirty bits. Input uses a
     * separate short map lock and is never disabled by this exclusion. */
    set_busy(true);
    bool dirty = nemu_link_studio_dirty();
    struct rx_item item;
    if (!response_pending && !k_msgq_get(&control_rx, &item, K_NO_WAIT)) {
        struct nemu_link_studio_packet p;
        if (now - item.time < 2000 &&
            nemu_link_studio_frame_decode(&p, &route, route.source, item.bytes, item.size,
                                          FRAME_CRC) &&
            (ready || shared_control_packet(&p))) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
            if (p.kind == NEMU_LINK_STUDIO_REMOTE_REQUEST) {
                response = remote_request_packet(&p);
                response_pending = true;
            } else if (p.kind == NEMU_LINK_STUDIO_REMOTE_BASE) {
                response = remote_base_packet(&p);
                response_pending = true;
            } else if (p.kind == NEMU_LINK_STUDIO_REMOTE_BASE_ACK) {
                k_mutex_lock(&remote_lock, K_FOREVER);
                if (remote_inflight && p.transaction == remote_transaction && !remote_done &&
                    remote_base_needed && p.length == 2) {
                    int error = (int16_t)sys_get_le16(p.data);
                    if (error) {
                        if (error == -ESTALE) {
                            atomic_set(&shared_last_stale_reason, SHARED_STALE_REMOTE_REJECTED);
                        }
                        remote_error = error;
                        remote_done = true;
                        remote_sent = false;
                        remote_base_sent = false;
                        k_sem_give(&remote_done_sem);
                    } else {
                        remote_base_acked = true;
                        remote_base_sent = false;
                        remote_sent = false;
                    }
                }
                k_mutex_unlock(&remote_lock);
            } else if (p.kind == NEMU_LINK_STUDIO_REMOTE_RESPONSE) {
                k_mutex_lock(&remote_lock, K_FOREVER);
                if (remote_inflight && p.transaction == remote_transaction && !remote_done) {
                    remote_error = (int16_t)sys_get_le16(p.data);
                    bool record_response =
                        remote_request.setting_id == 0 &&
                        (remote_request.operation == 2 || remote_request.operation == 3);
                    if (remote_error == -ESTALE) {
                        atomic_set(&shared_last_stale_reason, SHARED_STALE_REMOTE_REJECTED);
                    }
                    if (p.length != (record_response ? 24 : 16) ||
                        p.data[3] > (record_response ? 10 : DYNAMIC_SETTINGS_VALUE_MAX) ||
                        p.data[4] > DYNAMIC_SETTING_ORIGIN_TEMP) {
                        remote_error = -EINVAL;
                    } else {
                        remote_response = (struct dynamic_setting_remote_response){
                            .value_type = p.data[2],
                            .value_length = p.data[3],
                            .origin = p.data[4],
                            .dirty = p.data[5],
                            .persisted = p.data[6],
                            .schema_known = peer_shared_contract_known &&
                                            remote_request.target == DYNAMIC_SETTING_SCOPE_SHARED,
                            .generation = sys_get_le32(p.data + 8),
                            .total_length = record_response ? sys_get_le16(p.data + 12) : 0,
                        };
                        if (remote_response.schema_known) {
                            memcpy(remote_response.schema_sha256, peer_shared_contract_hash,
                                   sizeof(remote_response.schema_sha256));
                        }
                        memcpy(remote_response.value, p.data + (record_response ? 14 : 12),
                               remote_response.value_length);
                        remote_owner_schema_expected =
                            remote_request.target == DYNAMIC_SETTING_SCOPE_OWNER && p.data[7];
                    }
                    remote_response_received = true;
                    remote_sent = false;
                    if (remote_error || !remote_owner_schema_expected ||
                        remote_owner_schema_received == (BIT(0) | BIT(1))) {
                        if (!remote_error && remote_owner_schema_expected) {
                            remote_response.schema_known = true;
                            memcpy(remote_response.schema_sha256, remote_owner_schema_hash,
                                   sizeof(remote_response.schema_sha256));
                        }
                        remote_done = true;
                        k_sem_give(&remote_done_sem);
                    }
                }
                k_mutex_unlock(&remote_lock);
            } else if (p.kind == NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH) {
                k_mutex_lock(&remote_lock, K_FOREVER);
                if (remote_inflight && p.transaction == remote_transaction && !remote_done &&
                    remote_request.target == DYNAMIC_SETTING_SCOPE_OWNER) {
                    uint8_t part = p.offset ? 1 : 0;
                    uint8_t bit = BIT(part);
                    uint8_t *destination = remote_owner_schema_hash + p.offset;
                    if ((remote_owner_schema_received & bit) &&
                        memcmp(destination, p.data, p.length)) {
                        remote_owner_schema_received = 0;
                    }
                    memcpy(destination, p.data, p.length);
                    remote_owner_schema_received |= bit;
                    if (remote_response_received && remote_owner_schema_expected &&
                        remote_owner_schema_received == (BIT(0) | BIT(1))) {
                        remote_response.schema_known = true;
                        memcpy(remote_response.schema_sha256, remote_owner_schema_hash,
                               sizeof(remote_response.schema_sha256));
                        remote_done = true;
                        k_sem_give(&remote_done_sem);
                    }
                }
                k_mutex_unlock(&remote_lock);
            } else if (p.kind == NEMU_LINK_STUDIO_SCHEMA_HASH) {
                uint8_t part = p.offset ? 1 : 0;
                uint8_t bit = BIT(part);
                uint8_t *destination = peer_shared_contract_hash + p.offset;
                if ((peer_shared_contract_received & bit) &&
                    memcmp(destination, p.data, p.length)) {
                    peer_shared_contract_received = 0;
                    peer_shared_contract_known = false;
                }
                memcpy(destination, p.data, p.length);
                peer_shared_contract_received |= bit;
                if (peer_shared_contract_received == (BIT(0) | BIT(1))) {
                    peer_shared_contract_known = true;
                    peer_shared_contract_matches = !memcmp(
                        peer_shared_contract_hash, dynamic_settings_shared_contract_sha256, 32);
                    if (peer_shared_contract_matches) {
                        LOG_INF("Peer shared settings contract matches");
                    } else {
                        LOG_WRN("Peer shared settings contract differs; remote operations remain "
                                "enabled");
                    }
                }
            } else if (p.kind == NEMU_LINK_STUDIO_GROUP_META ||
                       p.kind == NEMU_LINK_STUDIO_GROUP_DATA ||
                       p.kind == NEMU_LINK_STUDIO_GROUP_COMMIT) {
                if (atomic_get(&shared_save_pending) ||
                    dynamic_settings_generation(DYNAMIC_SETTING_SCOPE_SHARED) !=
                        shared_generation) {
                    atomic_set(&shared_last_stale_reason, atomic_get(&shared_save_pending)
                                                              ? SHARED_STALE_SAVE_PENDING
                                                              : SHARED_STALE_GENERATION_CHANGED);
                    atomic_set(&shared_save_pending, 1);
                    response = (struct nemu_link_studio_packet){
                        .kind = NEMU_LINK_STUDIO_ERROR,
                        .transaction = p.transaction,
                        .offset = p.offset,
                        .length = 2,
                        .data = {NEMU_LINK_STUDIO_STALE, (uint8_t)p.kind},
                    };
                    response_pending = true;
                } else if (dynamic_settings_scope_is_dirty(DYNAMIC_SETTING_SCOPE_SHARED) &&
                           !dynamic_settings_shared_dirty_is_preview()) {
                    response = (struct nemu_link_studio_packet){
                        .kind = NEMU_LINK_STUDIO_ERROR,
                        .transaction = p.transaction,
                        .offset = p.offset,
                        .length = 2,
                        .data = {NEMU_LINK_STUDIO_BUSY, (uint8_t)p.kind},
                    };
                    response_pending = true;
                } else {
                    response_pending = nemu_link_studio_group_receive(
                        &shared_rx, &p, &shared_live, SIDE, dynamic_settings_shared_group_count(),
                        shared_group_id, apply_shared_group, commit_shared_received, NULL, now,
                        &response);
                    if (shared_rx.metadata_received == NEMU_LINK_STUDIO_VERSION_SIZE &&
                        shared_rx.target.size) {
                        consider_shared_offer();
                    }
                    if (shared_rx.receipt) {
                        poll_until = now + 200;
                    }
                }
            } else if (p.kind == NEMU_LINK_STUDIO_GROUP_META_ACK ||
                       p.kind == NEMU_LINK_STUDIO_GROUP_DATA_ACK ||
                       p.kind == NEMU_LINK_STUDIO_GROUP_COMMIT_ACK ||
                       (p.kind == NEMU_LINK_STUDIO_ERROR &&
                        (p.data[1] == NEMU_LINK_STUDIO_GROUP_META ||
                         p.data[1] == NEMU_LINK_STUDIO_GROUP_DATA ||
                         p.data[1] == NEMU_LINK_STUDIO_GROUP_COMMIT))) {
                if (nemu_link_studio_tx_receive(&shared_tx, &p, now)) {
                    if (shared_tx.phase == NEMU_LINK_STUDIO_TX_DONE) {
                        shared_synchronized = true;
                        atomic_set(&shared_last_stale_reason, SHARED_STALE_NONE);
                        shared_next_offer = UINT64_MAX;
                        poll_until = now + 200;
                        LOG_INF("Shared settings synchronized K=%llu D=%llu", shared_live.clock[0],
                                shared_live.clock[1]);
                    } else if (shared_tx.phase == NEMU_LINK_STUDIO_TX_FAILED) {
                        shared_next_offer =
                            shared_tx.error == NEMU_LINK_STUDIO_INVALID ? UINT64_MAX : now + 1000;
                    }
                }
            } else
#endif
                if (p.kind == NEMU_LINK_STUDIO_META || p.kind == NEMU_LINK_STUDIO_DELTA_META ||
                    p.kind == NEMU_LINK_STUDIO_DATA || p.kind == NEMU_LINK_STUDIO_COMMIT) {
                if ((p.kind == NEMU_LINK_STUDIO_META || p.kind == NEMU_LINK_STUDIO_DELTA_META) &&
                    !p.offset && p.transaction != rx.transaction) {
                    last_keymap_receive_error = NEMU_LINK_STUDIO_OK;
                    last_keymap_receive_error_code = 0;
                    last_keymap_receive_error_index = 0;
                }
                /* Always inspect incoming keymap generations first. The
                 * coordinator, not a blanket dirty check, decides which
                 * persisted image wins and whether a local preview is stale. */
                response_pending =
                    nemu_link_studio_stream_receive(&rx, &p, &live, nemu_link_studio_schema(), now,
                                                    apply_record, commit_received, NULL, &response);
                if (response.kind == NEMU_LINK_STUDIO_ERROR) {
                    last_keymap_receive_error = response.data[0];
                }
                if (rx.offered) {
                    consider_offer();
                }
                if (rx.expired && rx.started) {
                    rx.started = false;
                    (void)describe(&live);
                    next_offer = now + 1000;
                }
                if (rx.receipt) {
                    poll_until = now + 200;
                }
            } else if (nemu_link_studio_tx_receive(&tx, &p, now)) {
                if (tx.phase == NEMU_LINK_STUDIO_TX_DONE) {
                    delta_valid = false;
                    atomic_set(&synchronized, 1);
                    next_offer = UINT64_MAX;
                    poll_until = now + 200;
                    LOG_INF("Studio synchronized K=%llu D=%llu", live.clock[0], live.clock[1]);
                } else if (tx.phase == NEMU_LINK_STUDIO_TX_FAILED) {
                    delta_valid = false;
                    next_offer = now + 1000;
                }
            }
        } else {
            rejected_frames++;
        }
    }
    if (ready && !dirty && !rx.started && !tx_active() && now >= next_offer) {
        uint16_t next_transaction =
            tx.transaction == UINT16_MAX ? 1U : (uint16_t)(tx.transaction + 1U);
        memset(&reader, 0, sizeof(reader));
        reader.delta = delta_valid;
        bool started =
            !describe(&live) &&
            (delta_valid
                 ? nemu_link_studio_tx_start_delta(&tx, next_transaction, &live, &delta_base,
                                                   delta_size, read_image, NULL, now, 40, 10000)
                 : nemu_link_studio_tx_start_reader(&tx, next_transaction, &live, read_image, NULL,
                                                    now, 40, 10000));
        if (started) {
            next_offer = UINT64_MAX;
            LOG_INF("Studio offer id=%u K=%llu D=%llu delta=%u bytes=%u", tx.transaction,
                    live.clock[0], live.clock[1], tx.delta, tx.transfer_size);
        }
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    bool shared_dirty = dynamic_settings_scope_is_dirty(DYNAMIC_SETTING_SCOPE_SHARED);
    if (shared_ready && dynamic_settings_shared_group_count() && atomic_get(&synchronized) &&
        next_offer == UINT64_MAX && !shared_synchronized && !shared_dirty && !shared_rx_active() &&
        !shared_tx_active() && shared_next_offer != UINT64_MAX && now >= shared_next_offer) {
        struct nemu_link_studio_version current = shared_live;
        int err = describe_shared(&current);
        if (!err && (current.size != shared_live.size ||
                     memcmp(current.checksum, shared_live.checksum, sizeof(current.checksum)))) {
            if (shared_live.clock[SIDE] != UINT64_MAX) {
                current.clock[SIDE] = shared_live.clock[SIDE] + 1;
                err = persist_shared_version(&current);
                if (!err) {
                    shared_live = current;
                    shared_synchronized = false;
                }
            } else {
                err = -EOVERFLOW;
            }
        }
        if (!err) {
            memset(&shared_reader, 0, sizeof(shared_reader));
            uint16_t transaction =
                shared_tx.transaction == UINT16_MAX ? 1U : (uint16_t)(shared_tx.transaction + 1U);
            if (nemu_link_studio_tx_start_groups_reader(&shared_tx, transaction, &shared_live,
                                                        read_shared_image, NULL, now, 40, 10000)) {
                shared_next_offer = UINT64_MAX;
                LOG_INF("Shared settings offer id=%u K=%llu D=%llu groups=%u bytes=%u",
                        shared_tx.transaction, shared_live.clock[0], shared_live.clock[1],
                        dynamic_settings_shared_group_count(), shared_tx.transfer_size);
            }
        } else if (err != -ENOENT) {
            shared_next_offer = now + 1000;
        }
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    bool remote_due = false;
    bool remote_active = false;
    bool schema_due = local_shared_contract_sent != (BIT(0) | BIT(1));
    k_mutex_lock(&remote_lock, K_FOREVER);
    remote_active = remote_inflight;
    remote_due = remote_inflight && !remote_done &&
                 ((!remote_base_acked && remote_base_needed ? !remote_base_sent : !remote_sent) ||
                  now < remote_last_sent || now - remote_last_sent >= 40);
    k_mutex_unlock(&remote_lock);
#endif
    if (response_pending) {
        outgoing = response;
        send_action = SEND_RESPONSE;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    } else if (owner_schema_reply_pending) {
        outgoing = owner_schema_reply[owner_schema_reply_next];
        send_action = SEND_OWNER_SCHEMA;
    } else if (schema_due) {
        uint8_t part = (local_shared_contract_sent & BIT(0)) ? 1 : 0;
        outgoing = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_SCHEMA_HASH,
            .offset = part ? 24 : 0,
            .length = part ? 8 : 24,
        };
        memcpy(outgoing.data, dynamic_settings_shared_contract_sha256 + outgoing.offset,
               outgoing.length);
        send_action = SEND_SCHEMA;
    } else if (remote_due) {
        k_mutex_lock(&remote_lock, K_FOREVER);
        if (remote_due && remote_inflight && !remote_done) {
            if (remote_base_needed && !remote_base_acked) {
                outgoing = (struct nemu_link_studio_packet){
                    .kind = NEMU_LINK_STUDIO_REMOTE_BASE,
                    .transaction = remote_transaction,
                    .length = NEMU_LINK_STUDIO_CHUNK_SIZE,
                };
                encode_remote_base(outgoing.data, remote_request.session_id,
                                   remote_request.sequence, remote_shared_base);
                send_action = SEND_REMOTE_BASE;
            } else {
                outgoing = (struct nemu_link_studio_packet){
                    .kind = NEMU_LINK_STUDIO_REMOTE_REQUEST,
                    .transaction = remote_transaction,
                    .length = NEMU_LINK_STUDIO_CHUNK_SIZE,
                };
                encode_remote_request(outgoing.data, &remote_request);
                send_action = SEND_REMOTE_TX;
            }
        }
        k_mutex_unlock(&remote_lock);
#endif
    } else if (ready && !dirty && nemu_link_studio_tx_next(&tx, now, &outgoing)) {
        send_action = SEND_KEYMAP_TX;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    } else if (!shared_dirty && nemu_link_studio_tx_next(&shared_tx, now, &outgoing)) {
        send_action = SEND_SHARED_TX;
#endif
    } else if (SIDE == NEMU_LINK_STUDIO_KEYBOARD &&
               (rx_active() || tx_active() || now < poll_until
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
                || shared_rx_active() || shared_tx_active() || remote_active
#endif
                ) &&
               now - last_poll >= ACTIVE_SERVICE_MS) {
        outgoing = (struct nemu_link_studio_packet){.kind = NEMU_LINK_STUDIO_POLL};
        send_action = SEND_POLL;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    if (send_action == SEND_NONE && shared_tx.phase == NEMU_LINK_STUDIO_TX_FAILED &&
        shared_tx.error == NEMU_LINK_STUDIO_STALE && shared_next_offer == UINT64_MAX) {
        shared_next_offer = now + 1000;
    }
#endif
    set_busy(tx_active() || rx.started);
    bool active = tx_active() || rx_active() || response_pending || now < poll_until ||
                  k_msgq_num_used_get(&control_rx);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    active = active || shared_tx_active() || shared_rx_active();
    active = active || remote_active || schema_due || owner_schema_reply_pending;
#endif
    atomic_set(&radio_busy, active || atomic_get(&remote_notice));
    /* Advertise useful outbound work only. Empty POLLs, an incoming transfer
     * and completion drain are local service work, not another remote update.
     * Folding them into pending feeds the receiver's POLLs back into notices. */
    atomic_set(&discovery_pending,
               response_pending || tx_active() || (!dirty && next_offer != UINT64_MAX)
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
                   || shared_tx_active() || (!shared_dirty && shared_next_offer != UINT64_MAX) ||
                   remote_active || schema_due || owner_schema_reply_pending
#endif
    );
    if (active) {
        delay_ms = ACTIVE_SERVICE_MS;
    } else if (!dirty && next_offer != UINT64_MAX) {
        delay_ms = next_offer > now ? next_offer - now : ACTIVE_SERVICE_MS;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    } else if (!shared_dirty && shared_next_offer != UINT64_MAX) {
        delay_ms = shared_next_offer > now ? shared_next_offer - now : ACTIVE_SERVICE_MS;
    } else if (!shared_ready) {
        delay_ms = 1000;
#endif
    }
done:
    if (!valid) {
        /* A route pause waits for a link event, with one timeout to free editors
         * if the in-flight transfer never resumes. No discovery/idle timer. */
        atomic_set(&radio_busy, 0);
        if (tx_active()) {
            uint64_t deadline = tx.last_progress + 10000;
            delay_ms = deadline > now ? deadline - now : ACTIVE_SERVICE_MS;
        }
        if (rx_active()) {
            uint64_t deadline = rx.last_progress + 10000;
            delay_ms = MIN(delay_ms, deadline > now ? deadline - now : ACTIVE_SERVICE_MS);
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
        if (shared_tx_active()) {
            uint64_t deadline = shared_tx.last_progress + 10000;
            delay_ms = MIN(delay_ms, deadline > now ? deadline - now : ACTIVE_SERVICE_MS);
        }
        if (shared_rx_active()) {
            uint64_t deadline = shared_rx.last_progress + 10000;
            delay_ms = MIN(delay_ms, deadline > now ? deadline - now : ACTIVE_SERVICE_MS);
        }
#endif
    } else if (!ready && atomic_get(&loaded)) {
        delay_ms = 1000;
    }
    if (now >= next_status) {
        LOG_INF("Studio status ready=%u route=%u synced=%d dirty=%d tx=%u/%u rx=%u/%u recv=%ld "
                "drop=%ld reject=%u sent=%u wait=%u errors=%u last=%d",
                ready, valid, (int)atomic_get(&synchronized), nemu_link_studio_dirty(), tx.phase,
                tx.offset, rx.metadata_received, rx.received, (long)atomic_get(&received_frames),
                (long)atomic_get(&dropped_frames), rejected_frames, sent_frames, send_waits,
                send_errors, last_send_error);
        next_status = now + 3000;
    }
    k_mutex_unlock(&store_lock);
    /* No store/map lock across link_lock: the link input path takes map_lock. */
    if (send_action != SEND_NONE) {
        uint8_t bytes[64];
        size_t size =
            nemu_link_studio_frame_encode(bytes, sizeof(bytes), &route, &outgoing, FRAME_CRC);
        int err = size ? nemu_link_link_studio_send(&route, bytes, size) : -EINVAL;
        if (!err) {
            sent_frames++;
            k_mutex_lock(&store_lock, K_FOREVER);
            if (send_action == SEND_KEYMAP_TX && tx.transaction == outgoing.transaction) {
                nemu_link_studio_tx_submitted(&tx, now);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
            } else if (send_action == SEND_SHARED_TX &&
                       shared_tx.transaction == outgoing.transaction) {
                nemu_link_studio_tx_submitted(&shared_tx, now);
            } else if (send_action == SEND_REMOTE_TX || send_action == SEND_REMOTE_BASE) {
                k_mutex_lock(&remote_lock, K_FOREVER);
                if (send_action == SEND_REMOTE_BASE) {
                    remote_base_sent = true;
                } else {
                    remote_sent = true;
                }
                remote_last_sent = now;
                k_mutex_unlock(&remote_lock);
            } else if (send_action == SEND_SCHEMA) {
                local_shared_contract_sent |= outgoing.offset ? BIT(1) : BIT(0);
            } else if (send_action == SEND_OWNER_SCHEMA) {
                owner_schema_reply_pending &= (uint8_t)~BIT(owner_schema_reply_next);
                owner_schema_reply_next++;
#endif
            } else if (response_pending && response.transaction == outgoing.transaction &&
                       response.kind == outgoing.kind && response.offset == outgoing.offset) {
                response_pending = false;
            }
            last_poll = now;
            k_mutex_unlock(&store_lock);
        } else if (err == -EBUSY || err == -EAGAIN) {
            send_waits++;
        } else {
            send_errors++;
            last_send_error = err;
        }
    }
    if (delay_ms != UINT64_MAX) {
        /* schedule preserves an event's concurrent immediate reschedule. */
        k_work_schedule_for_queue(&worker, &sync_work, K_MSEC(delay_ms));
    }
}
static int handle_set(const char *name, size_t len, settings_read_cb read, void *arg) {
    if (strcmp(name, "version")) {
        return -ENOENT;
    }
    uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (len != sizeof(bytes) || read(arg, bytes, sizeof(bytes)) != sizeof(bytes) ||
        !nemu_link_studio_version_decode(&live, bytes)) {
        return -EINVAL;
    }
    have_metadata = true;
    return 0;
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
static int handle_esbc1_set(const char *name, size_t len, settings_read_cb read, void *arg) {
    if (strcmp(name, "v/shared_config")) {
        return -ENOENT;
    }
    uint8_t bytes[NEMU_LINK_STUDIO_VERSION_SIZE];
    if (len != sizeof(bytes) || read(arg, bytes, sizeof(bytes)) != sizeof(bytes) ||
        !nemu_link_studio_version_decode(&shared_live, bytes)) {
        return -EINVAL;
    }
    have_shared_metadata = true;
    return 0;
}
#endif
static int handle_commit(void) {
    atomic_set(&loaded, 1);
    wake_service();
    return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(m6, "m6", NULL, handle_set, handle_commit, NULL);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
SETTINGS_STATIC_HANDLER_DEFINE(esbc1, "esbc1", NULL, handle_esbc1_set, handle_commit, NULL);
#endif
static int init(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SHARED_SETTINGS)
    if (!nemu_link_studio_group_rx_init(&shared_rx, dynamic_settings_storage_scratch,
                                        dynamic_settings_storage_scratch_size)) {
        return -EINVAL;
    }
    int err = dynamic_settings_register_scope_saved(shared_scope_saved, NULL);
    if (err) {
        LOG_ERR("Shared settings save observer registration failed: %d", err);
    }
    err = dynamic_settings_register_remote(remote_settings_call, NULL);
    if (err) {
        LOG_ERR("Settings peer relay registration failed: %d", err);
    }
    err = dynamic_settings_register_scope_save_validator(validate_shared_scope_save, NULL);
    if (err) {
        LOG_ERR("Shared settings save validator registration failed: %d", err);
    }
#endif
    k_work_queue_start(&worker, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), 6, NULL);
    k_thread_name_set(&worker.thread, "studio_sync");
    atomic_set(&worker_started, 1);
    wake_service();
    return 0;
}
SYS_INIT(init, APPLICATION, 99);
