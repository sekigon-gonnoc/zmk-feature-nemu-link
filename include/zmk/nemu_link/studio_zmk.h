#pragma once
#include <zephyr/kernel.h>
#include <zmk/nemu_link/studio_stream.h>
#include <zmk/nemu_link/studio_transport.h>
extern struct k_mutex nemu_link_studio_map_lock;
const struct nemu_link_studio_schema *nemu_link_studio_schema(void);
void nemu_link_studio_schema_digest(uint8_t checksum[32]);
/* Caller serializes editor/save/receive. Each operation uses one <=72B record. */
int nemu_link_studio_record_read(uint16_t index, bool stock,
                                 uint8_t bytes[NEMU_LINK_STUDIO_RECORD_MAX], size_t *size);
/* Pending native editor bits, captured before save clears them. Header is always included. */
bool nemu_link_studio_record_changed(uint16_t index);
int nemu_link_studio_record_write(uint16_t index, const uint8_t *bytes, size_t size);
int nemu_link_studio_native_save(void);
int nemu_link_studio_native_discard(void);
int nemu_link_studio_native_dirty(void);
int nemu_link_studio_save(bool stock);
int nemu_link_studio_discard(void);
int nemu_link_studio_dirty(void);
int nemu_link_studio_select_layout(uint8_t index);
bool nemu_link_studio_edit_allowed(void);
void nemu_link_studio_release_all(void);
void nemu_link_studio_link_state(bool up, const struct nemu_link_studio_route *route);
void nemu_link_studio_receive(const uint8_t *bytes, size_t size, uint64_t received_ms);
bool nemu_link_studio_radio_busy(void);
bool nemu_link_studio_synced(void);
struct nemu_link_studio_shared_diagnostics {
    bool ready;
    bool synchronized;
    bool save_pending;
    bool receive_active;
    bool contract_matches;
    bool peer_shared_contract_known;
    bool peer_shared_contract_matches;
    bool keymap_synchronized;
    bool keymap_schema_matches;
    bool keymap_dirty;
    bool keymap_transfer_active;
    bool keymap_offer_pending;
    bool keymap_receive_active;
    bool keymap_waiting_for_keyboard;
    bool shared_dirty;
    bool shared_transfer_active;
    bool shared_offer_pending;
    uint32_t store_generation;
    uint32_t synchronized_generation;
    uint32_t keymap_transfer_offset;
    uint32_t keymap_transfer_size;
    uint32_t keymap_transfer_age_ms;
    uint32_t keymap_receive_offset;
    uint32_t keymap_receive_size;
    uint32_t keymap_receive_metadata;
    uint32_t keymap_receive_error_code;
    uint32_t keymap_receive_error_index;
    uint32_t shared_transfer_offset;
    uint32_t shared_receive_metadata;
    int16_t peripheral_0_rssi;
    int16_t peripheral_1_rssi;
    uint64_t keyboard_revision;
    uint64_t dongle_revision;
    uint8_t last_stale_reason;
    uint8_t keymap_transfer_phase;
    uint8_t keymap_transfer_error;
    uint8_t keymap_receive_error;
    uint8_t keymap_offer_decision;
    uint8_t shared_offer_decision;
    uint8_t shared_transfer_phase;
    uint8_t shared_transfer_error;
};
void nemu_link_studio_get_shared_diagnostics(
    struct nemu_link_studio_shared_diagnostics *diagnostics);
bool nemu_link_studio_get_peripheral_rssi(uint8_t source, int8_t *rssi_dbm);
/* Atomic snapshot for authenticated link ACKs; never takes the store/map lock. */
bool nemu_link_studio_pending(void);
/* Only call after an authenticated, fresh application ACK for the current session. */
void nemu_link_studio_notify(void);
int nemu_link_link_studio_send(const struct nemu_link_studio_route *route, const uint8_t *bytes,
                               size_t size);

struct zmk_position_state_changed;
void nemu_link_studio_track_position(const struct zmk_position_state_changed *pos);
