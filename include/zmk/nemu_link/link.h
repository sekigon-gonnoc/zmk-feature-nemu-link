#ifndef ZMK_NEMU_LINK_H
#define ZMK_NEMU_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zmk/nemu_link/link_keys.h>

#define NEMU_LINK_LINK_SOURCES 2
#define NEMU_LINK_LINK_VERSION 1
#define NEMU_LINK_LINK_BATTERY_UNKNOWN UINT8_MAX
#define NEMU_LINK_LINK_WIRE_SIZE (24 + NEMU_LINK_LINK_KEY_BYTES)
#define NEMU_LINK_LINK_CONTROL_WIRE_SIZE 20

/* Application payload inside the existing ESB envelope and CRC. No native
 * struct/enum layout goes on air. Every STATE is a complete snapshot in stock physical-layout
 * positions. */
enum nemu_link_link_kind {
    NEMU_LINK_LINK_HELLO = 1,
    NEMU_LINK_LINK_CHALLENGE,
    NEMU_LINK_LINK_STATE,
    NEMU_LINK_LINK_ACK,
};

struct nemu_link_link_message {
    enum nemu_link_link_kind kind;
    uint8_t source;
    uint8_t battery;     /* Percent or UNKNOWN in STATE; zero in other messages. */
    bool studio_pending; /* Only ACK: peer has Studio work to collect. */
    uint32_t peer_nonce;
    uint32_t epoch;
    uint32_t session;
    uint32_t sequence;
    struct nemu_link_link_keys keys;
    uint32_t buttons;
};

bool nemu_link_link_encode(uint8_t *wire, size_t size,
                           const struct nemu_link_link_message *message);
bool nemu_link_link_decode(struct nemu_link_link_message *message, const uint8_t *wire,
                           size_t size);
/* Envelope codecs may use the short control core directly, without padding a
 * temporary full STATE core. The public fixed-size codec above remains strict. */
bool nemu_link_link_core_encode(uint8_t *wire, size_t size,
                                const struct nemu_link_link_message *message);
bool nemu_link_link_core_decode(struct nemu_link_link_message *message, const uint8_t *wire,
                                size_t size);
bool nemu_link_link_sequence_after(uint32_t candidate, uint32_t previous);

enum nemu_link_link_phase {
    NEMU_LINK_LINK_DOWN,
    NEMU_LINK_LINK_SYNCING,
    NEMU_LINK_LINK_UP,
};

struct nemu_link_link_peer {
    enum nemu_link_link_phase phase;
    uint32_t peer_nonce;
    uint32_t session;
    uint32_t sequence;
    uint64_t last_seen_ms;
    struct nemu_link_link_keys keys;
    struct nemu_link_link_keys blocked_keys;
    uint32_t buttons;
    uint32_t blocked_buttons;
};

struct nemu_link_link_central {
    uint32_t epoch;
    uint32_t release_ms;
    uint32_t allowed_buttons[NEMU_LINK_LINK_SOURCES];
    struct nemu_link_link_peer peers[NEMU_LINK_LINK_SOURCES];
};

/* Outputs are returned, never dispatched from the core. A serialized adapter
 * MUST apply cancellation/changes before submitting reply. Cancellation is
 * deliberately separate from normal release: a hold-tap must not produce a tap
 * and a pending combo or mode timer must not fire because a source vanished. */
struct nemu_link_link_result {
    bool connected;
    bool disconnected;
    struct nemu_link_link_keys cancel_keys;
    uint32_t cancel_buttons;
    struct nemu_link_link_keys press_keys;
    struct nemu_link_link_keys release_keys;
    uint32_t press_buttons;
    uint32_t release_buttons;
    bool has_reply;
    struct nemu_link_link_message reply;
};

/* epoch must be fresh for each central boot; zero is invalid. Generation is
 * central-owned and never wraps (exhaustion fails closed). Key state is tracked separately per
 * source. Button slots are local to each source. All times are local monotonic milliseconds. APIs
 * require serialized calls and output handling. */
bool nemu_link_link_central_init(struct nemu_link_link_central *central, uint32_t epoch,
                                 uint32_t release_ms,
                                 const uint32_t buttons[NEMU_LINK_LINK_SOURCES]);
void nemu_link_link_central_tick(struct nemu_link_link_central *central, uint8_t source,
                                 uint64_t now_ms, struct nemu_link_link_result *result);
/* physical_source is the validated RX pipe, never the payload's asserted ID.
 * true means a fresh HELLO or STATE was accepted. Duplicates can return an ACK
 * without extending liveness. Even rejected messages can cause due expiry. */
bool nemu_link_link_central_receive(struct nemu_link_link_central *central, uint8_t physical_source,
                                    const struct nemu_link_link_message *message, uint64_t now_ms,
                                    struct nemu_link_link_result *result);

/* Bounded acknowledgement ledger; overwritten entries can no longer confirm
 * liveness. Saturation therefore fails closed rather than accepting old ACKs. */
#define NEMU_LINK_LINK_ACK_SLOTS 16

enum nemu_link_link_peripheral_phase {
    NEMU_LINK_LINK_PERIPHERAL_HELLO,
    NEMU_LINK_LINK_PERIPHERAL_SYNCING,
    NEMU_LINK_LINK_PERIPHERAL_UP,
    NEMU_LINK_LINK_PERIPHERAL_LOST,
};

struct nemu_link_link_sent {
    bool valid;
    uint32_t sequence;
    uint64_t sent_ms;
};

struct nemu_link_link_peripheral {
    enum nemu_link_link_peripheral_phase phase;
    uint8_t source;
    uint32_t peer_nonce;
    uint32_t epoch;
    uint32_t session;
    uint32_t sequence;
    uint32_t ack_sequence;
    uint32_t interval_ms;
    uint32_t release_ms;
    uint32_t allowed_buttons;
    uint64_t confirmed_ms;
    uint64_t last_sent_ms;
    struct nemu_link_link_keys last_keys;
    uint32_t last_buttons;
    bool has_sent;
    unsigned int next_slot;
    struct nemu_link_link_sent sent[NEMU_LINK_LINK_ACK_SLOTS];
};

bool nemu_link_link_peripheral_init(struct nemu_link_link_peripheral *peer, uint8_t source,
                                    uint32_t nonce, uint64_t now_ms, uint32_t interval_ms,
                                    uint32_t release_ms, uint32_t buttons);
/* Only after LOST; caller allocates a fresh nonce for every attempt, including
 * retries without reboot. A delayed CHALLENGE must not identify a new attempt. */
bool nemu_link_link_peripheral_restart(struct nemu_link_link_peripheral *peer, uint32_t fresh_nonce,
                                       uint64_t now_ms);
void nemu_link_link_peripheral_tick(struct nemu_link_link_peripheral *peer, uint64_t now_ms);
/* Produces immediate snapshots on physical changes, otherwise at interval_ms.
 * Call regularly even without input. Queue submission is NOT acknowledgement.
 * Generated frames may be lost; a later complete snapshot repairs that loss. */
bool nemu_link_link_peripheral_next(struct nemu_link_link_peripheral *peer, uint64_t now_ms,
                                    struct nemu_link_link_keys keys, uint32_t buttons,
                                    struct nemu_link_link_message *message);
bool nemu_link_link_peripheral_receive(struct nemu_link_link_peripheral *peer, uint64_t now_ms,
                                       const struct nemu_link_link_message *message);

#endif
