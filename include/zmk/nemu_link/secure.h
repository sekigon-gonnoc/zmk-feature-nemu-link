#pragma once
#include <zmk/nemu_link/wire.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>

#define NEMU_LINK_SECURE_WIRE_VERSION NEMU_LINK_WIRE_VERSION
#define NEMU_LINK_SECURE_TAG_SIZE NEMU_LINK_CCM_MIC_SIZE
#define NEMU_LINK_SECURE_FRAME_SIZE NEMU_LINK_FRAME_MAX_SIZE
#define NEMU_LINK_SECURE_RANDOM_SIZE NEMU_LINK_AUTH_RANDOM_SIZE

enum nemu_link_secure_phase {
    NEMU_LINK_SECURE_OFF,
    NEMU_LINK_SECURE_HELLO,
    NEMU_LINK_SECURE_CHALLENGE,
    NEMU_LINK_SECURE_FINISH,
    NEMU_LINK_SECURE_READY,
};

struct nemu_link_mini_metadata {
    uint32_t epoch, session, peer_nonce, hop_epoch, mode;
    uint8_t hop_index, dongle, source;
};
struct nemu_link_mini_context {
    struct nemu_link_mini_metadata tx, rx;
    uint8_t key[NEMU_LINK_AES_KEY_SIZE], iv[NEMU_LINK_CCM_IV_SIZE], battery, tx_battery;
    bool have_tx, have_rx, valid;
};

struct nemu_link_secure_peer {
    enum nemu_link_secure_phase phase;
    uint8_t client_random[NEMU_LINK_SECURE_RANDOM_SIZE];
    uint8_t server_random[NEMU_LINK_SECURE_RANDOM_SIZE];
    uint8_t nonce_prefix[NEMU_LINK_CCM_IV_SIZE];
    uint8_t ccm_key[NEMU_LINK_AES_KEY_SIZE];
    bool ccm_key_valid;
    uint8_t ccm_failures;
    uint8_t dongle_id;
    uint32_t tx_sequence;
    uint32_t rx_sequence;
    struct nemu_link_mini_context mini;
};

#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
/* Local metadata only; client/server random bytes and key material stay private. */
struct nemu_link_secure_auth_diag {
    enum nemu_link_secure_phase phase_before;
    bool challenge_matches, mac_checked;
    psa_status_t mac_status;
    uint32_t loaded_key_fingerprint;
};
struct nemu_link_secure_auth_diag nemu_link_secure_auth_diagnostic(void);
#endif

int nemu_link_secure_init(void);
void nemu_link_secure_reset(struct nemu_link_secure_peer *peer);
int nemu_link_secure_start(struct nemu_link_secure_peer *peer, uint32_t link_nonce);
bool nemu_link_secure_ready(const struct nemu_link_secure_peer *peer);
size_t nemu_link_secure_auth_next(const struct nemu_link_secure_peer *peer, uint8_t source,
                                  uint8_t dongle, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE]);
int nemu_link_secure_auth_receive(struct nemu_link_secure_peer *peer, bool central, bool allow_new,
                                  uint8_t source, uint8_t dongle, const uint8_t *wire, size_t size,
                                  uint8_t reply[NEMU_LINK_FRAME_MAX_SIZE], size_t *reply_size);
int nemu_link_secure_wrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                          const uint8_t *old, size_t old_size,
                          uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *out_size);
int nemu_link_secure_wrap_compact(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                                  const uint8_t *old, size_t old_size,
                                  uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *out_size);
int nemu_link_secure_unwrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                            const uint8_t *wire, size_t size, uint32_t epoch, uint32_t session,
                            uint32_t peer_nonce, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE],
                            size_t *out_size);
