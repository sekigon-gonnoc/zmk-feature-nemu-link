#include <errno.h>
#include <string.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/nemu_link/secure.h>
#include <zmk/nemu_link/security_role.h>
#include <zmk/nemu_link/key_store.h>
#include <zmk/nemu_link/power_profile.h>
#include <zmk/nemu_link/power_trace.h>
#include <zmk/nemu_link/compact.h>
#include <zmk/nemu_link/nrf_ccm.h>
#include <mbedtls/platform_util.h>
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
#include <zmk/nemu_link/ccm_radio.h>
#endif

#define AUTH_VERSION NEMU_LINK_SECURE_WIRE_VERSION
#define AUTH_HELLO 1
#define AUTH_CHALLENGE 2
#define AUTH_FINISH 3
#define AUTH_CONFIRM 4
#define SECURE_PLAIN_MAX                                                                           \
    NEMU_LINK_COMPACT_BODY_MAX /* Native frame content bound, shared by both wire formats. */
#define SECURE_BIND_SIZE NEMU_LINK_COMPACT_BIND_SIZE
#define CCM_FAILURE_LIMIT NEMU_LINK_CCM_FAILURE_LIMIT
#define SECURE_REKEY_AT NEMU_LINK_COUNTER_REKEY_AT
#define AUTH_TRANSCRIPT_HEADER_SIZE 6U
#define AUTH_TRANSCRIPT_CLIENT_OFFSET AUTH_TRANSCRIPT_HEADER_SIZE
#define AUTH_TRANSCRIPT_SERVER_OFFSET (AUTH_TRANSCRIPT_HEADER_SIZE + NEMU_LINK_AUTH_RANDOM_SIZE)
#define AUTH_TRANSCRIPT_SIZE (AUTH_TRANSCRIPT_HEADER_SIZE + NEMU_LINK_AUTH_RANDOM_PAIR_SIZE)
#define TRAFFIC_MATERIAL_SIZE (NEMU_LINK_AES_KEY_SIZE + NEMU_LINK_CCM_IV_SIZE)

static uint8_t psk[NEMU_LINK_KEY_SIZE];
static const uint8_t auth_salt[] = "zmk-nemu-link-auth-v1-mini-ccm4";
static const uint8_t auth_info[] = "handshake-hmac-sha256";
static const uint8_t traffic_info[] = "traffic-aes128-ccm";
static psa_key_id_t auth_key;
#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
static uint32_t loaded_key_fingerprint;
static struct nemu_link_secure_auth_diag auth_diag;
struct nemu_link_secure_auth_diag nemu_link_secure_auth_diagnostic(void) { return auth_diag; }
#endif

static int derive(const uint8_t *salt, size_t salt_size, const uint8_t *info, size_t info_size,
                  uint8_t *out, size_t out_size) {
    psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
    psa_status_t rc = psa_key_derivation_setup(&op, PSA_ALG_HKDF(PSA_ALG_SHA_256));
    if (rc == PSA_SUCCESS) {
        rc = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT, salt, salt_size);
    }
    if (rc == PSA_SUCCESS) {
        rc = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SECRET, psk, sizeof(psk));
    }
    if (rc == PSA_SUCCESS) {
        rc = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO, info, info_size);
    }
    if (rc == PSA_SUCCESS) {
        rc = psa_key_derivation_output_bytes(&op, out, out_size);
    }
    psa_key_derivation_abort(&op);
    return rc == PSA_SUCCESS ? 0 : -EIO;
}

int nemu_link_secure_init(void) {
    /* Public vectors only; no provisioned key is sent or changed by this test. */
    int ccm_err = nemu_link_nrf_ccm_self_test();
    if (ccm_err) {
        return ccm_err;
    }
    if (auth_key) {
        return 0;
    }
    int key_rc = nemu_link_key_store_read(psk);
    if (key_rc) {
        return key_rc;
    }
#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
    nemu_link_key_store_fingerprint(&loaded_key_fingerprint);
#endif
    if (psa_crypto_init() != PSA_SUCCESS) {
        return -EIO;
    }
    uint8_t material[NEMU_LINK_AUTH_TAG_SIZE];
    if (derive(auth_salt, sizeof(auth_salt) - 1, auth_info, sizeof(auth_info) - 1, material,
               sizeof(material))) {
        return -EIO;
    }
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attr, NEMU_LINK_AUTH_TAG_SIZE * NEMU_LINK_RADIO_BITS_PER_BYTE);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_status_t rc = psa_import_key(&attr, material, sizeof(material), &auth_key);
    psa_reset_key_attributes(&attr);
    memset(material, 0, sizeof(material));
    return rc == PSA_SUCCESS ? 0 : -EIO;
}

void nemu_link_secure_reset(struct nemu_link_secure_peer *peer) {
    mbedtls_platform_zeroize(peer, sizeof(*peer));
}

int nemu_link_secure_start(struct nemu_link_secure_peer *peer, uint32_t link_nonce) {
    nemu_link_secure_reset(peer);
    if (!link_nonce || sys_csrand_get(peer->client_random, sizeof(peer->client_random))) {
        return -EIO;
    }
    sys_put_le32(link_nonce, peer->client_random);
    peer->phase = NEMU_LINK_SECURE_HELLO;
    return 0;
}

bool nemu_link_secure_ready(const struct nemu_link_secure_peer *peer) {
    return peer->phase == NEMU_LINK_SECURE_READY && peer->ccm_key_valid;
}

static void transcript(uint8_t data[AUTH_TRANSCRIPT_SIZE], uint8_t type, uint8_t source,
                       uint8_t dongle, const struct nemu_link_secure_peer *peer) {
    data[0] = AUTH_VERSION;
    data[1] = type;
    data[2] = source;
    data[3] = dongle;
    data[4] = 'P';
    data[5] = 'C'; /* Peripheral challenge, central challenge. */
    memcpy(data + AUTH_TRANSCRIPT_CLIENT_OFFSET, peer->client_random, NEMU_LINK_AUTH_RANDOM_SIZE);
    memcpy(data + AUTH_TRANSCRIPT_SERVER_OFFSET, peer->server_random, NEMU_LINK_AUTH_RANDOM_SIZE);
}

static int mac(uint8_t type, uint8_t source, uint8_t dongle,
               const struct nemu_link_secure_peer *peer, uint8_t out[NEMU_LINK_AUTH_TAG_SIZE]) {
    uint8_t data[AUTH_TRANSCRIPT_SIZE];
    size_t length = 0;
    transcript(data, type, source, dongle, peer);
    return psa_mac_compute(auth_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), data, sizeof(data), out,
                           NEMU_LINK_AUTH_TAG_SIZE, &length) == PSA_SUCCESS &&
                   length == NEMU_LINK_AUTH_TAG_SIZE
               ? 0
               : -EIO;
}

static bool verify(uint8_t type, uint8_t source, uint8_t dongle,
                   const struct nemu_link_secure_peer *peer,
                   const uint8_t mac_bytes[NEMU_LINK_AUTH_TAG_SIZE]) {
    uint8_t data[AUTH_TRANSCRIPT_SIZE];
    transcript(data, type, source, dongle, peer);
    psa_status_t status = psa_mac_verify(auth_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), data,
                                         sizeof(data), mac_bytes, NEMU_LINK_AUTH_TAG_SIZE);
#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
    auth_diag.mac_checked = true;
    auth_diag.mac_status = status;
#endif
    return status == PSA_SUCCESS;
}

static size_t auth_frame(uint8_t *out, uint8_t type, uint8_t source, uint8_t dongle,
                         const struct nemu_link_secure_peer *peer) {
    out[0] = AUTH_VERSION;
    out[NEMU_LINK_AUTH_KIND_OFFSET] = type;
    out[NEMU_LINK_AUTH_SOURCE_OFFSET] = source;
    out[NEMU_LINK_AUTH_DONGLE_OFFSET] = dongle;
    size_t body = 0;
    if (type == AUTH_HELLO) {
        memcpy(out + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET, peer->client_random,
               NEMU_LINK_AUTH_RANDOM_SIZE);
        body = NEMU_LINK_AUTH_RANDOM_SIZE;
    } else if (type == AUTH_CHALLENGE) {
        memcpy(out + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET, peer->client_random,
               NEMU_LINK_AUTH_RANDOM_SIZE);
        memcpy(out + NEMU_LINK_AUTH_SERVER_RANDOM_OFFSET, peer->server_random,
               NEMU_LINK_AUTH_RANDOM_SIZE);
        body = NEMU_LINK_AUTH_RANDOM_PAIR_SIZE;
    }
    if (mac(type, source, dongle, peer, out + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET + body)) {
        return 0;
    }
    return NEMU_LINK_AUTH_HEADER_SIZE + body + NEMU_LINK_AUTH_TAG_SIZE;
}

size_t nemu_link_secure_auth_next(const struct nemu_link_secure_peer *peer, uint8_t source,
                                  uint8_t dongle, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE]) {
    if (peer->phase == NEMU_LINK_SECURE_HELLO) {
        return auth_frame(out, AUTH_HELLO, source, dongle, peer);
    }
    if (peer->phase == NEMU_LINK_SECURE_FINISH) {
        return auth_frame(out, AUTH_FINISH, source, dongle, peer);
    }
    return 0;
}

static int traffic_key(struct nemu_link_secure_peer *peer, uint8_t source, uint8_t dongle) {
    uint8_t salt[NEMU_LINK_AUTH_RANDOM_PAIR_SIZE], info[sizeof(traffic_info) + 2],
        material[TRAFFIC_MATERIAL_SIZE];
    memcpy(salt, peer->client_random, NEMU_LINK_AUTH_RANDOM_SIZE);
    memcpy(salt + NEMU_LINK_AUTH_RANDOM_SIZE, peer->server_random, NEMU_LINK_AUTH_RANDOM_SIZE);
    memcpy(info, traffic_info, sizeof(traffic_info) - 1);
    info[sizeof(traffic_info) - 1] = AUTH_VERSION;
    info[sizeof(traffic_info)] = source;
    info[sizeof(traffic_info) + 1] = dongle;
    if (derive(salt, sizeof(salt), info, sizeof(info), material, sizeof(material))) {
        return -EIO;
    }
    memcpy(peer->ccm_key, material, sizeof(peer->ccm_key));
    peer->ccm_key_valid = true;
    peer->dongle_id = dongle;
    memcpy(peer->nonce_prefix, material + NEMU_LINK_AES_KEY_SIZE, NEMU_LINK_CCM_IV_SIZE);
    mbedtls_platform_zeroize(material, sizeof(material));
    return 0;
}

int nemu_link_secure_auth_receive(struct nemu_link_secure_peer *peer, bool central, bool allow_new,
                                  uint8_t source, uint8_t dongle, const uint8_t *wire, size_t size,
                                  uint8_t reply[NEMU_LINK_FRAME_MAX_SIZE], size_t *reply_size) {
    central = NEMU_LINK_SECURITY_CENTRAL(central);
    *reply_size = 0;
#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
    auth_diag = (struct nemu_link_secure_auth_diag){
        .phase_before = peer->phase, .loaded_key_fingerprint = loaded_key_fingerprint};
#endif
    if (!auth_key || size < NEMU_LINK_AUTH_CONFIRM_SIZE || size > NEMU_LINK_AUTH_CHALLENGE_SIZE ||
        wire[0] != AUTH_VERSION || wire[NEMU_LINK_AUTH_SOURCE_OFFSET] != source ||
        wire[NEMU_LINK_AUTH_DONGLE_OFFSET] != dongle) {
        return -EINVAL;
    }
    uint8_t type = wire[NEMU_LINK_AUTH_KIND_OFFSET];
#if defined(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC) && CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC
    auth_diag.challenge_matches =
        type == AUTH_CHALLENGE && size == NEMU_LINK_AUTH_CHALLENGE_SIZE &&
        !memcmp(peer->client_random, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET,
                NEMU_LINK_AUTH_RANDOM_SIZE);
#endif
    if (central && type == AUTH_HELLO && size == NEMU_LINK_AUTH_HELLO_SIZE && allow_new) {
        struct nemu_link_secure_peer hello = {0};
        memcpy(hello.client_random, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET,
               NEMU_LINK_AUTH_RANDOM_SIZE);
        if (!verify(AUTH_HELLO, source, dongle, &hello,
                    wire + NEMU_LINK_AUTH_SERVER_RANDOM_OFFSET)) {
            return -EACCES;
        }
        if ((peer->phase == NEMU_LINK_SECURE_CHALLENGE || nemu_link_secure_ready(peer)) &&
            !memcmp(peer->client_random, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET,
                    NEMU_LINK_AUTH_RANDOM_SIZE)) {
            *reply_size = auth_frame(reply, AUTH_CHALLENGE, source, dongle, peer);
            return *reply_size ? 0 : -EIO;
        }
        nemu_link_secure_reset(peer);
        memcpy(peer->client_random, hello.client_random, NEMU_LINK_AUTH_RANDOM_SIZE);
        if (sys_csrand_get(peer->server_random, NEMU_LINK_AUTH_RANDOM_SIZE)) {
            return -EIO;
        }
        peer->phase = NEMU_LINK_SECURE_CHALLENGE;
        *reply_size = auth_frame(reply, AUTH_CHALLENGE, source, dongle, peer);
        return *reply_size ? 0 : -EIO;
    }
    if (central && type == AUTH_FINISH && size == NEMU_LINK_AUTH_CONFIRM_SIZE &&
        (peer->phase == NEMU_LINK_SECURE_CHALLENGE || nemu_link_secure_ready(peer)) &&
        verify(AUTH_FINISH, source, dongle, peer, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET)) {
        if (peer->phase == NEMU_LINK_SECURE_CHALLENGE) {
            if (traffic_key(peer, source, dongle)) {
                nemu_link_secure_reset(peer);
                return -EIO;
            }
            peer->phase = NEMU_LINK_SECURE_READY;
        }
        *reply_size = auth_frame(reply, AUTH_CONFIRM, source, dongle, peer);
        return *reply_size ? 0 : -EIO;
    }
    if (!central && type == AUTH_CHALLENGE && size == NEMU_LINK_AUTH_CHALLENGE_SIZE &&
        (peer->phase == NEMU_LINK_SECURE_HELLO || peer->phase == NEMU_LINK_SECURE_FINISH) &&
        !memcmp(peer->client_random, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET,
                NEMU_LINK_AUTH_RANDOM_SIZE)) {
        struct nemu_link_secure_peer candidate = *peer;
        if (peer->phase == NEMU_LINK_SECURE_FINISH &&
            memcmp(peer->server_random, wire + NEMU_LINK_AUTH_SERVER_RANDOM_OFFSET,
                   NEMU_LINK_AUTH_RANDOM_SIZE)) {
            return -EACCES;
        }
        memcpy(candidate.server_random, wire + NEMU_LINK_AUTH_SERVER_RANDOM_OFFSET,
               NEMU_LINK_AUTH_RANDOM_SIZE);
        if (!verify(AUTH_CHALLENGE, source, dongle, &candidate,
                    wire + NEMU_LINK_AUTH_HEADER_SIZE + NEMU_LINK_AUTH_RANDOM_PAIR_SIZE)) {
            return -EACCES;
        }
        memcpy(peer->server_random, candidate.server_random, NEMU_LINK_AUTH_RANDOM_SIZE);
        if (peer->phase == NEMU_LINK_SECURE_HELLO && traffic_key(peer, source, dongle)) {
            nemu_link_secure_reset(peer);
            return -EIO;
        }
        peer->phase = NEMU_LINK_SECURE_FINISH;
        *reply_size = auth_frame(reply, AUTH_FINISH, source, dongle, peer);
        return *reply_size ? 0 : -EIO;
    }
    if (!central && type == AUTH_CONFIRM && size == NEMU_LINK_AUTH_CONFIRM_SIZE &&
        peer->phase == NEMU_LINK_SECURE_FINISH &&
        verify(AUTH_CONFIRM, source, dongle, peer, wire + NEMU_LINK_AUTH_CLIENT_RANDOM_OFFSET)) {
        peer->phase = NEMU_LINK_SECURE_READY;
        return 0;
    }
    return -EACCES;
}

static void ccm_nonce(const struct nemu_link_secure_peer *peer, uint8_t source, bool sender_central,
                      uint32_t sequence, uint8_t out[NEMU_LINK_CCM_NONCE_SIZE]) {
    /* 39-bit counter: source in bits 32..38; direction is nonce bit 39. */
    sys_put_le32(sequence, out);
    out[NEMU_LINK_CCM_DIRECTION_OFFSET] =
        source | (sender_central ? NEMU_LINK_CCM_DIRECTION_BIT : 0);
    memcpy(out + NEMU_LINK_CCM_COUNTER_SIZE, peer->nonce_prefix, NEMU_LINK_CCM_IV_SIZE);
}

static int secure_wrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                       const uint8_t *old, size_t old_size, bool compact_link,
                       uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *out_size) {
    central = NEMU_LINK_SECURITY_CENTRAL(central);
    if (!nemu_link_secure_ready(peer) || peer->tx_sequence >= SECURE_REKEY_AT ||
        old_size < NEMU_LINK_NATIVE_AUDIO_CONTROL_SIZE || old_size > NEMU_LINK_FRAME_MAX_SIZE) {
        return -ENOTCONN;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_PACK_BEGIN, 0);
    uint8_t plain[SECURE_PLAIN_MAX], kind;
    size_t length;
    if (nemu_link_compact_pack(source, compact_link, old, old_size, &kind, plain, &length)) {
        /* The key and nonce bind wire version, physical source and direction.
         * Authenticate kind inside the plaintext before selecting a decoder. */
    } else if (!memcmp(old, "ZmKe", NEMU_LINK_NATIVE_MAGIC_SIZE) &&
               old[NEMU_LINK_NATIVE_SOURCE_OFFSET] == source &&
               (old[NEMU_LINK_NATIVE_KIND_OFFSET] == NEMU_LINK_WIRE_LINK ||
                old[NEMU_LINK_NATIVE_KIND_OFFSET] == NEMU_LINK_WIRE_STUDIO)) {
        kind = old[NEMU_LINK_NATIVE_KIND_OFFSET];
        if (kind == NEMU_LINK_WIRE_LINK) {
            if (old_size < NEMU_LINK_NATIVE_LINK_MIN_SIZE) {
                return -EINVAL;
            }
            length = old_size - NEMU_LINK_NATIVE_LINK_AUTH_PREFIX_SIZE;
            memcpy(plain, old + NEMU_LINK_NATIVE_LINK_HEADER_SIZE, 4);
            memcpy(plain + 4, old + NEMU_LINK_NATIVE_LINK_AUTH_PREFIX_SIZE, length - 4);
        } else {
            if (old_size < NEMU_LINK_NATIVE_STUDIO_MIN_SIZE ||
                old_size > NEMU_LINK_NATIVE_STUDIO_MAX_SIZE) {
                return -EINVAL;
            }
            length = old_size - NEMU_LINK_NATIVE_MIN_SIZE;
            memcpy(plain, old + NEMU_LINK_NATIVE_LINK_HEADER_SIZE, length);
        }
    } else if (!memcmp(old, "ErEv", NEMU_LINK_NATIVE_MAGIC_SIZE) &&
               old[NEMU_LINK_NATIVE_SPLIT_SOURCE_OFFSET] == source &&
               (old[NEMU_LINK_NATIVE_KIND_OFFSET] == NEMU_LINK_WIRE_SPLIT_EVENT ||
                old[NEMU_LINK_NATIVE_KIND_OFFSET] == NEMU_LINK_WIRE_SPLIT_COMMAND) &&
               old_size >= NEMU_LINK_NATIVE_SPLIT_MIN_SIZE) {
        kind = old[NEMU_LINK_NATIVE_KIND_OFFSET];
        length = old_size - NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE;
        memcpy(plain, old + NEMU_LINK_NATIVE_MAGIC_SIZE, 4);
        memcpy(plain + 4, old + NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE, length - 4);
    } else if (!memcmp(old, "EsAu", NEMU_LINK_NATIVE_MAGIC_SIZE) && old[4] == source &&
               old[6] == NEMU_LINK_WIRE_AUDIO && old_size >= NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE &&
               old_size <= NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE + NEMU_LINK_AUDIO_DATA_MAX &&
               old[11] == old_size - NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE) {
        kind = NEMU_LINK_WIRE_AUDIO;
        length = old_size - (NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 5U);
        memcpy(plain, old + NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 5U, length);
    } else if (!memcmp(old, "EsAc", NEMU_LINK_NATIVE_MAGIC_SIZE) && old[4] == source &&
               old[6] == NEMU_LINK_WIRE_AUDIO_CONTROL &&
               old_size == NEMU_LINK_NATIVE_AUDIO_CONTROL_SIZE && old[7] <= 1U &&
               (old[5] <= 0x7fU || old[5] == UINT8_MAX)) {
        kind = NEMU_LINK_WIRE_AUDIO_CONTROL;
        length = 2U;
        plain[0] = old[7];
        plain[1] = old[5];
    } else {
        return -EINVAL;
    }
    if (source > 0x7f) {
        return -EINVAL;
    }
    if (length > SECURE_PLAIN_MAX) {
        return -EMSGSIZE;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_PACK_END, (uint16_t)((kind << 8) | length));
    uint32_t sequence = ++peer->tx_sequence;
    out[0] = NEMU_LINK_SECURE_WIRE_VERSION;
    out[NEMU_LINK_COMPACT_KIND_OFFSET] = kind;
    sys_put_le32(sequence, out + NEMU_LINK_COMPACT_COUNTER_OFFSET);
    uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE];
    ccm_nonce(peer, source, central, sequence, nonce);
    size_t encrypted = 0;
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCRYPT_BEGIN, length);
    uint32_t profile_start = nemu_link_profile_start();
    /* Authenticate version/kind/source inside the ciphertext. Sequence is
     * bound by the nonce; length is bound by CCM. Never trust outer kind first. */
    uint8_t bound[NEMU_LINK_CCM_PLAIN_MAX];
    bound[0] = kind;
    memcpy(bound + SECURE_BIND_SIZE, plain, length);
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    int rc =
        nemu_link_ccm_radio_queue(source, peer->ccm_key, nonce, bound, length + SECURE_BIND_SIZE,
                                  out, NEMU_LINK_COMPACT_HEADER_SIZE);
#else
    int rc = nemu_link_nrf_ccm_crypt(false, peer->ccm_key, nonce, bound, length + SECURE_BIND_SIZE,
                                     out + NEMU_LINK_COMPACT_HEADER_SIZE,
                                     NEMU_LINK_FRAME_MAX_SIZE - NEMU_LINK_COMPACT_HEADER_SIZE);
#endif
    mbedtls_platform_zeroize(bound, sizeof(bound));
    encrypted = length + SECURE_BIND_SIZE + NEMU_LINK_SECURE_TAG_SIZE;
    nemu_link_profile_end(NEMU_LINK_PROFILE_ENCRYPT, profile_start);
    nemu_link_trace_mark(NEMU_LINK_TRACE_ENCRYPT_END, (uint16_t)-rc);
    if (rc != 0 || encrypted != length + SECURE_BIND_SIZE + NEMU_LINK_SECURE_TAG_SIZE) {
        return rc ? rc : -EIO;
    }
    *out_size = NEMU_LINK_COMPACT_HEADER_SIZE + encrypted;
    return 0;
}

int nemu_link_secure_wrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                          const uint8_t *old, size_t size, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE],
                          size_t *out_size) {
    return secure_wrap(peer, central, source, old, size, false, out, out_size);
}
int nemu_link_secure_wrap_compact(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                                  const uint8_t *old, size_t size,
                                  uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t *out_size) {
    return secure_wrap(peer, central, source, old, size, true, out, out_size);
}

int nemu_link_secure_unwrap(struct nemu_link_secure_peer *peer, bool central, uint8_t source,
                            const uint8_t *wire, size_t size, uint32_t epoch, uint32_t session,
                            uint32_t peer_nonce, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE],
                            size_t *out_size) {
    central = NEMU_LINK_SECURITY_CENTRAL(central);
    if (!nemu_link_secure_ready(peer) ||
        size < NEMU_LINK_COMPACT_HEADER_SIZE + SECURE_BIND_SIZE + NEMU_LINK_SECURE_TAG_SIZE ||
        size > NEMU_LINK_FRAME_MAX_SIZE || wire[0] != NEMU_LINK_SECURE_WIRE_VERSION ||
        size > NEMU_LINK_COMPACT_HEADER_SIZE + SECURE_BIND_SIZE + NEMU_LINK_SECURE_TAG_SIZE +
                   SECURE_PLAIN_MAX) {
        return -EINVAL;
    }
    uint32_t sequence = sys_get_le32(wire + NEMU_LINK_COMPACT_COUNTER_OFFSET);
    if (!sequence || sequence <= peer->rx_sequence) {
        return -EALREADY;
    }
    uint8_t plain[SECURE_PLAIN_MAX], nonce[NEMU_LINK_CCM_NONCE_SIZE];
    ccm_nonce(peer, source, !central, sequence, nonce);
    size_t length = 0;
    nemu_link_trace_mark(NEMU_LINK_TRACE_DECRYPT_BEGIN, size);
    uint32_t profile_start = nemu_link_profile_start();
    if (source > 0x7f) {
        return -EINVAL;
    }
    uint8_t bound[NEMU_LINK_CCM_PLAIN_MAX];
    int rc;
#if CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    rc =
        nemu_link_ccm_radio_rx_take(source, peer->ccm_key, nonce, wire, size, bound, sizeof(bound));
    if (rc == -ENOENT)
#endif
    {
        rc = nemu_link_nrf_ccm_crypt(true, peer->ccm_key, nonce,
                                     wire + NEMU_LINK_COMPACT_HEADER_SIZE,
                                     size - NEMU_LINK_COMPACT_HEADER_SIZE, bound, sizeof(bound));
    }
    if (!rc) {
        length =
            size - NEMU_LINK_COMPACT_HEADER_SIZE - NEMU_LINK_SECURE_TAG_SIZE - SECURE_BIND_SIZE;
        bool header_bad = bound[0] != wire[NEMU_LINK_COMPACT_KIND_OFFSET];
        if (header_bad) {
            rc = -EACCES;
        } else {
            memcpy(plain, bound + SECURE_BIND_SIZE, length);
        }
    }
    mbedtls_platform_zeroize(bound, sizeof(bound));
    nemu_link_profile_end(NEMU_LINK_PROFILE_DECRYPT, profile_start);
    nemu_link_trace_mark(NEMU_LINK_TRACE_DECRYPT_END, (uint16_t)-rc);
    if (rc != PSA_SUCCESS) {
        /* Do not replenish this budget after valid traffic. The existing link
         * timeout then establishes a fresh authenticated session with new keys. */
        if (rc == -EACCES && ++peer->ccm_failures >= CCM_FAILURE_LIMIT) {
            nemu_link_secure_reset(peer);
        }
        return rc;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_UNPACK_BEGIN, wire[NEMU_LINK_COMPACT_KIND_OFFSET]);
    uint8_t kind = wire[NEMU_LINK_COMPACT_KIND_OFFSET];
    size_t old_size;
    if (nemu_link_compact_kind(kind)) {
        if ((central && kind == NEMU_LINK_WIRE_ACK) || (!central && kind != NEMU_LINK_WIRE_ACK) ||
            !nemu_link_compact_unpack(kind, source, peer->dongle_id, plain, length, sequence, epoch,
                                      session, peer_nonce, out, &old_size)) {
            return -EINVAL;
        }
        peer->rx_sequence = sequence;
        *out_size = old_size;
        nemu_link_trace_mark(NEMU_LINK_TRACE_UNPACK_END, old_size);
        return 0;
    }
    if (kind == NEMU_LINK_WIRE_LINK &&
        length >= NEMU_LINK_NATIVE_LINK_MIN_SIZE - NEMU_LINK_NATIVE_LINK_AUTH_PREFIX_SIZE &&
        length <= NEMU_LINK_COMPACT_BODY_MAX) {
        old_size = length + NEMU_LINK_NATIVE_LINK_AUTH_PREFIX_SIZE;
        memcpy(out, "ZmKe", NEMU_LINK_NATIVE_MAGIC_SIZE);
        out[NEMU_LINK_NATIVE_LENGTH_OFFSET] = old_size - NEMU_LINK_NATIVE_LENGTH_OVERHEAD;
        out[NEMU_LINK_NATIVE_SOURCE_OFFSET] = source;
        out[NEMU_LINK_NATIVE_KIND_OFFSET] = kind;
        memcpy(out + NEMU_LINK_NATIVE_LINK_HEADER_SIZE, plain, 4);
        sys_put_le32(peer_nonce, out + NEMU_LINK_NATIVE_LINK_NONCE_OFFSET);
        memcpy(out + NEMU_LINK_NATIVE_LINK_AUTH_PREFIX_SIZE, plain + 4, length - 4);
    } else if (kind == NEMU_LINK_WIRE_STUDIO &&
               length >= NEMU_LINK_NATIVE_STUDIO_MIN_SIZE - NEMU_LINK_NATIVE_MIN_SIZE &&
               length <= NEMU_LINK_COMPACT_BODY_MAX) {
        old_size = length + NEMU_LINK_NATIVE_MIN_SIZE;
        memcpy(out, "ZmKe", NEMU_LINK_NATIVE_MAGIC_SIZE);
        out[NEMU_LINK_NATIVE_LENGTH_OFFSET] = old_size - NEMU_LINK_NATIVE_LENGTH_OVERHEAD;
        out[NEMU_LINK_NATIVE_SOURCE_OFFSET] = source;
        out[NEMU_LINK_NATIVE_KIND_OFFSET] = kind;
        memcpy(out + NEMU_LINK_NATIVE_LINK_HEADER_SIZE, plain, length);
    } else if ((kind == NEMU_LINK_WIRE_SPLIT_EVENT || kind == NEMU_LINK_WIRE_SPLIT_COMMAND) &&
               length >= NEMU_LINK_NATIVE_SPLIT_MIN_SIZE - NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE &&
               length <= NEMU_LINK_FRAME_MAX_SIZE - NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE && epoch &&
               session && peer_nonce) {
        old_size = length + NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE;
        memcpy(out, "ErEv", NEMU_LINK_NATIVE_MAGIC_SIZE);
        memcpy(out + 4, plain, 4);
        sys_put_le32(epoch, out + 8);
        sys_put_le32(session, out + 12);
        sys_put_le32(peer_nonce, out + 16);
        memcpy(out + NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE, plain + 4, length - 4);
        /* All split frames share the authenticated counter, including raw
         * commands/fallbacks; mixed formats cannot regress the split ledger. */
        sys_put_le32(sequence, out + NEMU_LINK_NATIVE_SPLIT_HEADER_SIZE);
    } else if (kind == NEMU_LINK_WIRE_AUDIO && length >= NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 6U &&
               length <= NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 5U + NEMU_LINK_AUDIO_DATA_MAX &&
               epoch && session && peer_nonce) {
        old_size = length + NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 5U;
        memcpy(out, "EsAu", NEMU_LINK_NATIVE_MAGIC_SIZE);
        out[4] = source;
        out[5] = 0;
        out[6] = kind;
        memcpy(out + NEMU_LINK_NATIVE_AUDIO_HEADER_SIZE - 5U, plain, length);
    } else if (kind == NEMU_LINK_WIRE_AUDIO_CONTROL && length == 2U && plain[0] <= 1U &&
               (plain[1] <= 0x7fU || plain[1] == UINT8_MAX) && epoch && session && peer_nonce) {
        old_size = NEMU_LINK_NATIVE_AUDIO_CONTROL_SIZE;
        memcpy(out, "EsAc", NEMU_LINK_NATIVE_MAGIC_SIZE);
        out[4] = source;
        out[5] = plain[1];
        out[6] = kind;
        out[7] = plain[0];
    } else {
        return -EINVAL;
    }
    if (kind != NEMU_LINK_WIRE_AUDIO && kind != NEMU_LINK_WIRE_AUDIO_CONTROL) {
        memset(out + old_size - NEMU_LINK_NATIVE_CRC_SIZE, 0, NEMU_LINK_NATIVE_CRC_SIZE);
    }
    peer->rx_sequence = sequence;
    *out_size = old_size;
    nemu_link_trace_mark(NEMU_LINK_TRACE_UNPACK_END, old_size);
    return 0;
}
