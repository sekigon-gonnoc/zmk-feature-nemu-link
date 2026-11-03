#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>
#include <zmk/nemu_link/key_provision.h>
#include <zmk/nemu_link/key_store.h>

static atomic_t busy;
static uint8_t pending_key[NEMU_LINK_KEY_SIZE];
static uint8_t pending_address[NEMU_LINK_RADIO_ADDRESS_SIZE];
static bool pending_address_set;
static uint8_t pending_channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX];
static uint8_t pending_channel_count;
static uint8_t last_sequence;
static enum nemu_link_key_provision_state state;
static uint8_t error_code;
static uint32_t challenge;
static uint32_t fingerprint;
static uint8_t radio_address[NEMU_LINK_RADIO_ADDRESS_SIZE];
static const uint8_t default_radio_channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX] =
    NEMU_LINK_DEFAULT_RADIO_CHANNELS;
static uint8_t radio_channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX] = NEMU_LINK_DEFAULT_RADIO_CHANNELS;
static uint8_t radio_channel_count = NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT;
static bool initialized;

static void write_key_work(struct k_work *work);
K_WORK_DEFINE(key_work, write_key_work);

static void initialize(void) {
    if (initialized)
        return;
    challenge = sys_rand32_get();
    if (!challenge)
        challenge = 1;
    state = nemu_link_key_store_fingerprint(&fingerprint) ? NEMU_LINK_KEY_PROVISION_EMPTY
                                                          : NEMU_LINK_KEY_PROVISION_READY;
    (void)nemu_link_key_store_read_radio_address(radio_address);
    if (nemu_link_key_store_read_radio_channels(radio_channels, &radio_channel_count)) {
        memcpy(radio_channels, default_radio_channels, sizeof(radio_channels));
        radio_channel_count = NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT;
    }
    initialized = true;
}

static void write_key_work(struct k_work *work) {
    ARG_UNUSED(work);
    int rc = nemu_link_key_store_write(pending_key, pending_address_set ? pending_address : NULL,
                                       pending_channels, pending_channel_count);
    memset(pending_key, 0, sizeof(pending_key));
    memset(pending_address, 0, sizeof(pending_address));
    pending_address_set = false;
    memset(pending_channels, 0, sizeof(pending_channels));
    pending_channel_count = 0;
    if (!rc)
        rc = nemu_link_key_store_fingerprint(&fingerprint);
    if (!rc)
        rc = nemu_link_key_store_read_radio_address(radio_address);
    if (!rc)
        rc = nemu_link_key_store_read_radio_channels(radio_channels, &radio_channel_count);
    error_code = rc ? (uint8_t)(-rc) : 0;
    state = rc ? NEMU_LINK_KEY_PROVISION_ERROR : NEMU_LINK_KEY_PROVISION_READY;
    atomic_clear(&busy);
}

void nemu_link_key_provision_status_get(struct nemu_link_key_provision_status *status) {
    initialize();
    *status = (struct nemu_link_key_provision_status){
        .state = state,
        .sequence = last_sequence,
        .error = error_code,
        .challenge = challenge,
        .fingerprint = fingerprint,
    };
    memcpy(status->address, radio_address, sizeof(status->address));
    status->channel_count = radio_channel_count;
    memcpy(status->channels, radio_channels, sizeof(status->channels));
}

int nemu_link_key_provision_submit(uint8_t sequence, uint32_t response,
                                   const uint8_t key[NEMU_LINK_KEY_SIZE],
                                   const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                   const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                   uint8_t channel_count) {
    initialize();
    if (!sequence || !key) {
        return -EINVAL;
    }
    if (!channels || !channel_count || channel_count > NEMU_LINK_RADIO_CHANNEL_COUNT_MAX) {
        return -EINVAL;
    }
    for (uint8_t i = 0; i < channel_count; i++) {
        if (channels[i] > NEMU_LINK_RADIO_CHANNEL_MAX || (channels[i] & 1U)) {
            return -EINVAL;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (channels[i] == channels[j]) {
                return -EINVAL;
            }
        }
    }
    if (response != challenge) {
        return -EACCES;
    }
    if (!atomic_cas(&busy, 0, 1))
        return -EBUSY;
    memcpy(pending_key, key, NEMU_LINK_KEY_SIZE);
    if (address) {
        uint8_t any = 0, all = 0xff;
        for (size_t i = 0; i < NEMU_LINK_RADIO_ADDRESS_SIZE; i++) {
            any |= address[i];
            all &= address[i];
        }
        if (!any || all == 0xff) {
            memset(pending_key, 0, sizeof(pending_key));
            atomic_clear(&busy);
            return -EINVAL;
        }
        memcpy(pending_address, address, NEMU_LINK_RADIO_ADDRESS_SIZE);
        pending_address_set = true;
    } else {
        memset(pending_address, 0, sizeof(pending_address));
        pending_address_set = false;
    }
    memcpy(pending_channels, channels, sizeof(pending_channels));
    pending_channel_count = channel_count;
    last_sequence = sequence;
    state = NEMU_LINK_KEY_PROVISION_BUSY;
    error_code = 0;
    challenge = sys_rand32_get();
    if (!challenge)
        challenge = 1;
    if (k_work_submit(&key_work) < 0) {
        memset(pending_key, 0, sizeof(pending_key));
        memset(pending_address, 0, sizeof(pending_address));
        pending_address_set = false;
        memset(pending_channels, 0, sizeof(pending_channels));
        pending_channel_count = 0;
        state = NEMU_LINK_KEY_PROVISION_ERROR;
        error_code = EIO;
        atomic_clear(&busy);
        return -EIO;
    }
    return 0;
}
