#pragma once
#include <stdint.h>
#include <zmk/nemu_link/key_record.h>

enum nemu_link_key_provision_state {
    NEMU_LINK_KEY_PROVISION_EMPTY = 0,
    NEMU_LINK_KEY_PROVISION_BUSY = 1,
    NEMU_LINK_KEY_PROVISION_READY = 2,
    NEMU_LINK_KEY_PROVISION_ERROR = 3,
};

struct nemu_link_key_provision_status {
    enum nemu_link_key_provision_state state;
    uint8_t sequence;
    uint8_t error;
    uint32_t challenge;
    uint32_t fingerprint;
    uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE];
    uint8_t channel_count;
    uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX];
};

void nemu_link_key_provision_status_get(struct nemu_link_key_provision_status *status);
/* Queue a key/address/channel-list write after challenge validation. */
int nemu_link_key_provision_submit(uint8_t sequence, uint32_t response,
                                   const uint8_t key[NEMU_LINK_KEY_SIZE],
                                   const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                   const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                   uint8_t channel_count);
