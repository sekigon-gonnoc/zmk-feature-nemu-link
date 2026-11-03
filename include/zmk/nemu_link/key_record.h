#pragma once

#include <stdbool.h>
#include <stdint.h>

#define NEMU_LINK_KEY_SIZE 16U
#define NEMU_LINK_RADIO_ADDRESS_SIZE 4U
#define NEMU_LINK_KEY_RECORD_SIZE 36U
#define NEMU_LINK_KEY_RECORD_VERSION 1U
#define NEMU_LINK_RADIO_CHANNEL_MAX 80U
#define NEMU_LINK_RADIO_CHANNEL_COUNT_MAX 3U
#define NEMU_LINK_DEFAULT_RADIO_CHANNEL_COUNT 1U
#define NEMU_LINK_DEFAULT_RADIO_CHANNELS                                                           \
    { 48U, 0U, 0U }

/* The unreleased format v1 stores magic/version/length/count/channels/key/address/CRC.
 * The address and channels are public radio metadata, not secrets. Other versions are rejected.
 */
void nemu_link_key_record_encode(uint8_t out[NEMU_LINK_KEY_RECORD_SIZE],
                                 const uint8_t key[NEMU_LINK_KEY_SIZE],
                                 const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                 const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                 uint8_t channel_count);
bool nemu_link_key_record_decode(const uint8_t in[NEMU_LINK_KEY_RECORD_SIZE],
                                 uint8_t key[NEMU_LINK_KEY_SIZE],
                                 uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                 uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                 uint8_t *channel_count);
uint32_t nemu_link_key_record_fingerprint(const uint8_t key[NEMU_LINK_KEY_SIZE]);
