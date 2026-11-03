#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <zmk/nemu_link/key_record.h>

int nemu_link_key_store_read(uint8_t key[NEMU_LINK_KEY_SIZE]);
/* Returns -ENOENT when no valid key/address/channel record is provisioned. */
int nemu_link_key_store_read_radio_address(uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE]);
int nemu_link_key_store_read_radio_channels(uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                            uint8_t *channel_count);
int nemu_link_key_store_write(const uint8_t key[NEMU_LINK_KEY_SIZE],
                              const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                              const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                              uint8_t channel_count);
int nemu_link_key_store_fingerprint(uint32_t *fingerprint);
