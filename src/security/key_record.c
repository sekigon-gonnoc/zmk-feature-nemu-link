#include <string.h>

#include <zmk/nemu_link/key_record.h>

#define NEMU_LINK_KEY_CRC_OFFSET 24U
#define NEMU_LINK_KEY_RECORD_KEY_OFFSET 10U
#define NEMU_LINK_KEY_RECORD_ADDRESS_OFFSET 26U
#define NEMU_LINK_KEY_RECORD_CRC_OFFSET 30U

static const uint8_t key_magic[4] = {'N', 'L', 'K', 'Y'};

static uint32_t record_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (unsigned int bit = 0; bit < 8U; bit++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xedb88320) : 0U);
        }
    }
    return ~crc;
}

static uint32_t key_fingerprint_crc(const uint8_t *data, size_t size) {
    return record_crc32(data, size);
}

uint32_t nemu_link_key_record_fingerprint(const uint8_t key[NEMU_LINK_KEY_SIZE]) {
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE];
    memset(record, 0xff, sizeof(record));
    memcpy(record, key_magic, sizeof(key_magic));
    record[4] = 1U;
    record[5] = NEMU_LINK_KEY_SIZE;
    memcpy(record + 8, key, NEMU_LINK_KEY_SIZE);
    uint32_t fingerprint = key_fingerprint_crc(record, NEMU_LINK_KEY_CRC_OFFSET);
    memset(record, 0, sizeof(record));
    return fingerprint;
}

void nemu_link_key_record_encode(uint8_t out[NEMU_LINK_KEY_RECORD_SIZE],
                                 const uint8_t key[NEMU_LINK_KEY_SIZE],
                                 const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                 const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                 uint8_t channel_count) {
    memset(out, 0xff, NEMU_LINK_KEY_RECORD_SIZE);
    memcpy(out, key_magic, sizeof(key_magic));
    out[4] = NEMU_LINK_KEY_RECORD_VERSION;
    out[5] = NEMU_LINK_KEY_SIZE;
    out[6] = channel_count;
    memcpy(out + 7, channels, channel_count);
    memcpy(out + NEMU_LINK_KEY_RECORD_KEY_OFFSET, key, NEMU_LINK_KEY_SIZE);
    memcpy(out + NEMU_LINK_KEY_RECORD_ADDRESS_OFFSET, address, NEMU_LINK_RADIO_ADDRESS_SIZE);

    uint32_t crc = record_crc32(out, NEMU_LINK_KEY_RECORD_CRC_OFFSET);
    for (unsigned int i = 0; i < 4U; i++) {
        out[NEMU_LINK_KEY_RECORD_CRC_OFFSET + i] = (uint8_t)(crc >> (8U * i));
    }
}

bool nemu_link_key_record_decode(const uint8_t in[NEMU_LINK_KEY_RECORD_SIZE],
                                 uint8_t key[NEMU_LINK_KEY_SIZE],
                                 uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                                 uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                 uint8_t *channel_count) {
    if (memcmp(in, key_magic, sizeof(key_magic)) != 0 || in[5] != NEMU_LINK_KEY_SIZE ||
        in[4] != NEMU_LINK_KEY_RECORD_VERSION || !in[6] ||
        in[6] > NEMU_LINK_RADIO_CHANNEL_COUNT_MAX) {
        return false;
    }

    for (uint8_t i = 0; i < NEMU_LINK_RADIO_CHANNEL_COUNT_MAX; i++) {
        if (i < in[6]) {
            if (in[7U + i] > NEMU_LINK_RADIO_CHANNEL_MAX || (in[7U + i] & 1U)) {
                return false;
            }
            for (uint8_t j = 0; j < i; j++) {
                if (in[7U + i] == in[7U + j]) {
                    return false;
                }
            }
        } else if (in[7U + i] != 0xffU) {
            return false;
        }
    }
    if (in[34] != 0xffU || in[35] != 0xffU) {
        return false;
    }
    uint32_t stored_crc = 0;
    for (unsigned int i = 0; i < 4U; i++) {
        stored_crc |= (uint32_t)in[NEMU_LINK_KEY_RECORD_CRC_OFFSET + i] << (8U * i);
    }
    if (stored_crc != record_crc32(in, NEMU_LINK_KEY_RECORD_CRC_OFFSET)) {
        return false;
    }
    if (address) {
        memcpy(address, in + NEMU_LINK_KEY_RECORD_ADDRESS_OFFSET, NEMU_LINK_RADIO_ADDRESS_SIZE);
    }
    if (channels) {
        memset(channels, 0, NEMU_LINK_RADIO_CHANNEL_COUNT_MAX);
        memcpy(channels, in + 7, in[6]);
    }
    if (channel_count) {
        *channel_count = in[6];
    }
    memcpy(key, in + NEMU_LINK_KEY_RECORD_KEY_OFFSET, NEMU_LINK_KEY_SIZE);
    return true;
}
