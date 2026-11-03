#include <errno.h>
#include <string.h>
#include <zephyr/storage/flash_map.h>
#include <zmk/nemu_link/key_store.h>

#define KEY_PARTITION FIXED_PARTITION_ID(nemu_link_key_partition)
BUILD_ASSERT(FIXED_PARTITION_SIZE(nemu_link_key_partition) == 4096,
             "ESB key must occupy one flash page");

static int read_record(uint8_t record[NEMU_LINK_KEY_RECORD_SIZE]) {
    const struct flash_area *area;
    int rc = flash_area_open(KEY_PARTITION, &area);
    if (rc) {
        return rc;
    }
    rc = flash_area_read(area, 0, record, NEMU_LINK_KEY_RECORD_SIZE);
    flash_area_close(area);
    return rc;
}

int nemu_link_key_store_read(uint8_t key[NEMU_LINK_KEY_SIZE]) {
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE];
    int rc = read_record(record);
    if (rc) {
        memset(record, 0, sizeof(record));
        return rc;
    }
    bool valid = nemu_link_key_record_decode(record, key, NULL, NULL, NULL);
    memset(record, 0, sizeof(record));
    return valid ? 0 : -ENOENT;
}

int nemu_link_key_store_read_radio_address(uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE]) {
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE], key[NEMU_LINK_KEY_SIZE],
        stored_address[NEMU_LINK_RADIO_ADDRESS_SIZE];
    int rc = read_record(record);
    if (rc) {
        memset(record, 0, sizeof(record));
        return rc;
    }
    bool valid = nemu_link_key_record_decode(record, key, stored_address, NULL, NULL);
    if (valid) {
        memcpy(address, stored_address, sizeof(stored_address));
        memset(record, 0, sizeof(record));
        memset(key, 0, sizeof(key));
        memset(stored_address, 0, sizeof(stored_address));
        return 0;
    }
    memset(record, 0, sizeof(record));
    memset(key, 0, sizeof(key));
    memset(stored_address, 0, sizeof(stored_address));
    return -ENOENT;
}

int nemu_link_key_store_read_radio_channels(uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                                            uint8_t *channel_count) {
    if (!channels || !channel_count) {
        return -EINVAL;
    }
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE], key[NEMU_LINK_KEY_SIZE];
    int rc = read_record(record);
    if (rc) {
        memset(record, 0, sizeof(record));
        return rc;
    }
    bool valid = nemu_link_key_record_decode(record, key, NULL, channels, channel_count);
    memset(record, 0, sizeof(record));
    memset(key, 0, sizeof(key));
    return valid ? 0 : -ENOENT;
}

int nemu_link_key_store_fingerprint(uint32_t *fingerprint) {
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE], key[NEMU_LINK_KEY_SIZE];
    int rc = read_record(record);
    if (rc) {
        memset(record, 0, sizeof(record));
        return rc;
    }
    if (!nemu_link_key_record_decode(record, key, NULL, NULL, NULL)) {
        memset(record, 0, sizeof(record));
        return -ENOENT;
    }
    *fingerprint = nemu_link_key_record_fingerprint(key);
    memset(key, 0, sizeof(key));
    memset(record, 0, sizeof(record));
    return 0;
}

int nemu_link_key_store_write(const uint8_t key[NEMU_LINK_KEY_SIZE],
                              const uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE],
                              const uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX],
                              uint8_t channel_count) {
    const struct flash_area *area;
    uint8_t record[NEMU_LINK_KEY_RECORD_SIZE], verify[NEMU_LINK_KEY_RECORD_SIZE],
        read_key[NEMU_LINK_KEY_SIZE];
    uint8_t target_address[NEMU_LINK_RADIO_ADDRESS_SIZE],
        read_address[NEMU_LINK_RADIO_ADDRESS_SIZE];
    uint8_t read_channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX], read_channel_count;
    if (address) {
        memcpy(target_address, address, sizeof(target_address));
    } else {
        int address_rc = nemu_link_key_store_read_radio_address(target_address);
        if (address_rc) {
            return address_rc;
        }
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
    nemu_link_key_record_encode(record, key, target_address, channels, channel_count);
    int rc = flash_area_open(KEY_PARTITION, &area);
    if (rc)
        goto cleanup;
    if (area->fa_size != 4096) {
        rc = -EINVAL;
        goto done;
    }
    rc = flash_area_erase(area, 0, area->fa_size);
    if (!rc)
        rc = flash_area_write(area, 0, record, sizeof(record));
    if (!rc)
        rc = flash_area_read(area, 0, verify, sizeof(verify));
    if (!rc &&
        (!nemu_link_key_record_decode(verify, read_key, read_address, read_channels,
                                      &read_channel_count) ||
         memcmp(read_key, key, NEMU_LINK_KEY_SIZE) ||
         memcmp(read_address, target_address, NEMU_LINK_RADIO_ADDRESS_SIZE) ||
         read_channel_count != channel_count || memcmp(read_channels, channels, channel_count)))
        rc = -EIO;
done:
    flash_area_close(area);
cleanup:
    memset(record, 0, sizeof(record));
    memset(verify, 0, sizeof(verify));
    memset(read_key, 0, sizeof(read_key));
    memset(target_address, 0, sizeof(target_address));
    memset(read_address, 0, sizeof(read_address));
    memset(read_channels, 0, sizeof(read_channels));
    read_channel_count = 0;
    return rc;
}
