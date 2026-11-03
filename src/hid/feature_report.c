#include <errno.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <zmk/nemu_link/hid_feature_report.h>
#include <zmk/nemu_link/key_provision.h>
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#include <zmk/nemu_link/battery_status.h>
_Static_assert(20 + NEMU_LINK_BATTERY_STATUS_SIZE <= NEMU_LINK_HID_FEATURE_REPORT_SIZE,
               "Battery metadata must fit in the feature report");
#endif

#define COMMAND_KEY_WRITE 1

int nemu_link_hid_feature_report_get(uint8_t out[NEMU_LINK_HID_FEATURE_REPORT_SIZE]) {
    struct nemu_link_key_provision_status status;
    nemu_link_key_provision_status_get(&status);
    memset(out, 0, NEMU_LINK_HID_FEATURE_REPORT_SIZE);
    out[0] = NEMU_LINK_HID_FEATURE_REPORT_ID;
    out[1] = NEMU_LINK_HID_FEATURE_PROTOCOL_VERSION;
    out[2] = (uint8_t)status.state;
    out[3] = status.sequence;
    out[4] = status.error;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    out[6] = NEMU_LINK_HID_FEATURE_ROLE_CENTRAL;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE)
    out[7] = CONFIG_ZMK_NEMU_LINK_DONGLE_ID;
#else
    out[7] = 1;
#endif
#else
    out[6] = NEMU_LINK_HID_FEATURE_ROLE_PERIPHERAL;
    out[7] = CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID;
#endif
    memcpy(out + 16, NEMU_LINK_HID_FEATURE_MAGIC, 4);
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    nemu_link_battery_status_get(out + 20);
#endif
    for (int i = 0; i < 4; ++i) {
        out[8 + i] = (uint8_t)(status.challenge >> (i * 8));
        out[12 + i] = (uint8_t)(status.fingerprint >> (i * 8));
    }
    memcpy(out + 27, status.address, sizeof(status.address));
    out[31] = 3; /* Capabilities: shared radio address and channel provisioning. */
    out[32] = status.channel_count;
    memcpy(out + 33, status.channels, sizeof(status.channels));
    return 0;
}

int nemu_link_hid_feature_report_set(const uint8_t *report, size_t length) {
    if (length != NEMU_LINK_HID_FEATURE_REPORT_SIZE || !report ||
        report[0] != NEMU_LINK_HID_FEATURE_REPORT_ID ||
        report[1] != NEMU_LINK_HID_FEATURE_PROTOCOL_VERSION || report[2] != COMMAND_KEY_WRITE) {
        return -EINVAL;
    }
    for (int i = 32; i < NEMU_LINK_HID_FEATURE_REPORT_SIZE; ++i) {
        if (report[i]) {
            return -EINVAL;
        }
    }
    uint32_t response = 0;
    for (int i = 0; i < 4; ++i) {
        response |= (uint32_t)report[4 + i] << (i * 8);
    }
    uint8_t address[NEMU_LINK_RADIO_ADDRESS_SIZE];
    memcpy(address, report + 24, sizeof(address));
    uint8_t any = 0;
    for (size_t i = 0; i < sizeof(address); ++i) {
        any |= address[i];
    }
    uint8_t channel_count = report[28];
    uint8_t channels[NEMU_LINK_RADIO_CHANNEL_COUNT_MAX] = {report[29], report[30], report[31]};
    if (!channel_count || channel_count > NEMU_LINK_RADIO_CHANNEL_COUNT_MAX) {
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
    for (uint8_t i = channel_count; i < NEMU_LINK_RADIO_CHANNEL_COUNT_MAX; i++) {
        if (channels[i]) {
            return -EINVAL;
        }
    }
    return nemu_link_key_provision_submit(report[3], response, report + 8, any ? address : NULL,
                                          channels, channel_count);
}
