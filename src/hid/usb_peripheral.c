#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>
#include <zmk/nemu_link/hid_feature_report.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
#include <zmk/nemu_link/control.h>
#endif

LOG_MODULE_REGISTER(nemu_link_usb_peripheral, CONFIG_ZMK_LOG_LEVEL);

static const uint8_t descriptor[] = {
    0x06, 0x00, 0xff, /* Vendor-defined usage page 0xff00. */
    0x09, 0x01, 0xa1, 0x01, 0x85, NEMU_LINK_HID_FEATURE_REPORT_ID,
    0x09, 0x02, 0x15, 0x00, 0x26, 0xff,
    0x00, 0x75, 0x08, 0x95, 0x24, 0xb1,
    0x02, 0xc0,
};
static uint8_t status_report[NEMU_LINK_HID_FEATURE_REPORT_SIZE];

static int get_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                      uint8_t **data) {
    ARG_UNUSED(dev);
    if (setup->wValue != (0x300 | NEMU_LINK_HID_FEATURE_REPORT_ID))
        return -ENOTSUP;
    int rc = nemu_link_hid_feature_report_get(status_report);
    if (rc)
        return rc;
    *len = sizeof(status_report);
    *data = status_report;
    return 0;
}

static int set_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                      uint8_t **data) {
    ARG_UNUSED(dev);
    if (setup->wValue != (0x300 | NEMU_LINK_HID_FEATURE_REPORT_ID))
        return -ENOTSUP;
    return nemu_link_hid_feature_report_set(*data, *len);
}

static const struct hid_ops ops = {.get_report = get_report, .set_report = set_report};

static int nemu_link_hid_peripheral_init(void) {
    const struct device *dev = device_get_binding("HID_0");
    if (!dev || !device_is_ready(dev)) {
        LOG_ERR("USB HID_0 device is unavailable");
        return -ENODEV;
    }
    usb_hid_register_device(dev, descriptor, sizeof(descriptor), &ops);
    int rc = usb_hid_init(dev);
    if (rc) {
        LOG_ERR("Failed to initialize Nemu-link USB HID: %d", rc);
    }
    return rc;
}
SYS_INIT(nemu_link_hid_peripheral_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);

static int nemu_link_usb_peripheral_enable(void) {
    int rc = usb_enable(NULL);
    if (rc == -EALREADY) {
        LOG_INF("USB device stack was already enabled");
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
        zmk_nemu_link_set_receiver_ready(true);
#endif
        return 0;
    }
    if (rc) {
        LOG_ERR("Failed to enable standalone USB device stack: %d", rc);
        return rc;
    }
    LOG_INF("Standalone USB device stack enabled");
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    zmk_nemu_link_set_receiver_ready(true);
#endif
    return 0;
}
/* Start after the USB HID registrations (95) and before later application init. */
SYS_INIT(nemu_link_usb_peripheral_enable, APPLICATION, 97);
