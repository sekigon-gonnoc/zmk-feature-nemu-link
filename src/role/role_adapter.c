#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/transport/peripheral.h>
#include "radio.h"

LOG_MODULE_REGISTER(nemu_link_role_adapter, LOG_LEVEL_INF);

int nemu_link_native_send(app_esb_data_t *packet);
void nemu_link_radio_event(app_esb_event_t *event);
int nemu_link_link_enable(bool enabled);

static bool active;
static uint16_t message_id;
bool nemu_link_peripheral_transport_active(void) { return active; }

static int native_send(uint8_t source, const void *message, size_t length) {
    /* The link adapter converts this local ZMK envelope to the M9 split wire. */
    uint8_t tx[5 + 1 + sizeof(struct zmk_split_transport_central_command)];
    memcpy(tx, "ZmKe", 4);
    tx[4] = length + 1;
    tx[5] = source;
    memcpy(tx + 6, message, length);
    app_esb_data_t packet = {.pipe = source, .data = tx, .len = length + 6, .msg_id = ++message_id};
    return nemu_link_native_send(&packet);
}
static int set_enabled(bool enabled) {
    LOG_INF("split transport enable request value=%u active=%u", enabled, active);
    int err = nemu_link_link_enable(enabled);
    if (!err) {
        active = enabled;
    }
    LOG_INF("split transport enable result value=%u active=%u err=%d", enabled, active, err);
    return err;
}
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static zmk_split_transport_central_status_changed_cb_t status_cb;
static int send_command(uint8_t source, struct zmk_split_transport_central_command cmd) {
    size_t size;
    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        size = 0;
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        size = sizeof(cmd.data.invoke_behavior);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        size = sizeof(cmd.data.set_physical_layout);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        size = sizeof(cmd.data.set_hid_indicators);
        break;
    default:
        return -ENOTSUP;
    }
    return native_send(source, &cmd, sizeof(cmd.type) + size);
}
static int sources(uint8_t *ids) {
    for (uint8_t i = 0; i < CONFIG_ZMK_NEMU_LINK_PERIPHERAL_COUNT; i++) {
        ids[i] = i;
    }
    return CONFIG_ZMK_NEMU_LINK_PERIPHERAL_COUNT;
}
static struct zmk_split_transport_status status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = active,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED};
}
static int set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    status_cb = cb;
    return 0;
}
static const struct zmk_split_transport_central_api api = {.send_command = send_command,
                                                           .get_available_source_ids = sources,
                                                           .set_enabled = set_enabled,
                                                           .get_status = status,
                                                           .set_status_callback =
                                                               set_status_callback};
ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(nemu_link_central, &api, CONFIG_ZMK_NEMU_LINK_PRIORITY);
static void notify(struct k_work *work) {
    if (status_cb) {
        status_cb(&nemu_link_central, status());
    }
}
static K_WORK_DEFINE(notify_work, notify);
static int adapter_init(void) {
    int err = nemu_link_radio_init(APP_ESB_MODE_PRX, nemu_link_radio_event);
    if (!err) {
        k_work_submit(&notify_work);
    }
    return err;
}
#else
static zmk_split_transport_peripheral_status_changed_cb_t status_cb;
static int report_event(const struct zmk_split_transport_peripheral_event *event) {
    size_t size;
    switch (event->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return 0;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        size = sizeof(event->data.input_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        size = sizeof(event->data.sensor_event);
        break;
    default:
        return -ENOTSUP;
    }
    return native_send(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, event, sizeof(event->type) + size);
}
static struct zmk_split_transport_status status(void) {
    return (struct zmk_split_transport_status){
        .available = active || IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB),
        .enabled = active,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED};
}
static int set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    status_cb = cb;
    return 0;
}
static const struct zmk_split_transport_peripheral_api api = {.report_event = report_event,
                                                              .set_enabled = set_enabled,
                                                              .get_status = status,
                                                              .set_status_callback =
                                                                  set_status_callback};
ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(nemu_link_peripheral, &api, CONFIG_ZMK_NEMU_LINK_PRIORITY);
const struct zmk_split_transport_peripheral *zmk_nemu_link_peripheral_transport(void) {
    return &nemu_link_peripheral;
}
static void notify(struct k_work *work) {
    if (status_cb) {
        status_cb(&nemu_link_peripheral, status());
    }
}
static K_WORK_DEFINE(notify_work, notify);
static int adapter_init(void) {
    int err = nemu_link_radio_init(APP_ESB_MODE_PTX, nemu_link_radio_event);
    if (!err) {
        k_work_submit(&notify_work);
    }
    return err;
}
#endif
SYS_INIT(adapter_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
