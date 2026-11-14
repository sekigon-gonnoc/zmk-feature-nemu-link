/* Registered after activity, before behavior listeners in the app target. */
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nemu_link_input_router, LOG_LEVEL_INF);
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/endpoints.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/split/peripheral.h>
#endif
#include <zmk/nemu_link/control.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
#include <zmk/nemu_link/studio_zmk.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
extern const struct zmk_listener zmk_listener_nemu_link_mode_control;
#endif

static int route_input(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *pos = as_zmk_position_state_changed(eh);
    if (pos) {
        /* Track physical state even when BLE behaviors capture the event. */
        zmk_nemu_link_position(pos);
        if (!zmk_nemu_link_owns_radio()) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD) && !IS_ENABLED(CONFIG_ZMK_SPLIT)
            zmk_listener_nemu_link_mode_control.callback(eh);
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
            nemu_link_studio_track_position(pos);
#endif
            return ZMK_EV_EVENT_BUBBLE;
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
        zmk_listener_nemu_link_mode_control.callback(eh);
#endif
        return ZMK_EV_EVENT_HANDLED;
    }
    const struct zmk_sensor_event *sensor = as_zmk_sensor_event(eh);
    if (!zmk_nemu_link_owns_radio()) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
        /* In standalone mode the USB endpoint consumes the event locally.
         * Other endpoint choices send the encoder event to the dongle. */
        if (sensor && zmk_endpoint_get_selected().transport != ZMK_TRANSPORT_USB) {
            if (sensor->channel_data_size == 1) {
                struct sensor_value value;
                const uint8_t *channel_data =
                    (const uint8_t *)sensor + offsetof(struct zmk_sensor_event, channel_data);
                memcpy(&value, channel_data + offsetof(struct zmk_sensor_channel_data, value),
                       sizeof(value));
                int err = zmk_nemu_link_sensor_event(sensor->sensor_index,
                                                     sensor->channel_data[0].channel, &value);
                if (err && err != -ENOTCONN) {
                    LOG_WRN("sensor report failed index=%u err=%d", sensor->sensor_index, err);
                }
            } else {
                LOG_WRN("unsupported sensor channel count=%u", sensor->channel_data_size);
            }
        }
#endif
        return ZMK_EV_EVENT_BUBBLE;
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    if (sensor && sensor->channel_data_size == 1) {
        struct zmk_split_transport_peripheral_event event = {
            .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT,
            .data.sensor_event = {.sensor_index = sensor->sensor_index,
                                  .channel_data = sensor->channel_data[0]}};
        int err = zmk_split_peripheral_report_event(&event);
        if (err) {
            LOG_WRN("sensor report failed index=%u err=%d", sensor->sensor_index, err);
        }
    } else if (sensor) {
        LOG_WRN("unsupported sensor channel count=%u", sensor->channel_data_size);
    }
#endif
    return ZMK_EV_EVENT_HANDLED;
}
ZMK_LISTENER(nemu_link_input_router, route_input);
ZMK_SUBSCRIPTION(nemu_link_input_router, zmk_position_state_changed);
ZMK_SUBSCRIPTION(nemu_link_input_router, zmk_sensor_event);

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB) && !IS_ENABLED(CONFIG_ZMK_SPLIT)
#define DT_DRV_COMPAT zmk_input_listener
static uint8_t fixed_esb_input_log_budget = 5;

static void fixed_esb_input_handler(struct input_event *evt, void *user_data, uint8_t reg) {
    ARG_UNUSED(user_data);
    if (zmk_endpoint_get_selected().transport == ZMK_TRANSPORT_USB) {
        return;
    }

    int err = zmk_nemu_link_input(reg, evt);
    if (err && err != -ENOTCONN) {
        LOG_WRN("fixed ESB input route failed reg=%u type=%u code=%u err=%d", reg, evt->type,
                evt->code, err);
    } else if (evt->sync && fixed_esb_input_log_budget) {
        --fixed_esb_input_log_budget;
        LOG_INF("fixed ESB input routed reg=%u type=%u code=%u value=%d err=%d", reg, evt->type,
                evt->code, evt->value, err);
    }
}

#define FIXED_ESB_INPUT_CALLBACK(inst)                                                             \
    COND_CODE_1(                                                                                   \
        DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, device), zmk_behavior_input_two_axis), (),        \
        (COND_CODE_1(                                                                              \
            DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, device), zmk_behavior_mouse_key_press), (),   \
            (static void fixed_esb_input_handler_##inst(struct input_event *evt,                   \
                                                        void *user_data) {                         \
                fixed_esb_input_handler(evt, user_data, inst);                                     \
            } INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(inst, device)),                  \
                                    fixed_esb_input_handler_##inst, NULL);))))

DT_INST_FOREACH_STATUS_OKAY(FIXED_ESB_INPUT_CALLBACK)
#endif
