#include <errno.h>
#include <limits.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nemu_link_split_diag, LOG_LEVEL_INF);
#endif
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zmk/nemu_link/link.h>
#include <zmk/nemu_link/split_frame.h>
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/secure.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/pointing/input_split.h>
#include <zmk/keymap.h>
#if !IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <dt-bindings/zmk/outputs.h>
#include <zmk/endpoints.h>
#endif
#include "link_internal.h"
#include "radio.h"

#define CENTRAL IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
#if defined(NEMU_LINK_SPLIT_HOST_TEST)
#define FRAME_CRC crc32_ieee
#else
#define FRAME_CRC NULL
#endif
/* Split bridge implementation: called under link_lock except zmk_nemu_link_input. */
BUILD_ASSERT(ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT == 2);
BUILD_ASSERT(ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT == 1);

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
struct split_input_diag {
    uint32_t reports, x_events, y_events, errors;
    uint32_t busy, disconnected, queue_full, other;
    int last_error;
    int64_t x_total, y_total;
};
static struct split_input_diag input_diag[NEMU_LINK_LINK_SOURCES];
static void split_diag_input(uint8_t source, uint8_t type, uint16_t code, int32_t value,
                             bool sync) {
    struct split_input_diag *d = &input_diag[source];
    d->reports += sync;
    if (type == INPUT_EV_REL && code == INPUT_REL_X) {
        d->x_events++;
        d->x_total += value;
    } else if (type == INPUT_EV_REL && code == INPUT_REL_Y) {
        d->y_events++;
        d->y_total += value;
    }
}
void nemu_link_split_diag_log(void) {
    for (uint8_t source = 0; source < NEMU_LINK_LINK_SOURCES; source++) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
        if (source != CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID) {
            continue;
        }
#endif
        const struct split_input_diag *d = &input_diag[source];
        LOG_INF(
            "pointer diag source=%u reports=%u x_events=%u y_events=%u x_total=%lld "
            "y_total=%lld errors=%u busy=%u disconnected=%u queue_full=%u other=%u last_error=%d",
            source, d->reports, d->x_events, d->y_events, (long long)d->x_total,
            (long long)d->y_total, d->errors, d->busy, d->disconnected, d->queue_full, d->other,
            d->last_error);
    }
}
#endif

static uint32_t split_tx_sequence[NEMU_LINK_LINK_SOURCES];
static struct nemu_link_split_rx split_rx[NEMU_LINK_LINK_SOURCES];
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT) && IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
static uint8_t split_input_regs[NEMU_LINK_LINK_SOURCES][32];
#endif

static bool split_route(uint8_t source, struct nemu_link_split_frame *f) {
    if (!enabled || source >= NEMU_LINK_LINK_SOURCES || !nemu_link_link_input_allowed(source)) {
        return false;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    const struct nemu_link_link_peer *peer = &central.peers[source];
    if (peer->phase != NEMU_LINK_LINK_UP) {
        return false;
    }
    f->epoch = central.epoch;
    f->session = peer->session;
    f->nonce = peer->peer_nonce;
#else
    if (source != peripheral.source || peripheral.phase != NEMU_LINK_LINK_PERIPHERAL_UP) {
        return false;
    }
    f->epoch = peripheral.epoch;
    f->session = peripheral.session;
    f->nonce = peripheral.peer_nonce;
#endif
    f->source = source;
    return true;
}

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
int nemu_link_split_receive_standalone(const struct rx_frame *rx) {
    struct nemu_link_split_frame f, current = {0};
    if (!rx || !nemu_link_split_decode(&f, rx->bytes, rx->size, FRAME_CRC) ||
        f.kind != NEMU_LINK_SPLIT_COMMAND || !split_route(f.source, &current) ||
        f.epoch != current.epoch || f.session != current.session || f.nonce != current.nonce) {
        return -EINVAL;
    }
    if (!nemu_link_split_accept(&split_rx[f.source], &current, &f)) {
        return -EALREADY;
    }
    if (f.size != 31 || f.data[0] != ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR ||
        !memchr(f.data + 1, 0, 16) || f.data[29] > 1 || f.data[30] > 1) {
        return -EINVAL;
    }
    if (strcmp((const char *)f.data + 1, DEVICE_DT_NAME(DT_NODELABEL(out)))) {
        return -ENOTSUP;
    }
    if (!f.data[30]) {
        return 0;
    }

    uint32_t output = sys_get_le32(f.data + 17);
    enum zmk_transport transport;
    switch (output) {
    case OUT_USB:
#if IS_ENABLED(CONFIG_ZMK_USB)
        transport = ZMK_TRANSPORT_USB;
        break;
#else
        return -ENOTSUP;
#endif
    case OUT_NONE:
        transport = ZMK_TRANSPORT_NONE;
        break;
    default:
        return -ENOTSUP;
    }

    return zmk_endpoint_set_preferred_transport(transport);
}
#endif

static int split_send(struct nemu_link_split_frame *f) {
    if (!split_route(f->source, f)) {
        return -ENOTCONN;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
    int wake_err = idle_wake();
    if (wake_err) {
        return wake_err;
    }
#endif
    if (++split_tx_sequence[f->source] == 0) {
        ++split_tx_sequence[f->source];
    }
    f->sequence = split_tx_sequence[f->source];
    uint8_t bytes[NEMU_LINK_SPLIT_FRAME_SIZE];
    size_t size = nemu_link_split_encode(bytes, f, FRAME_CRC);
    if (!size) {
        return -EINVAL;
    }
    uint8_t encrypted[NEMU_LINK_FRAME_MAX_SIZE];
    size_t encrypted_size = 0;
    int secure_err = nemu_link_secure_wrap(secure_peer(f->source), CENTRAL, f->source, bytes, size,
                                           encrypted, &encrypted_size);
    return secure_err ? secure_err : send_frame(f->source, encrypted, encrypted_size);
}

#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
/* One in-progress report; retain only unsent relative motion under congestion. */
static struct nemu_link_split_frame split_pending_input;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MOTION_BACKPRESSURE)
static struct nemu_link_split_frame split_deferred_motion;
static void split_motion_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(split_motion_work, split_motion_work_handler);
#define MOTION_RETRY_MS 5

/* Merge only matching, complete X/Y-only reports from the same input device.
 * Other event types and arithmetic overflow must not be silently combined. */
static bool split_motion_merge(struct nemu_link_split_frame *dst,
                               const struct nemu_link_split_frame *src) {
    if (src->size < 10 || (src->size - 2) % 8 || src->data[0] != 2) {
        return false;
    }
    for (size_t i = 2; i < src->size; i += 8) {
        uint16_t code = sys_get_le16(src->data + i + 2);
        if (src->data[i + 1] != INPUT_EV_REL || (code != INPUT_REL_X && code != INPUT_REL_Y) ||
            src->data[i] != (i + 8 == src->size)) {
            return false;
        }
    }
    if (!dst->size) {
        *dst = *src;
        return true;
    }
    if (dst->size != src->size || dst->source != src->source || dst->data[1] != src->data[1]) {
        return false;
    }
    for (size_t i = 2; i < src->size; i += 8) {
        if (memcmp(dst->data + i, src->data + i, 4)) {
            return false;
        }
        int64_t sum = (int32_t)sys_get_le32(dst->data + i + 4);
        sum += (int32_t)sys_get_le32(src->data + i + 4);
        if (sum < INT32_MIN || sum > INT32_MAX) {
            return false;
        }
    }
    for (size_t i = 2; i < src->size; i += 8) {
        int64_t sum = (int32_t)sys_get_le32(dst->data + i + 4);
        sum += (int32_t)sys_get_le32(src->data + i + 4);
        sys_put_le32((uint32_t)(int32_t)sum, dst->data + i + 4);
    }
    return true;
}
static int split_motion_flush(void) {
    if (!split_deferred_motion.size) {
        return 0;
    }
    struct nemu_link_split_frame route = {0};
    if (!split_route(split_deferred_motion.source, &route) ||
        route.epoch != split_deferred_motion.epoch ||
        route.session != split_deferred_motion.session ||
        route.nonce != split_deferred_motion.nonce) {
        split_deferred_motion.size = 0;
        return -ENOTCONN;
    }
    int err = split_send(&split_deferred_motion);
    if (!err || err == -ENOTCONN) {
        split_deferred_motion.size = 0;
    }
    return err;
}
static void split_motion_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    k_mutex_lock(&link_lock, K_FOREVER);
    int err = split_motion_flush();
    if (err && split_deferred_motion.size) {
        k_work_reschedule(&split_motion_work, K_MSEC(MOTION_RETRY_MS));
    }
    k_mutex_unlock(&link_lock);
}
#endif
static int split_input_flush(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MOTION_BACKPRESSURE)
    int old_err = split_motion_flush();
    if (old_err) {
        if (split_pending_input.size &&
            split_motion_merge(&split_deferred_motion, &split_pending_input)) {
            split_pending_input.size = 0;
            k_work_reschedule(&split_motion_work, K_MSEC(MOTION_RETRY_MS));
            return 0;
        }
        split_pending_input.size = 0;
        return old_err;
    }
#endif
    if (!split_pending_input.size) {
        return 0;
    }
    int err = split_send(&split_pending_input);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MOTION_BACKPRESSURE)
    if ((err == -ENOMEM || err == -ENOMSG) &&
        split_motion_merge(&split_deferred_motion, &split_pending_input)) {
        k_work_reschedule(&split_motion_work, K_MSEC(MOTION_RETRY_MS));
        err = 0;
    }
#endif
    split_pending_input.size = 0;
    return err;
}
static int split_input(uint8_t reg, uint8_t sync, uint8_t type, uint16_t code, int32_t value) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_HOP)
    if (type == INPUT_EV_KEY || (type == INPUT_EV_REL && value != 0)) {
        search_input(k_uptime_get());
    }
#endif
    struct nemu_link_split_frame route = {0};
    if (!split_route(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, &route)) {
        split_pending_input.size = 0;
        return -ENOTCONN;
    }
    int err = 0;
    if (split_pending_input.size && (split_pending_input.data[1] != reg ||
                                     split_pending_input.size + 8 > NEMU_LINK_SPLIT_DATA_SIZE)) {
        err = split_input_flush();
    }
    if (!split_pending_input.size) {
        split_pending_input = (struct nemu_link_split_frame){
            .kind = NEMU_LINK_SPLIT_EVENT,
            .source = CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID,
            .size = 2,
            .data = {ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT, reg}};
    }
    uint8_t *data = split_pending_input.data + split_pending_input.size;
    data[0] = sync;
    data[1] = type;
    sys_put_le16(code, data + 2);
    sys_put_le32(value, data + 4);
    split_pending_input.size += 8;
    if (sync) {
        int result = split_input_flush();
        if (!err) {
            err = result;
        }
    }
    return err;
}
#endif

/* Explicit byte encoding: independent of C padding, enum and sensor_value ABI. */
#if IS_ENABLED(CONFIG_ZMK_SPLIT) || defined(NEMU_LINK_SPLIT_HOST_TEST)
int nemu_link_split_native_send(app_esb_data_t *packet) {
    struct nemu_link_split_frame f = {0};
    if (!packet || packet->len < sizeof(struct esb_msg_prefix) + 1) {
        return -EINVAL;
    }
    const struct esb_msg_prefix *prefix = (const void *)packet->data;
    if (memcmp(prefix->magic_prefix, "ZmKe", 4) ||
        packet->len != sizeof(*prefix) + prefix->payload_size) {
        return -EINVAL;
    }
    const uint8_t *payload = packet->data + sizeof(*prefix);
    f.source = payload[0];
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    struct zmk_split_transport_central_command cmd = {0};
    if (prefix->payload_size < 2 || prefix->payload_size > 1 + sizeof(cmd)) {
        return -EINVAL;
    }
    memcpy(&cmd, payload + 1, prefix->payload_size - 1);
    f.kind = NEMU_LINK_SPLIT_COMMAND;
    f.data[0] = cmd.type;
    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        if (prefix->payload_size != 1 + sizeof(cmd.type) + sizeof(cmd.data.invoke_behavior)) {
            return -EINVAL;
        }
        memcpy(f.data + 1, cmd.data.invoke_behavior.behavior_dev, 16);
        sys_put_le32(cmd.data.invoke_behavior.param1, f.data + 17);
        sys_put_le32(cmd.data.invoke_behavior.param2, f.data + 21);
        sys_put_le32(cmd.data.invoke_behavior.position, f.data + 25);
        f.data[29] = cmd.data.invoke_behavior.event_source;
        f.data[30] = cmd.data.invoke_behavior.state;
        f.size = 31;
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        f.data[1] = cmd.data.set_physical_layout.layout_idx;
        f.size = 2;
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        f.data[1] = cmd.data.set_hid_indicators.indicators;
        f.size = 2;
        break;
    default:
        return -ENOTSUP;
    }
#else
    struct zmk_split_transport_peripheral_event ev = {0};
    if (prefix->payload_size < 1 + sizeof(ev.type) || prefix->payload_size > 1 + sizeof(ev)) {
        return -EINVAL;
    }
    memcpy(&ev, payload + 1, prefix->payload_size - 1);
    f.kind = NEMU_LINK_SPLIT_EVENT;
    f.data[0] = ev.type;
    switch (ev.type) {
    /* Physical transitions and battery travel in complete STATE snapshots. */
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return 0;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        f.data[1] = ev.data.sensor_event.sensor_index;
        sys_put_le32(ev.data.sensor_event.channel_data.channel, f.data + 2);
        sys_put_le32(ev.data.sensor_event.channel_data.value.val1, f.data + 6);
        sys_put_le32(ev.data.sensor_event.channel_data.value.val2, f.data + 10);
        f.size = 14;
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        k_mutex_lock(&link_lock, K_FOREVER);
        int input_err =
            split_input(ev.data.input_event.reg, ev.data.input_event.sync, ev.data.input_event.type,
                        ev.data.input_event.code, ev.data.input_event.value);
        k_mutex_unlock(&link_lock);
        return input_err;
    default:
        return -ENOTSUP;
    }
#endif
    k_mutex_lock(&link_lock, K_FOREVER);
#if !IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    int flush_err = split_input_flush(); /* Preserve ordering before a sensor event. */
    if (flush_err) {
        k_mutex_unlock(&link_lock);
        return flush_err;
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_HOP)
    if (ev.data.sensor_event.channel_data.value.val1 ||
        ev.data.sensor_event.channel_data.value.val2) {
        search_input(k_uptime_get());
    }
#endif
#endif
    int err = split_send(&f);
    k_mutex_unlock(&link_lock);
    return err;
}
#endif

int zmk_nemu_link_sensor_event(uint8_t sensor_index, uint32_t channel,
                               const struct sensor_value *value) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    if (!value || sensor_index >= ZMK_KEYMAP_SENSORS_LEN) {
        return -EINVAL;
    }
    struct nemu_link_split_frame frame = {
        .kind = NEMU_LINK_SPLIT_EVENT,
        .source = CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID,
        .size = 14,
        .data = {ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT, sensor_index},
    };
    sys_put_le32(channel, frame.data + 2);
    sys_put_le32((uint32_t)value->val1, frame.data + 6);
    sys_put_le32((uint32_t)value->val2, frame.data + 10);

    k_mutex_lock(&link_lock, K_FOREVER);
    int err = split_input_flush();
    if (!err && (value->val1 || value->val2)) {
        search_input(k_uptime_get());
    }
    if (!err) {
        err = split_send(&frame);
    }
    k_mutex_unlock(&link_lock);
    return err;
#else
    ARG_UNUSED(sensor_index);
    ARG_UNUSED(channel);
    ARG_UNUSED(value);
    return -ENOTSUP;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT) || defined(NEMU_LINK_SPLIT_HOST_TEST)
int nemu_link_split_receive(const struct rx_frame *rx) {
    struct nemu_link_split_frame f, current = {0};
    if (!nemu_link_split_decode(&f, rx->bytes, rx->size, FRAME_CRC) ||
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        f.source != rx->pipe || f.kind != NEMU_LINK_SPLIT_EVENT ||
#else
        f.kind != NEMU_LINK_SPLIT_COMMAND ||
#endif
        !split_route(f.source, &current) || f.epoch != current.epoch ||
        f.session != current.session || f.nonce != current.nonce) {
        return -EINVAL;
    }
    if (!nemu_link_split_accept(&split_rx[f.source], &current, &f)) {
        return -EALREADY;
    }
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    struct zmk_split_transport_peripheral_event ev = {.type = f.data[0]};
    switch (ev.type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        if (f.size != 14 || f.data[1] >= ZMK_KEYMAP_SENSORS_LEN) {
            return -EINVAL;
        }
        ev.data.sensor_event.sensor_index = f.data[1];
        ev.data.sensor_event.channel_data.channel = sys_get_le32(f.data + 2);
        ev.data.sensor_event.channel_data.value.val1 = (int32_t)sys_get_le32(f.data + 6);
        ev.data.sensor_event.channel_data.value.val2 = (int32_t)sys_get_le32(f.data + 10);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        if (f.size < 10 || (f.size - 2) % 8) {
            return -EINVAL;
        }
        for (uint8_t i = 2; i < f.size; i += 8) {
            if (f.data[i] > 1) {
                return -EINVAL;
            }
        }
        ev.data.input_event.reg = f.data[1];
        for (uint8_t i = 2; i < f.size; i += 8) {
            ev.data.input_event.sync = f.data[i];
            ev.data.input_event.type = f.data[i + 1];
            ev.data.input_event.code = sys_get_le16(f.data + i + 2);
            ev.data.input_event.value = (int32_t)sys_get_le32(f.data + i + 4);
            int err = zmk_split_transport_central_peripheral_event_handler(active_transport,
                                                                           f.source, ev);
            if (err) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
                input_diag[f.source].errors++;
#endif
                return err;
            }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
            split_diag_input(f.source, ev.data.input_event.type, ev.data.input_event.code,
                             ev.data.input_event.value, ev.data.input_event.sync);
#endif
        }
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        split_input_regs[f.source][f.data[1] / 8] |= BIT(f.data[1] % 8);
#endif
        return 0;
    default:
        return -ENOTSUP;
    }
    return zmk_split_transport_central_peripheral_event_handler(active_transport, f.source, ev);
#else
    struct zmk_split_transport_central_command cmd = {.type = f.data[0]};
    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        if (f.size != 31 || !memchr(f.data + 1, 0, 16) || f.data[30] > 1) {
            return -EINVAL;
        }
        memcpy(cmd.data.invoke_behavior.behavior_dev, f.data + 1, 16);
        cmd.data.invoke_behavior.param1 = sys_get_le32(f.data + 17);
        cmd.data.invoke_behavior.param2 = sys_get_le32(f.data + 21);
        cmd.data.invoke_behavior.position = sys_get_le32(f.data + 25);
        cmd.data.invoke_behavior.event_source = f.data[29];
        cmd.data.invoke_behavior.state = f.data[30];
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        if (f.size != 2) {
            return -EINVAL;
        }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
        /* This half is also a Studio central: its selection is versioned data.
         * A handshake command must not replace an offline edit or create a
         * spurious unsaved edit before Studio compares revisions. Stock wire
         * positions already support different selections during convergence. */
        return 0;
#else
        cmd.data.set_physical_layout.layout_idx = f.data[1];
        break;
#endif
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        if (f.size != 2) {
            return -EINVAL;
        }
        cmd.data.set_hid_indicators.indicators = f.data[1];
        break;
    default:
        return -ENOTSUP;
    }
    return zmk_split_transport_peripheral_command_handler(zmk_nemu_link_peripheral_transport(),
                                                          cmd);
#endif
}
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH) || IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
int zmk_nemu_link_input(uint8_t reg, const struct input_event *evt) {
    k_mutex_lock(&link_lock, K_FOREVER);
    int err = split_input(reg, evt->sync, evt->type, evt->code, evt->value);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_DIAGNOSTIC)
    split_diag_input(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, evt->type, evt->code, evt->value,
                     evt->sync);
    if (err) {
        struct split_input_diag *d = &input_diag[CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID];
        d->errors++;
        d->last_error = err;
        d->busy += err == -EBUSY;
        d->disconnected += err == -ENOTCONN;
        d->queue_full += err == -ENOMSG || err == -ENOMEM;
        d->other += err != -EBUSY && err != -ENOTCONN && err != -ENOMSG && err != -ENOMEM;
    }
#endif
    k_mutex_unlock(&link_lock);
    return err;
}
#endif

void nemu_link_split_discard_input(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL)
    split_pending_input.size = 0;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MOTION_BACKPRESSURE)
    split_deferred_motion.size = 0;
    k_work_cancel_delayable(&split_motion_work);
#endif
#endif
}

void nemu_link_split_reset(void) {
    memset(split_rx, 0, sizeof(split_rx));
    nemu_link_split_discard_input();
}

void nemu_link_split_source_lost(uint8_t source) {
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT) && IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    for (unsigned int reg = 0; reg < 256; reg++) {
        if (split_input_regs[source][reg / 8] & BIT(reg % 8)) {
            zmk_input_split_peripheral_disconnected(reg);
        }
    }
    memset(split_input_regs[source], 0, sizeof(split_input_regs[source]));
#else
    ARG_UNUSED(source);
#endif
}
