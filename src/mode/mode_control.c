#define DT_DRV_COMPAT zmk_behavior_nemu_link_mode
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/internal.h>
#include <zmk/nemu_link/link_keys.h>
#include <zmk/physical_layouts.h>
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB) &&                                 \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
#include <zmk/endpoints.h>
#endif
#include <errno.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/split/central.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH) && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) &&            \
    IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/bluetooth/service.h>
BUILD_ASSERT(sizeof(DEVICE_DT_NAME(DT_NODELABEL(nl_mode))) <= ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN,
             "Nemu Link mode behavior name exceeds the BLE split payload field");
#endif
LOG_MODULE_REGISTER(nemu_link_mode, LOG_LEVEL_INF);
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
/* The chord lists are DT arrays; build their compact bitmaps once at startup. */
#define NEMU_LINK_CONFIG_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(sekigon_nemu_link)
static struct nemu_link_link_keys down[2];
static struct nemu_link_link_keys nemu_link_mode_keys[2];
static bool fired[2];
static struct k_spinlock chord_lock;
static int mode_key_position_count;

static int mode_key_add(uint8_t source, uint32_t position) {
    if (position >= (uint32_t)mode_key_position_count || position >= NEMU_LINK_LINK_KEY_COUNT) {
        LOG_ERR("Mode chord position %u is outside the keymap", position);
        nemu_link_mode_keys[0] = (struct nemu_link_link_keys){0};
        nemu_link_mode_keys[1] = (struct nemu_link_link_keys){0};
        return -EINVAL;
    }
    if (nemu_link_link_keys_test(nemu_link_mode_keys[source], position)) {
        LOG_ERR("Duplicate mode chord position %u for source %u", position, source);
        nemu_link_mode_keys[0] = (struct nemu_link_link_keys){0};
        nemu_link_mode_keys[1] = (struct nemu_link_link_keys){0};
        return -EINVAL;
    }
    nemu_link_link_keys_set(&nemu_link_mode_keys[source], position, true);
    return 0;
}

#define NEMU_LINK_MODE_KEY_INIT_0(node, prop, idx)                                                 \
    do {                                                                                           \
        int err = mode_key_add(0, DT_PROP_BY_IDX(node, prop, idx));                                \
        if (err) {                                                                                 \
            return err;                                                                            \
        }                                                                                          \
    } while (0)
#define NEMU_LINK_MODE_KEY_INIT_1(node, prop, idx)                                                 \
    do {                                                                                           \
        int err = mode_key_add(1, DT_PROP_BY_IDX(node, prop, idx));                                \
        if (err) {                                                                                 \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

static int nemu_link_mode_keys_init(void) {
    uint32_t const *map;
    mode_key_position_count = zmk_physical_layouts_get_selected_to_stock_position_map(&map);
    ARG_UNUSED(map);
    if (mode_key_position_count <= 0 || mode_key_position_count > NEMU_LINK_LINK_KEY_COUNT) {
        LOG_ERR("Invalid keymap position count: %d", mode_key_position_count);
        return -EINVAL;
    }
    COND_CODE_1(DT_NODE_HAS_PROP(NEMU_LINK_CONFIG_NODE, source_0_switch_positions),
                (DT_FOREACH_PROP_ELEM_SEP(NEMU_LINK_CONFIG_NODE, source_0_switch_positions,
                                          NEMU_LINK_MODE_KEY_INIT_0, (;))),
                ());
    COND_CODE_1(DT_NODE_HAS_PROP(NEMU_LINK_CONFIG_NODE, source_1_switch_positions),
                (DT_FOREACH_PROP_ELEM_SEP(NEMU_LINK_CONFIG_NODE, source_1_switch_positions,
                                          NEMU_LINK_MODE_KEY_INIT_1, (;))),
                ());
    return 0;
}
SYS_INIT(nemu_link_mode_keys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void chord_timeout(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(source0_work, chord_timeout);
static K_WORK_DELAYABLE_DEFINE(source1_work, chord_timeout);
static struct k_work_delayable *const timers[] = {&source0_work, &source1_work};

static bool complete(uint8_t source) {
    return nemu_link_link_keys_any(nemu_link_mode_keys[source]) &&
           nemu_link_link_keys_equal(down[source], nemu_link_mode_keys[source]);
}
void nemu_link_mode_source_lost(uint8_t source) {
    if (source >= 2) {
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&chord_lock);
    down[source] = (struct nemu_link_link_keys){0};
    fired[source] = false;
    k_work_cancel_delayable(timers[source]);
    k_spin_unlock(&chord_lock, key);
}
void nemu_link_mode_requested(bool target) {
    ARG_UNUSED(target);
    k_spinlock_key_t key = k_spin_lock(&chord_lock);
    for (uint8_t source = 0; source < 2; source++) {
        fired[source] = nemu_link_link_keys_any(down[source]);
        k_work_cancel_delayable(timers[source]);
    }
    k_spin_unlock(&chord_lock, key);
}
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH)
static void request_mode(bool target, bool broadcast) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    if (broadcast && target && !zmk_nemu_link_owns_radio()) {
        struct zmk_behavior_binding binding = {
            .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(nl_mode)),
            .param1 = 1,
        };
        struct zmk_behavior_binding_event event = {
            .position = 0,
            .timestamp = k_uptime_get(),
            .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
        };
        int err = zmk_split_central_invoke_behavior(0, &binding, event, true);
        if (err) {
            LOG_WRN("BLE peer mode request failed: %d", err);
        }
    }
#else
    ARG_UNUSED(broadcast);
#endif
    zmk_nemu_link_request_mode(target);
}
#endif
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
static void chord_timeout(struct k_work *work) {
    for (uint8_t source = 0; source < 2; source++) {
        if (work != &timers[source]->work) {
            continue;
        }
        k_spinlock_key_t key = k_spin_lock(&chord_lock);
        bool trigger = complete(source) && !fired[source];
        if (trigger) {
            fired[source] = true;
        }
        k_spin_unlock(&chord_lock, key);
        if (!trigger) {
            return;
        }
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB) &&                                 \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
        int err = nemu_link_mode_command(2);
        if (err) {
            LOG_WRN("Standalone USB/ESB chord failed: %d", err);
        }
#else
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
        zmk_nemu_link_request_peer_mode(false);
#else
        bool active = zmk_nemu_link_owns_radio();
        if (source == CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID) {
            request_mode(!active, !active);
        } else if (!active) {
            request_mode(true, true);
        }
#endif
#endif
        return;
    }
}
static int position_listener(const zmk_event_t *event) {
    const struct zmk_position_state_changed *pos = as_zmk_position_state_changed(event);
    if (!pos) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    uint8_t source;
    int delay = CONFIG_ZMK_NEMU_LINK_MODE_CHORD_MS;
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    source = pos->source;
#else
    bool local = pos->source == ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
    bool active = zmk_nemu_link_owns_radio();
    if (!local && (!IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || active)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    source = local ? CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID : 1 - CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID;
    delay = local ? (active ? CONFIG_ZMK_NEMU_LINK_MODE_RECOVERY_MS
                            : CONFIG_ZMK_NEMU_LINK_MODE_CHORD_MS)
                  : CONFIG_ZMK_NEMU_LINK_MODE_REMOTE_CHORD_MS;
#endif
    const uint32_t *map;
    int count = zmk_physical_layouts_get_selected_to_stock_position_map(&map);
    if (count <= 0 || pos->position >= (uint32_t)count) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    uint32_t position = map[pos->position];
    if (source >= 2 || !nemu_link_link_keys_test(nemu_link_mode_keys[source], position)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    k_spinlock_key_t key = k_spin_lock(&chord_lock);
    bool was_complete = complete(source);
    nemu_link_link_keys_set(&down[source], position, pos->state);
    if (!pos->state) {
        if (!nemu_link_link_keys_any(down[source])) {
            fired[source] = false;
        }
        k_work_cancel_delayable(timers[source]);
    } else if (!was_complete && complete(source) && !fired[source]) {
        k_work_reschedule(timers[source], K_MSEC(delay));
    }
    k_spin_unlock(&chord_lock, key);
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(nemu_link_mode_control, position_listener);
ZMK_SUBSCRIPTION(nemu_link_mode_control, zmk_position_state_changed);

#endif

/* Split: 0 BLE, 1 ESB, 2 toggle. Standalone fixed ESB: 0 USB, 1 NONE, 2 toggle. */
int nemu_link_mode_command(uint32_t mode) {
    if (mode > 2) {
        return -ENOTSUP;
    }
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB) &&                                 \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
    enum zmk_transport target =
        mode == 2
            ? (zmk_endpoint_get_preferred_transport() == ZMK_TRANSPORT_USB ? ZMK_TRANSPORT_NONE
                                                                           : ZMK_TRANSPORT_USB)
            : (mode == 1 ? ZMK_TRANSPORT_NONE : ZMK_TRANSPORT_USB);
    LOG_INF("Standalone endpoint mode command mode=%u target=%u", mode, target);
    return zmk_endpoint_set_preferred_transport(target);
#else
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    return mode == 1 ? 0 : zmk_nemu_link_request_peer_mode(false);
#else
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH)
    bool target = mode == 2 ? !zmk_nemu_link_owns_radio() : mode == 1;
    request_mode(target, target);
    return 0;
#else
    return -ENOTSUP;
#endif
#endif
#endif
}

static int on_mode_pressed(struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    int err = nemu_link_mode_command(binding->param1);
    if (err) {
        return err;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}
static int on_mode_released(struct zmk_behavior_binding *binding,
                            struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata mode_values[] = {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_USB) &&                                 \
    IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
    {.value = 0, .display_name = "USB", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
    {.value = 1, .display_name = "ESB (USB disabled)", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
    {.value = 2, .display_name = "Toggle USB/ESB", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
#else
    {.value = 0, .display_name = "BLE", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
    {.value = 1, .display_name = "ESB", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
    {.value = 2, .display_name = "Toggle BLE/ESB", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE},
#endif
};
static const struct behavior_parameter_metadata_set mode_set = {
    .param1_values = mode_values,
    .param1_values_len = ARRAY_SIZE(mode_values),
};
static const struct behavior_parameter_metadata mode_metadata = {.sets_len = 1, .sets = &mode_set};
#endif
static const struct behavior_driver_api mode_behavior_api = {
    .binding_pressed = on_mode_pressed,
    .binding_released = on_mode_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &mode_metadata,
#endif
};
BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &mode_behavior_api);
