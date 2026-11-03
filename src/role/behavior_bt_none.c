/* The USB/ESB dongle has no Bluetooth host profiles. Consume BT bindings
 * instead of letting a missing device fall through to the base layer. */
#define DT_DRV_COMPAT zmk_behavior_bluetooth
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <dt-bindings/zmk/bt.h>
#include <zmk/behavior.h>
#include <zmk/nemu_link/internal.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static int discard_bt(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MULTI_DONGLE) && binding->param1 == BT_NXT_CMD) {
        int err = nemu_link_request_next_dongle();
        return err ? err : ZMK_BEHAVIOR_OPAQUE;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
/* Keep BT bindings editable/synchronizable in Studio. The BLE central owns
 * profile bounds; this device accepts the uint8 profile index but does no IO. */
static const struct behavior_parameter_value_metadata commands[] = {
    {.display_name = "Next Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_NXT_CMD},
    {.display_name = "Previous Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_PRV_CMD},
    {.display_name = "Clear All Profiles",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_CLR_ALL_CMD},
    {.display_name = "Clear Selected Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_CLR_CMD},
};
static const struct behavior_parameter_value_metadata indexed_commands[] = {
    {.display_name = "Select Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_SEL_CMD},
    {.display_name = "Disconnect Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BT_DISC_CMD},
};
static const struct behavior_parameter_value_metadata profile[] = {
    {.display_name = "Profile",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range = {.min = 0, .max = UINT8_MAX}},
};
static const struct behavior_parameter_metadata_set sets[] = {
    {.param1_values = commands, .param1_values_len = ARRAY_SIZE(commands)},
    {.param1_values = indexed_commands,
     .param1_values_len = ARRAY_SIZE(indexed_commands),
     .param2_values = profile,
     .param2_values_len = ARRAY_SIZE(profile)},
};
static const struct behavior_parameter_metadata metadata = {
    .sets = sets,
    .sets_len = ARRAY_SIZE(sets),
};
#endif
static const struct behavior_driver_api behavior_bt_none_api = {
    .binding_pressed = discard_bt,
    .binding_released = discard_bt,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};
BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_bt_none_api);
#endif
