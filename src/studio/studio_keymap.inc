/* Included only in the generated copy of the pinned upstream keymap.c. */
#include <zephyr/sys/byteorder.h>
#include <zmk/physical_layouts.h>
#include <zmk/events/position_state_changed.h>
#include <studio_schema.h>
K_MUTEX_DEFINE(nemu_link_studio_map_lock);
static const char *const studio_stock_names[] = {ZMK_KEYMAP_LAYERS_FOREACH_SEP(LAYER_NAME, (, ))};
static const struct nemu_link_studio_schema studio_schema = {.layers = ZMK_KEYMAP_LAYERS_LEN,
                                                        .positions = ZMK_KEYMAP_LEN,
                                                        .layouts = STUDIO_LAYOUT_COUNT,
                                                        .behaviors = studio_behaviors,
                                                        .behavior_count =
                                                            ARRAY_SIZE(studio_behaviors)};
BUILD_ASSERT(ZMK_KEYMAP_LEN > 0 && ZMK_KEYMAP_LEN <= NEMU_LINK_STUDIO_MAX_POSITIONS);
BUILD_ASSERT(ZMK_KEYMAP_LAYERS_LEN > 0 && ZMK_KEYMAP_LAYERS_LEN <= NEMU_LINK_STUDIO_MAX_LAYERS);
BUILD_ASSERT(CONFIG_ZMK_KEYMAP_LAYER_NAME_MAX_LEN <= 64);
const struct nemu_link_studio_schema *nemu_link_studio_schema(void) { return &studio_schema; }
void nemu_link_studio_schema_digest(uint8_t digest[32]) {
    /* Build identity uses semantic CRC32; runtime content retains XOR32. */
    memcpy(digest, studio_build_contract, 32);
}
static unsigned int studio_active_count(void) {
    unsigned int n = 0;
    while (n < ZMK_KEYMAP_LAYERS_LEN && keymap_layer_orders[n] != ZMK_KEYMAP_LAYER_ID_INVAL) {
        n++;
    }
    return n;
}
static bool studio_layer_present(zmk_keymap_layer_id_t id) {
    for (unsigned int i = 0; i < studio_active_count(); i++) {
        if (keymap_layer_orders[i] == id) {
            return true;
        }
    }
    return false;
}
static int studio_saved_layout = -1;
static int studio_behavior_index(const char *name, size_t len) {
    if (!name) {
        return -1;
    }
    for (size_t i = 0; i < ARRAY_SIZE(studio_behaviors); i++) {
        if (strlen(studio_behaviors[i].name) == len &&
            !memcmp(studio_behaviors[i].name, name, len)) {
            return i;
        }
    }
    return -1;
}
static bool studio_binding_valid(const struct zmk_behavior_binding *binding) {
    int i = studio_behavior_index(binding->behavior_dev,
                                  binding->behavior_dev ? strlen(binding->behavior_dev) : 0);
    if (i < 0 || (studio_behaviors[i].params < 1 && binding->param1) ||
        (studio_behaviors[i].params < 2 && binding->param2) ||
        ((studio_layer_params[i] & 1) && binding->param1 >= ZMK_KEYMAP_LAYERS_LEN) ||
        ((studio_layer_params[i] & 2) && binding->param2 >= ZMK_KEYMAP_LAYERS_LEN)) {
        return false;
    }
    if (!zmk_behavior_get_binding(binding->behavior_dev)) {
        return false;
    }
    int err = zmk_behavior_validate_binding(binding);
    /* Some zero-argument behaviors do not provide parameter metadata. The
     * generated Studio schema already declares that they accept no params;
     * ENODEV here means metadata is absent, not that the behavior is missing
     * (the device lookup above succeeded). */
    return !err ||
           (err == -ENODEV && !studio_behaviors[i].params && !binding->param1 && !binding->param2);
}
int nemu_link_studio_record_read(uint16_t index, bool stock, uint8_t b[NEMU_LINK_STUDIO_RECORD_MAX],
                            size_t *size) {
    if (index >= nemu_link_studio_record_count(&studio_schema)) {
        return -EINVAL;
    }
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!index) {
        int layout = zmk_physical_layouts_get_selected();
        if (studio_saved_layout < 0) {
            studio_saved_layout = layout;
        }
        memcpy(b, "DSC1", 4);
        sys_put_le16(stock ? STUDIO_STOCK_LAYOUT : layout, b + 4);
        b[6] = ZMK_KEYMAP_LAYERS_LEN;
        const uint8_t *order = stock ? studio_stock_order : keymap_layer_orders;
        b[7] = 0;
        for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
            if (order[i] != ZMK_KEYMAP_LAYER_ID_INVAL) {
                b[10 + b[7]++] = order[i];
            }
        }
        sys_put_le16(ZMK_KEYMAP_LEN, b + 8);
        *size = 10u + b[7];
    } else {
        unsigned int layer = (index - 1) / (ZMK_KEYMAP_LEN + 1),
                     item = (index - 1) % (ZMK_KEYMAP_LEN + 1);
        const char *name;
        const struct zmk_behavior_binding *binding = NULL;
        if (!item) {
            name = stock ? studio_stock_names[layer] : zmk_keymap_layer_names[layer];
        } else {
            binding = stock ? &zmk_stock_keymap[layer][item - 1] : &zmk_keymap[layer][item - 1];
            name = binding->behavior_dev;
        }
        size_t len = name ? strnlen(name, NEMU_LINK_STUDIO_MAX_NAME + 1) : NEMU_LINK_STUDIO_MAX_NAME + 1;
        if (len > NEMU_LINK_STUDIO_MAX_NAME) {
            k_mutex_unlock(&nemu_link_studio_map_lock);
            return -EINVAL;
        }
        b[0] = len;
        memcpy(b + 1, name, len);
        *size = 1 + len;
        if (binding) {
            sys_put_le32(binding->param1, b + *size);
            sys_put_le32(binding->param2, b + *size + 4);
            *size += 8;
        }
    }
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return nemu_link_studio_record_valid(&studio_schema, index, b, *size) ? 0 : -EINVAL;
}
bool nemu_link_studio_record_changed(uint16_t index) {
    if (!index || studio_saved_layout != zmk_physical_layouts_get_selected()) {
        return true;
    }
    if (index >= nemu_link_studio_record_count(&studio_schema)) {
        return false;
    }
    unsigned int layer = (index - 1) / (ZMK_KEYMAP_LEN + 1);
    unsigned int item = (index - 1) % (ZMK_KEYMAP_LEN + 1);
    return item ? !!(zmk_keymap_layer_pending_changes[layer][(item - 1) / 8] & BIT((item - 1) % 8))
                : !!(changed_layer_names & BIT(layer));
}
int nemu_link_studio_record_write(uint16_t index, const uint8_t *b, size_t size) {
    if (!nemu_link_studio_record_valid(&studio_schema, index, b, size)) {
        return -EINVAL;
    }
    uint8_t old[NEMU_LINK_STUDIO_RECORD_MAX];
    size_t old_size;
    int err = nemu_link_studio_record_read(index, false, old, &old_size);
    if (!err && old_size == size && !memcmp(old, b, size)) {
        return 0;
    }
    char key[32];
    unsigned int layer = index ? (index - 1) / (ZMK_KEYMAP_LEN + 1) : 0;
    unsigned int item = index ? (index - 1) % (ZMK_KEYMAP_LEN + 1) : 0;
    uint8_t order[ZMK_KEYMAP_LAYERS_LEN];
    struct zmk_behavior_binding_setting setting = {0};
    struct zmk_behavior_binding binding = {0};
    if (!index) {
        memset(order, ZMK_KEYMAP_LAYER_ID_INVAL, sizeof(order));
        memcpy(order, b + 10, b[7]);
        uint8_t layout = sys_get_le16(b + 4);
        err = zmk_physical_layouts_select(layout);
        if (!err) {
            err = zmk_physical_layouts_save_selected();
        }
        if (!err) {
            err = settings_save_one(LAYER_ORDER_SETTINGS_KEY, order, sizeof(order));
        }
        if (!err) {
            studio_saved_layout = layout;
        }
    } else if (!item) {
        snprintf(key, sizeof(key), LAYER_NAME_SETTINGS_KEY, layer);
        err = settings_save_one(key, b[0] ? b + 1 : (const uint8_t *)"", MAX(1, b[0]));
    } else {
        snprintf(key, sizeof(key), LAYER_BINDING_SETTINGS_KEY, layer, item - 1);
        int i = studio_behavior_index((const char *)b + 1, b[0]);
        if (i < 0) {
            return -ENOTSUP;
        }
        binding = (struct zmk_behavior_binding){.behavior_dev = studio_behaviors[i].name,
                                                .param1 = sys_get_le32(b + 1 + b[0]),
                                                .param2 = sys_get_le32(b + 5 + b[0])};
        if (!studio_binding_valid(&binding)) {
            return -EINVAL;
        }
        setting.behavior_local_id = zmk_behavior_get_local_id(binding.behavior_dev);
        if (setting.behavior_local_id == UINT16_MAX) {
            return -ENODEV;
        }
        setting.param1 = binding.param1;
        setting.param2 = binding.param2;
        size_t n = sizeof(setting);
        if (!setting.param2) {
            n -= 4;
            if (!setting.param1) {
                n -= 4;
            }
        }
        err = settings_save_one(key, &setting, n);
    }
    if (err) {
        return err;
    }
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!index) {
        memcpy(keymap_layer_orders, order, sizeof(order));
        memcpy(settings_layer_orders, order, sizeof(order));
        _zmk_keymap_layer_default = order[0];
        uint32_t active = 0;
        for (unsigned int i = 0; i < b[7]; i++) {
            active |= BIT(order[i]);
        }
        _zmk_keymap_layer_state &= active;
        _zmk_keymap_layer_locks &= active;
    } else if (!item) {
        memcpy(zmk_keymap_layer_names[layer], b + 1, b[0]);
        zmk_keymap_layer_names[layer][b[0]] = 0;
        WRITE_BIT(changed_layer_names, layer, 0);
    } else {
        zmk_keymap[layer][item - 1] = (struct zmk_behavior_binding){
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_LOCAL_IDS_IN_BINDINGS)
            .local_id = setting.behavior_local_id,
#endif
            .behavior_dev = binding.behavior_dev,
            .param1 = setting.param1,
            .param2 = setting.param2};
        WRITE_BIT(zmk_keymap_layer_pending_changes[layer][(item - 1) / 8], (item - 1) % 8, 0);
    }
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return 0;
}
int nemu_link_studio_native_save(void) {
    int err = zmk_physical_layouts_save_selected();
    if (!err) {
        err = studio_native_zmk_keymap_save_changes();
    }
    if (!err) {
        studio_saved_layout = zmk_physical_layouts_get_selected();
    }
    return err;
}
int nemu_link_studio_native_dirty(void) {
    return (studio_saved_layout >= 0 &&
            studio_saved_layout != zmk_physical_layouts_get_selected()) ||
           changed_layer_names || studio_native_zmk_keymap_check_unsaved_changes();
}
int nemu_link_studio_native_discard(void) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
        strlcpy(zmk_keymap_layer_names[i], studio_stock_names[i],
                sizeof(zmk_keymap_layer_names[i]));
    }
    int ret = studio_native_zmk_keymap_discard_changes();
    k_mutex_unlock(&nemu_link_studio_map_lock);
    if (!ret && studio_saved_layout >= 0) {
        ret = zmk_physical_layouts_select(studio_saved_layout);
    }
    return ret;
}
int zmk_keymap_save_changes(void) { return nemu_link_studio_save(false); }
int zmk_keymap_reset_settings(void) { return nemu_link_studio_save(true); }
int zmk_keymap_discard_changes(void) { return nemu_link_studio_discard(); }
int zmk_keymap_check_unsaved_changes(void) { return nemu_link_studio_dirty(); }
/* Input execution remains the original ZMK keymap/behavior implementation.
 * Track raw BLE positions before combo capture only for ordinary mode-change
 * release events; never synthesize HID codes or implement tap/hold here. */
static struct {
    bool down;
    uint8_t source;
} studio_pressed[ZMK_KEYMAP_LEN];
void nemu_link_studio_track_position(const struct zmk_position_state_changed *pos) {
    if (pos->position >= ZMK_KEYMAP_LEN) {
        return;
    }
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    studio_pressed[pos->position].down = pos->state;
    studio_pressed[pos->position].source = pos->source;
    k_mutex_unlock(&nemu_link_studio_map_lock);
}
void nemu_link_studio_release_all(void) {
    for (int p = 0; p < ZMK_KEYMAP_LEN; p++) {
        k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
        bool down = studio_pressed[p].down;
        uint8_t source = studio_pressed[p].source;
        studio_pressed[p].down = false;
        k_mutex_unlock(&nemu_link_studio_map_lock);
        if (down) {
            raise_zmk_position_state_changed((struct zmk_position_state_changed){
                .position = p, .source = source, .state = false, .timestamp = k_uptime_get()});
        }
    }
}

int zmk_keymap_set_layer_binding_at_idx(zmk_keymap_layer_id_t layer_id, uint16_t binding_idx,
                                        struct zmk_behavior_binding binding) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }
    if (!studio_binding_valid(&binding)) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EINVAL;
    }
    int ret = studio_original_zmk_keymap_set_layer_binding_at_idx(layer_id, binding_idx, binding);
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int zmk_keymap_move_layer(zmk_keymap_layer_index_t start_idx, zmk_keymap_layer_index_t dest_idx) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }
    if (start_idx >= studio_active_count() || dest_idx >= studio_active_count()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EINVAL;
    }
    int ret = studio_original_zmk_keymap_move_layer(start_idx, dest_idx);
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int zmk_keymap_add_layer(void) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }

    int ret = studio_original_zmk_keymap_add_layer();
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int zmk_keymap_remove_layer(zmk_keymap_layer_index_t index) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }
    if (studio_active_count() <= 1) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EINVAL;
    }
    int ret = studio_original_zmk_keymap_remove_layer(index);
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int zmk_keymap_restore_layer(zmk_keymap_layer_id_t id, zmk_keymap_layer_index_t at_index) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }
    if (studio_layer_present(id) || at_index > studio_active_count() ||
        studio_active_count() == ZMK_KEYMAP_LAYERS_LEN) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EINVAL;
    }
    int ret = studio_original_zmk_keymap_restore_layer(id, at_index);
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
int zmk_keymap_set_layer_name(zmk_keymap_layer_id_t id, const char *name, size_t size) {
    k_mutex_lock(&nemu_link_studio_map_lock, K_FOREVER);
    if (!nemu_link_studio_edit_allowed()) {
        k_mutex_unlock(&nemu_link_studio_map_lock);
        return -EBUSY;
    }

    int ret = studio_original_zmk_keymap_set_layer_name(id, name, size);
    k_mutex_unlock(&nemu_link_studio_map_lock);
    return ret;
}
