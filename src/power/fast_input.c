#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/endpoints.h>
#endif
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/idle.h>

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_ROLE_PERIPHERAL) && IS_ENABLED(CONFIG_ZMK_NEMU_LINK_IDLE)
static struct nemu_link_fast_input_state fast_input = {
    .timeout_ms = CONFIG_ZMK_NEMU_LINK_FAST_INPUT_TIMEOUT_MS,
};
static struct k_spinlock fast_input_lock;

int zmk_nemu_link_fast_input_register(const void *device, bool enabled) {
    uint64_t now = k_uptime_get();
    k_spinlock_key_t key = k_spin_lock(&fast_input_lock);
    int err = nemu_link_fast_input_set(&fast_input, device, enabled, now);
    k_spin_unlock(&fast_input_lock, key);
    return err;
}

int zmk_nemu_link_fast_input_activity(const void *device) {
    uint64_t now = k_uptime_get();
    k_spinlock_key_t key = k_spin_lock(&fast_input_lock);
    int err = nemu_link_fast_input_activity(&fast_input, device, now);
    k_spin_unlock(&fast_input_lock, key);
    return err;
}

bool nemu_link_fast_input_any_active(uint64_t now) {
    bool esb_active = zmk_nemu_link_owns_radio();
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    bool endpoint_non_usb = true;
#else
    bool endpoint_non_usb = zmk_endpoint_get_selected().transport != ZMK_TRANSPORT_USB;
#endif
    k_spinlock_key_t key = k_spin_lock(&fast_input_lock);
    bool active =
        nemu_link_fast_input_active_for_route(&fast_input, now, esb_active, endpoint_non_usb);
    k_spin_unlock(&fast_input_lock, key);
    return active;
}

void nemu_link_fast_input_radio_started(uint64_t now) {
    k_spinlock_key_t key = k_spin_lock(&fast_input_lock);
    nemu_link_fast_input_refresh(&fast_input, now);
    k_spin_unlock(&fast_input_lock, key);
}
#else
int zmk_nemu_link_fast_input_register(const void *device, bool enabled) {
    ARG_UNUSED(device);
    ARG_UNUSED(enabled);
    return -ENOTSUP;
}

int zmk_nemu_link_fast_input_activity(const void *device) {
    ARG_UNUSED(device);
    return -ENOTSUP;
}

bool nemu_link_fast_input_any_active(uint64_t now) {
    ARG_UNUSED(now);
    return false;
}

void nemu_link_fast_input_radio_started(uint64_t now) { ARG_UNUSED(now); }
#endif
