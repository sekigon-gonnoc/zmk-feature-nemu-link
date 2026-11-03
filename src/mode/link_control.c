#include <errno.h>
#include <zephyr/kernel.h>
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/internal.h>
static const struct zmk_nemu_link_observer *observer;
int zmk_nemu_link_register_observer(const struct zmk_nemu_link_observer *value) {
    if (!value) {
        return -EINVAL;
    }
    if (observer) {
        return -EALREADY;
    }
    observer = value;
    return 0;
}
void nemu_link_notify_source_lost(uint8_t source) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
    nemu_link_mode_source_lost(source);
#endif
    if (observer && observer->source_lost) {
        observer->source_lost(source);
    }
}
void nemu_link_notify_mode_requested(bool esb) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_CHORD)
    nemu_link_mode_requested(esb);
#endif
    if (observer && observer->mode_requested) {
        observer->mode_requested(esb);
    }
}
void nemu_link_notify_mode_changed(bool esb, int error) {
    if (observer && observer->mode_changed) {
        observer->mode_changed(esb, error);
    }
}
int zmk_nemu_link_request_peer_mode(bool esb) {
#if IS_ENABLED(CONFIG_BOARD_NEMU_LINK_DONGLE)
    if (esb) {
        return -ENOTSUP;
    }
    nemu_link_request_ble_peers();
    return 0;
#else
    return -ENOTSUP;
#endif
}
#if !IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH)
int zmk_nemu_link_request_mode(bool esb) { return -ENOTSUP; }
bool zmk_nemu_link_owns_radio(void) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT)
    /* A standalone keyboard owns its normal local input and USB endpoint. */
    return false;
#elif IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
    return nemu_link_peripheral_transport_active();
#else
    return false;
#endif
}
#endif
