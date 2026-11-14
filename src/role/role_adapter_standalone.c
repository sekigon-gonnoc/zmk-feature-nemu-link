#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include "radio.h"

int nemu_link_link_enable(bool enabled);
void nemu_link_radio_event(app_esb_event_t *event);

static bool active;

bool nemu_link_peripheral_transport_active(void) { return active; }

int nemu_link_peripheral_transport_set_enabled(bool enabled) {
    int err = nemu_link_link_enable(enabled);
    if (!err) {
        active = enabled;
    }
    return err;
}

static int adapter_init(void) {
    int err = nemu_link_radio_init(APP_ESB_MODE_PTX, nemu_link_radio_event);
    if (err) {
        return err;
    }

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_MODE_SWITCH)
    /* Runtime mode switching starts in BLE mode unless BOOT_ESB requests ESB. */
    return 0;
#else
    return nemu_link_peripheral_transport_set_enabled(true);
#endif
}
SYS_INIT(adapter_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
