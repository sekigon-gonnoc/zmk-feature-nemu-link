#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include "radio.h"

int nemu_link_link_enable(bool enabled);
void nemu_link_radio_event(app_esb_event_t *event);

static bool active;

bool nemu_link_peripheral_transport_active(void) { return active; }

static int adapter_init(void) {
    int err = nemu_link_radio_init(APP_ESB_MODE_PTX, nemu_link_radio_event);
    if (err) {
        return err;
    }

    err = nemu_link_link_enable(true);
    if (!err) {
        active = true;
    }
    return err;
}
SYS_INIT(adapter_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
