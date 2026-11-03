#include <errno.h>
#include <zmk/nemu_link/control.h>
#include "link_internal.h"

int nemu_link_split_native_send(app_esb_data_t *packet) {
    ARG_UNUSED(packet);
    return -ENOTSUP;
}

int nemu_link_split_receive(const struct rx_frame *rx) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_FIXED_ESB)
    return nemu_link_split_receive_standalone(rx);
#else
    ARG_UNUSED(rx);
    return -ENOTSUP;
#endif
}
