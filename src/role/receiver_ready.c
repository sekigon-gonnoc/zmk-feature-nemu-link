#include <zephyr/init.h>
#include <zmk/nemu_link/control.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RECEIVER_WAIT_USB)
#include <zephyr/kernel.h>
#include <zmk/usb.h>
static void ready_work(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(receiver_ready_work, ready_work);
static void ready_work(struct k_work *work) {
    ARG_UNUSED(work);
    zmk_nemu_link_set_receiver_ready(zmk_usb_is_hid_ready());
    k_work_reschedule(&receiver_ready_work, K_MSEC(100));
}
#endif
static int receiver_init(void) {
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RECEIVER_WAIT_USB)
    k_work_reschedule(&receiver_ready_work, K_NO_WAIT);
#elif IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB) && IS_ENABLED(CONFIG_ZMK_USB)
    /* zmk_usb_init() enables the composite HID/CDC stack at priority 96. */
    zmk_nemu_link_set_receiver_ready(true);
#elif IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB)
    zmk_nemu_link_set_receiver_ready(false);
#else
    zmk_nemu_link_set_receiver_ready(true);
#endif
    return 0;
}
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_PERIPHERAL_WAIT_USB) && IS_ENABLED(CONFIG_ZMK_USB)
SYS_INIT(receiver_init, APPLICATION, 97);
#else
SYS_INIT(receiver_init, APPLICATION, 80);
#endif
