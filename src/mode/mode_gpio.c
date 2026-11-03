#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zmk/nemu_link/internal.h>
#define MODE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(sekigon_nemu_link)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(MODE_NODE, mode_switch_gpios);
static struct gpio_callback callback;
static bool pressed;
static void sample(struct k_work *work) {
    ARG_UNUSED(work);
    int value = gpio_pin_get_dt(&button);
    if (value < 0) {
        return;
    }
    if (value && !pressed) {
        nemu_link_mode_command(2);
    }
    pressed = value;
}
static K_WORK_DELAYABLE_DEFINE(debounce, sample);
static void edge(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_work_reschedule(&debounce, K_MSEC(20));
}
static int init(void) {
    if (!gpio_is_ready_dt(&button)) {
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err) {
        return err;
    }
    gpio_init_callback(&callback, edge, BIT(button.pin));
    err = gpio_add_callback(button.port, &callback);
    if (err) {
        return err;
    }
    pressed = gpio_pin_get_dt(&button) > 0;
    return gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
}
SYS_INIT(init, APPLICATION, 99);
