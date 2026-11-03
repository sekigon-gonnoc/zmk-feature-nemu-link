#include <zephyr/sys/atomic.h>
#include <zmk/nemu_link/battery_status.h>

#define KNOWN 0x100
#define CONNECTED 0x200
static atomic_t levels[NEMU_LINK_BATTERY_SOURCE_COUNT];

void nemu_link_battery_update(uint8_t source, uint8_t level, bool connected) {
    if (source >= NEMU_LINK_BATTERY_SOURCE_COUNT || level > 100) {
        return;
    }
    atomic_set(&levels[source], level | KNOWN | (connected ? CONNECTED : 0));
}

void nemu_link_battery_disconnect(uint8_t source) {
    if (source < NEMU_LINK_BATTERY_SOURCE_COUNT) {
        atomic_and(&levels[source], ~CONNECTED);
    }
}

void nemu_link_battery_status_get(uint8_t out[NEMU_LINK_BATTERY_STATUS_SIZE]) {
    out[0] = NEMU_LINK_BATTERY_STATUS_MAGIC;
    out[1] = NEMU_LINK_BATTERY_STATUS_VERSION;
    out[2] = CONFIG_ZMK_NEMU_LINK_PERIPHERAL_COUNT;
    out[3] = out[4] = 0;
    for (uint8_t source = 0; source < NEMU_LINK_BATTERY_SOURCE_COUNT; source++) {
        atomic_val_t value = atomic_get(&levels[source]);
        if (source < CONFIG_ZMK_NEMU_LINK_PERIPHERAL_COUNT && (value & KNOWN)) {
            out[3] |= (uint8_t)(1U << source);
            if (value & CONNECTED) {
                out[4] |= (uint8_t)(1U << source);
            }
            out[5 + source] = (uint8_t)value;
        } else {
            out[5 + source] = 255;
        }
    }
}
