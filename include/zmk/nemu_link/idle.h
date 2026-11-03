#ifndef ZMK_NEMU_LINK_IDLE_H
#define ZMK_NEMU_LINK_IDLE_H
#include <stdbool.h>
#include <stdint.h>

enum nemu_link_idle_phase {
    NEMU_LINK_IDLE_AWAKE,
    NEMU_LINK_IDLE_WAIT,
    NEMU_LINK_IDLE_OFF,
    NEMU_LINK_IDLE_WAKING
};
struct dt_idle {
    enum nemu_link_idle_phase phase;
    uint64_t quiet_since;
    uint32_t grace_ms;
    uint32_t wake_margin_ms;
};
void nemu_link_idle_init(struct dt_idle *idle, uint32_t grace_ms, uint32_t wake_margin_ms);
/* Serialized policy only: OFF is committed by the caller after radio shutdown
 * and HFCLK release both succeed. Busy includes unprocessed RX, queued TX,
 * hardware retry/ACK and pending mode control. An application ACK waiting
 * for the next ordinary PTX transaction does not require an awake radio. */
bool nemu_link_idle_should_sleep(struct dt_idle *idle, uint64_t now, uint64_t deadline, bool busy);
bool nemu_link_idle_should_wake(const struct dt_idle *idle, uint64_t now, uint64_t deadline,
                                bool input);
uint32_t nemu_link_idle_next_service(const struct dt_idle *idle, uint64_t now, uint64_t deadline,
                                     uint32_t maximum_ms);
/* Firmware API. Call under the link link mutex; sleep also requires IRQ lock. */
int nemu_link_radio_sleep(void);
int nemu_link_radio_wake(void);
#endif
