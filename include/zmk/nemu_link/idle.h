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
/* Small, bounded registry of high-rate input sources. */
#define NEMU_LINK_FAST_INPUT_MAX_SENSORS 4
struct nemu_link_fast_input_source {
    const void *device;
    uint64_t last_activity;
};
struct nemu_link_fast_input_state {
    struct nemu_link_fast_input_source sources[NEMU_LINK_FAST_INPUT_MAX_SENSORS];
    uint32_t timeout_ms;
};
void nemu_link_fast_input_init(struct nemu_link_fast_input_state *state, uint32_t timeout_ms);
int nemu_link_fast_input_set(struct nemu_link_fast_input_state *state, const void *device,
                             bool enabled, uint64_t now);
int nemu_link_fast_input_activity(struct nemu_link_fast_input_state *state, const void *device,
                                  uint64_t now);
void nemu_link_fast_input_refresh(struct nemu_link_fast_input_state *state, uint64_t now);
bool nemu_link_fast_input_active(const struct nemu_link_fast_input_state *state, uint64_t now);
bool nemu_link_fast_input_active_for_route(const struct nemu_link_fast_input_state *state,
                                           uint64_t now, bool esb_service_active,
                                           bool endpoint_non_usb);
/* Internal ESB service hooks. */
bool nemu_link_fast_input_any_active(uint64_t now);
void nemu_link_fast_input_radio_started(uint64_t now);
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
