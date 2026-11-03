#include <zmk/nemu_link/idle.h>

void nemu_link_idle_init(struct dt_idle *idle, uint32_t grace_ms, uint32_t wake_margin_ms) {
    *idle = (struct dt_idle){
        .phase = NEMU_LINK_IDLE_AWAKE, .grace_ms = grace_ms, .wake_margin_ms = wake_margin_ms};
}

bool nemu_link_idle_should_sleep(struct dt_idle *idle, uint64_t now, uint64_t deadline, bool busy) {
    if (idle->phase == NEMU_LINK_IDLE_OFF || idle->phase == NEMU_LINK_IDLE_WAKING) {
        return false;
    }
    if (busy || now >= deadline || deadline - now <= idle->wake_margin_ms) {
        idle->phase = NEMU_LINK_IDLE_AWAKE;
        return false;
    }
    if (idle->phase == NEMU_LINK_IDLE_AWAKE) {
        idle->phase = NEMU_LINK_IDLE_WAIT;
        idle->quiet_since = now;
    }
    return now >= idle->quiet_since && now - idle->quiet_since >= idle->grace_ms;
}

bool nemu_link_idle_should_wake(const struct dt_idle *idle, uint64_t now, uint64_t deadline,
                                bool input) {
    return idle->phase == NEMU_LINK_IDLE_OFF &&
           (input || now >= deadline || deadline - now <= idle->wake_margin_ms);
}

uint32_t nemu_link_idle_next_service(const struct dt_idle *idle, uint64_t now, uint64_t deadline,
                                     uint32_t maximum_ms) {
    uint64_t target = deadline;
    if (idle->phase == NEMU_LINK_IDLE_WAIT && idle->quiet_since + idle->grace_ms < target) {
        target = idle->quiet_since + idle->grace_ms;
    } else if (idle->phase == NEMU_LINK_IDLE_OFF) {
        target = deadline > idle->wake_margin_ms ? deadline - idle->wake_margin_ms : 0;
    }
    if (target <= now) {
        return 1;
    }
    return target - now < maximum_ms ? (uint32_t)(target - now) : maximum_ms;
}
