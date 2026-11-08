#include <errno.h>
#include <string.h>

#include <zmk/nemu_link/idle.h>

#define FAST_INPUT_SOURCE_COUNT                                                                    \
    (sizeof(((struct nemu_link_fast_input_state *)0)->sources) /                                   \
     sizeof(struct nemu_link_fast_input_source))

void nemu_link_fast_input_init(struct nemu_link_fast_input_state *state, uint32_t timeout_ms) {
    memset(state, 0, sizeof(*state));
    state->timeout_ms = timeout_ms;
}

int nemu_link_fast_input_set(struct nemu_link_fast_input_state *state, const void *device,
                             bool enabled, uint64_t now) {
    if (!state || !device) {
        return -EINVAL;
    }

    struct nemu_link_fast_input_source *free_source = NULL;
    for (size_t i = 0; i < FAST_INPUT_SOURCE_COUNT; i++) {
        struct nemu_link_fast_input_source *source = &state->sources[i];
        if (source->device == device) {
            if (enabled) {
                source->last_activity = now;
            } else {
                *source = (struct nemu_link_fast_input_source){0};
            }
            return 0;
        }
        if (!source->device && !free_source) {
            free_source = source;
        }
    }

    if (!enabled) {
        return 0;
    }
    if (!free_source) {
        return -ENOSPC;
    }
    *free_source = (struct nemu_link_fast_input_source){.device = device, .last_activity = now};
    return 0;
}

int nemu_link_fast_input_activity(struct nemu_link_fast_input_state *state, const void *device,
                                  uint64_t now) {
    if (!state || !device) {
        return -EINVAL;
    }
    for (size_t i = 0; i < FAST_INPUT_SOURCE_COUNT; i++) {
        struct nemu_link_fast_input_source *source = &state->sources[i];
        if (source->device == device) {
            source->last_activity = now;
            return 0;
        }
    }
    return -ENOENT;
}

void nemu_link_fast_input_refresh(struct nemu_link_fast_input_state *state, uint64_t now) {
    for (size_t i = 0; i < FAST_INPUT_SOURCE_COUNT; i++) {
        if (state->sources[i].device) {
            state->sources[i].last_activity = now;
        }
    }
}

bool nemu_link_fast_input_active(const struct nemu_link_fast_input_state *state, uint64_t now) {
    if (!state || !state->timeout_ms) {
        return false;
    }
    for (size_t i = 0; i < FAST_INPUT_SOURCE_COUNT; i++) {
        const struct nemu_link_fast_input_source *source = &state->sources[i];
        if (source->device && now >= source->last_activity &&
            now - source->last_activity < state->timeout_ms) {
            return true;
        }
    }
    return false;
}

bool nemu_link_fast_input_active_for_route(const struct nemu_link_fast_input_state *state,
                                           uint64_t now, bool esb_service_active,
                                           bool endpoint_non_usb) {
    return esb_service_active && endpoint_non_usb && nemu_link_fast_input_active(state, now);
}

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
