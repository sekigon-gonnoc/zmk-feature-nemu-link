#include <string.h>
#include <zephyr/logging/log.h>
#include <zmk/nemu_link/power_profile.h>
#include <zmk/nemu_link/power_trace.h>

#if defined(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE_HIRES) && CONFIG_ZMK_NEMU_LINK_POWER_PROFILE_HIRES
#include <hal/nrf_timer.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>

BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(timer4), okay), "TIMER4 is reserved by devicetree");
BUILD_ASSERT(!IS_ENABLED(CONFIG_NRFX_TIMER4), "TIMER4 is reserved by another driver");
BUILD_ASSERT(!IS_ENABLED(CONFIG_ESB_SYS_TIMER4), "TIMER4 is reserved by ESB");

static int profile_timer_init(void) {
    nrf_timer_task_trigger(NRF_TIMER4, NRF_TIMER_TASK_STOP);
    nrf_timer_shorts_set(NRF_TIMER4, 0);
    nrf_timer_int_disable(NRF_TIMER4, UINT32_MAX);
    nrf_timer_mode_set(NRF_TIMER4, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER4, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(NRF_TIMER4, 4); /* 16 MHz / 16 = 1 MHz. */
    nrf_timer_task_trigger(NRF_TIMER4, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(NRF_TIMER4, NRF_TIMER_TASK_START);
    return 0;
}
SYS_INIT(profile_timer_init, APPLICATION, 80);

uint32_t nemu_link_profile_start(void) {
    /* The same CC register is shared by thread/IRQ instrumentation. */
    unsigned int key = irq_lock();
    nrf_timer_task_trigger(NRF_TIMER4, NRF_TIMER_TASK_CAPTURE0);
    uint32_t value = nrf_timer_cc_get(NRF_TIMER4, NRF_TIMER_CC_CHANNEL0);
    irq_unlock(key);
    return value;
}
#endif

LOG_MODULE_REGISTER(nemu_link_power, LOG_LEVEL_INF);

struct profile_stats {
    uint32_t count, last, maximum;
    uint64_t total;
};
static struct profile_stats stats[NEMU_LINK_PROFILE_COUNT];
static const char *const names[] = {"clock", "init", "encrypt", "decrypt", "tx", "work", "stop"};
static bool completion_pending;
static uint32_t completion_start;
static uint32_t transactions, attempts_total, attempts_max;

void nemu_link_profile_end(enum nemu_link_profile_stage stage, uint32_t start) {
    uint32_t elapsed = nemu_link_profile_start() - start;
    unsigned int key = irq_lock();
    struct profile_stats *s = &stats[stage];
    s->count++;
    s->last = elapsed;
    s->maximum = MAX(s->maximum, elapsed);
    s->total += elapsed;
    irq_unlock(key);
}

/* Preserve the first outstanding completion, including a timestamp of zero. */
void nemu_link_profile_complete(void) {
    unsigned int key = irq_lock();
    if (!completion_pending) {
        completion_start = nemu_link_profile_start();
        completion_pending = true;
    }
    irq_unlock(key);
}

void nemu_link_profile_service(void) {
    unsigned int key = irq_lock();
    if (completion_pending) {
        nemu_link_profile_end(NEMU_LINK_PROFILE_WORK, completion_start);
        completion_pending = false;
    }
    irq_unlock(key);
}

void nemu_link_profile_attempts(uint32_t attempts) {
    unsigned int key = irq_lock();
    transactions++;
    attempts_total += attempts;
    attempts_max = MAX(attempts_max, attempts);
    irq_unlock(key);
}

void nemu_link_profile_log(void) {
    struct profile_stats snapshot[NEMU_LINK_PROFILE_COUNT];
    unsigned int key = irq_lock();
    memcpy(snapshot, stats, sizeof(snapshot));
    memset(stats, 0, sizeof(stats));
    uint32_t tx = transactions, attempts = attempts_total, maximum = attempts_max;
    transactions = attempts_total = attempts_max = 0;
    irq_unlock(key);

    /* Log after dropping the IRQ lock. No logging inside timed operations. */
    LOG_INF("power profile clock_hz=%u transactions=%u attempts=%u attempts_max=%u",
            nemu_link_profile_hz(), tx, attempts, maximum);
    for (unsigned int i = 0; i < NEMU_LINK_PROFILE_COUNT; i++) {
        const struct profile_stats *s = &snapshot[i];
        if (s->count) {
            LOG_INF("power %s n=%u avg_us=%u max_us=%u last_us=%u", names[i], s->count,
                    (uint32_t)(nemu_link_profile_to_us(s->total) / s->count),
                    (uint32_t)nemu_link_profile_to_us(s->maximum),
                    (uint32_t)nemu_link_profile_to_us(s->last));
        }
    }
    nemu_link_trace_log();
}
