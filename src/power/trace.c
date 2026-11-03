#include <string.h>
#include <zephyr/logging/log.h>
#include <zmk/nemu_link/power_profile.h>
#include <zmk/nemu_link/power_trace.h>

LOG_MODULE_REGISTER(nemu_link_trace, LOG_LEVEL_INF);

/* Bounded RAM only: no payloads, keys, new IRQs or work are recorded here. */
#define TRACE_EVENTS 64
#define TRACE_RECORDS 3
struct trace_mark {
    uint32_t elapsed;
    uint16_t value;
    uint8_t event;
};
struct trace_record {
    uint32_t id, sequence, start;
    uint32_t timer_begin, rtc_begin, timer_span, rtc_span;
    uint16_t lost;
    uint8_t count;
    struct trace_mark marks[TRACE_EVENTS];
};
static struct trace_record current, completed[TRACE_RECORDS];
static uint32_t next_id, overwritten;
static uint8_t first, used;
static bool active;

static const char *const names[] = {
    "begin",
    "probe",
    "message_ready",
    "clock_begin",
    "clock_ready",
    "init_begin",
    "init_end",
    "encode_begin",
    "encode_end",
    "pack_begin",
    "pack_end",
    "encrypt_begin",
    "encrypt_end",
    "ccm_enter",
    "ccm_locked",
    "ccm_configured",
    "ccm_start",
    "ccm_irq",
    "ccm_resumed",
    "ccm_stopped",
    "ccm_cleaned",
    "queue_begin",
    "queue_end",
    "tx_start",
    "tx_done",
    "complete",
    "rx_irq",
    "rx_queued",
    "work_entry",
    "work_locked",
    "rx_begin",
    "decrypt_begin",
    "decrypt_end",
    "unpack_begin",
    "unpack_end",
    "link_decode_begin",
    "link_decode_end",
    "ack_apply_begin",
    "ack_apply_end",
    "rx_end",
    "maint_begin",
    "maint_end",
    "stop_begin",
    "radio_disabled",
    "clock_released",
    "stop_end",
    "diag_begin",
    "diag_end",
    "work_end",
    "finish",
    "radio_tx_ccm_begin",
    "radio_tx_ready",
    "radio_tx_ccm_end",
    "radio_rx_ccm_arm",
    "radio_rx_ccm_begin",
    "radio_rx_ccm_end",
    "radio_rx_ccm_cached",
    "radio_rx_end",
    "radio_ccm_done",
    "radio_ccm_cleaned",
};
_Static_assert(sizeof(names) / sizeof(names[0]) == NEMU_LINK_TRACE_COUNT, "trace names");

/* Called only under the IRQ lock. Keep the final slot for the terminal mark. */
static void mark_locked(enum nemu_link_trace_event event, uint16_t value) {
    if (!active || (unsigned int)event >= NEMU_LINK_TRACE_COUNT) {
        return;
    }
    if (current.count >= TRACE_EVENTS) {
        return;
    }
    if (current.count >= TRACE_EVENTS - 1 && event != NEMU_LINK_TRACE_FINISH) {
        if (current.lost != UINT16_MAX) {
            current.lost++;
        }
        return;
    }
    current.marks[current.count++] =
        (struct trace_mark){.elapsed = nemu_link_profile_start() - current.start,
                            .event = (uint8_t)event,
                            .value = value};
}

void nemu_link_trace_begin(uint32_t sequence, uint32_t start) {
    unsigned int key = irq_lock();
    if (!active) {
        current.id = ++next_id;
        current.sequence = sequence;
        current.start = start;
        current.timer_begin = nemu_link_profile_start();
        current.rtc_begin = k_cycle_get_32();
        current.lost = current.count = 0;
        active = true;
        current.marks[current.count++] = (struct trace_mark){.event = NEMU_LINK_TRACE_BEGIN};
    }
    irq_unlock(key);
}

void nemu_link_trace_mark(enum nemu_link_trace_event event, uint16_t value) {
    unsigned int key = irq_lock();
    mark_locked(event, value);
    irq_unlock(key);
}

void nemu_link_trace_finish(uint16_t reason) {
    unsigned int key = irq_lock();
    if (active) {
        mark_locked(NEMU_LINK_TRACE_FINISH, reason);
        current.timer_span = nemu_link_profile_start() - current.timer_begin;
        current.rtc_span = k_cycle_get_32() - current.rtc_begin;
        if (used == TRACE_RECORDS) {
            first = (uint8_t)((first + 1U) % TRACE_RECORDS);
            used--;
            overwritten++;
        }
        completed[(first + used) % TRACE_RECORDS] = current;
        used++;
        active = false;
    }
    irq_unlock(key);
}

void nemu_link_trace_log(void) {
    /* Drain at most one interval's capacity, including calls during a trace.
     * Copy one record under the lock, then print with interrupts enabled. */
    for (unsigned int n = 0; n < TRACE_RECORDS; n++) {
        struct trace_record snapshot;
        unsigned int key = irq_lock();
        if (!used) {
            irq_unlock(key);
            return;
        }
        snapshot = completed[first];
        first = (uint8_t)((first + 1U) % TRACE_RECORDS);
        used--;
        uint32_t dropped = overwritten;
        overwritten = 0;
        irq_unlock(key);
        LOG_INF("trace id=%u link_seq=%u timer_hz=%u n=%u lost=%u overwritten=%u", snapshot.id,
                snapshot.sequence, nemu_link_profile_hz(), snapshot.count, snapshot.lost, dropped);
        LOG_INF("trace id=%u timer_span_us=%u rtc_span_us=%u", snapshot.id,
                (uint32_t)nemu_link_profile_to_us(snapshot.timer_span),
                (uint32_t)k_cyc_to_us_floor64(snapshot.rtc_span));
        uint32_t previous = 0;
        for (unsigned int i = 0; i < snapshot.count; i++) {
            const struct trace_mark *m = &snapshot.marks[i];
            LOG_INF("trace id=%u event=%s t_us=%u dt_us=%u value=%u", snapshot.id, names[m->event],
                    (uint32_t)nemu_link_profile_to_us(m->elapsed),
                    (uint32_t)nemu_link_profile_to_us(m->elapsed - previous), m->value);
            previous = m->elapsed;
        }
    }
}
