#pragma once
#include <stdint.h>

enum nemu_link_profile_stage {
    NEMU_LINK_PROFILE_CLOCK,
    NEMU_LINK_PROFILE_INIT,
    NEMU_LINK_PROFILE_ENCRYPT,
    NEMU_LINK_PROFILE_DECRYPT,
    NEMU_LINK_PROFILE_TX,
    NEMU_LINK_PROFILE_WORK,
    NEMU_LINK_PROFILE_STOP,
    NEMU_LINK_PROFILE_COUNT,
};

#if defined(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE) && CONFIG_ZMK_NEMU_LINK_POWER_PROFILE
#include <zephyr/kernel.h>
#if defined(CONFIG_ZMK_NEMU_LINK_POWER_PROFILE_HIRES) && CONFIG_ZMK_NEMU_LINK_POWER_PROFILE_HIRES
uint32_t nemu_link_profile_start(void);
static inline uint32_t nemu_link_profile_hz(void) { return 1000000U; }
static inline uint64_t nemu_link_profile_to_us(uint64_t cycles) { return cycles; }
#else
static inline uint32_t nemu_link_profile_start(void) { return k_cycle_get_32(); }
static inline uint32_t nemu_link_profile_hz(void) { return CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC; }
static inline uint64_t nemu_link_profile_to_us(uint64_t cycles) {
    return k_cyc_to_us_floor64(cycles);
}
#endif
void nemu_link_profile_end(enum nemu_link_profile_stage stage, uint32_t start);
void nemu_link_profile_complete(void);
void nemu_link_profile_service(void);
void nemu_link_profile_attempts(uint32_t attempts);
void nemu_link_profile_log(void);
#else
static inline uint32_t nemu_link_profile_start(void) { return 0; }
static inline void nemu_link_profile_end(enum nemu_link_profile_stage stage, uint32_t start) {
    (void)stage;
    (void)start;
}
static inline void nemu_link_profile_complete(void) {}
static inline void nemu_link_profile_service(void) {}
static inline void nemu_link_profile_attempts(uint32_t attempts) { (void)attempts; }
static inline void nemu_link_profile_log(void) {}
#endif
