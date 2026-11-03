#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zmk/nemu_link/nrf_ccm.h>
#include <zmk/nemu_link/power_trace.h>
#include <zmk/nemu_link/zeroize.h>
#ifdef NEMU_LINK_NRF_CCM_HOST_TEST
#include "nrf_ccm_platform.h"
#else
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_ccm.h>
#include <mbedtls/platform_util.h>
#include "../radio/radio_hw.h"
#if defined(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#endif
#endif

LOG_MODULE_REGISTER(nemu_link_ccm, LOG_LEVEL_INF);
K_MUTEX_DEFINE(ccm_lock);
K_SEM_DEFINE(ccm_done, 0, 1);

/* Arrays avoid compiler-dependent uint64_t/bitfield layout in CNFPTR. */
struct ccm_configuration {
    uint8_t key[NEMU_LINK_AES_KEY_SIZE];
    uint8_t counter[NEMU_LINK_CCM_HW_COUNTER_SIZE];
    uint8_t direction;
    uint8_t iv[NEMU_LINK_CCM_IV_SIZE];
};
static struct ccm_configuration configuration __aligned(4);
_Static_assert(offsetof(struct ccm_configuration, iv) == 25, "CCM CNFPTR layout");
static uint8_t dma_input[NEMU_LINK_CCM_DMA_SIZE] __aligned(4);
static uint8_t dma_output[sizeof(dma_input)] __aligned(4);
static uint8_t scratch[NEMU_LINK_NRF_CCM_PLAIN_MAX + NEMU_LINK_CCM_SCRATCH_OVERHEAD] __aligned(4);
static bool irq_connected;

#if defined(CONFIG_ZMK_NEMU_LINK_CCM_RADIO) && CONFIG_ZMK_NEMU_LINK_CCM_RADIO
static volatile bool radio_busy, sync_busy;
static bool radio_decrypt, radio_crypt_pending;
static uint8_t *radio_input;
extern void nemu_link_link_ccm_ready(void);

bool nemu_link_nrf_ccm_radio_available(void) { return !radio_busy && !sync_busy; }

static void radio_cleanup(void) {
    NRF_CCM->INTENCLR = UINT32_MAX;
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_STOP);
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
    NRF_CCM->SHORTS = 0;
    NRF_CCM->INPTR = NRF_CCM->OUTPTR = NRF_CCM->CNFPTR = NRF_CCM->SCRATCHPTR = 0;
    nemu_link_zeroize(&configuration, sizeof(configuration));
    nemu_link_zeroize(dma_output, sizeof(dma_output));
    nemu_link_zeroize(scratch, sizeof(scratch));
    radio_input = NULL;
    radio_crypt_pending = false;
    radio_busy = false;
    /* RADIO runs at priority 0 to keep ACK timing during flash writes. It
     * must not enter Zephyr's semaphore/scheduler code, even on completion. */
}

void nemu_link_nrf_ccm_radio_cancel(void) {
    unsigned int locked = irq_lock();
    if (radio_busy)
        radio_cleanup();
    irq_unlock(locked);
}

int nemu_link_nrf_ccm_radio_begin(bool decrypt, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                                  const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], uint8_t *input,
                                  bool crypt_now) {
    unsigned int locked = irq_lock();
    bool bluetooth_ready = false;
#if defined(CONFIG_BT)
    bluetooth_ready = bt_is_ready();
#endif
    if (radio_busy || sync_busy || !hf_clock_held || ((uintptr_t)input & 3) || bluetooth_ready) {
        irq_unlock(locked);
        return -EBUSY;
    }
    radio_busy = true;
    radio_decrypt = decrypt;
    radio_input = input;
    NRF_CCM->INTENCLR = UINT32_MAX;
    irq_disable(CCM_AAR_IRQn);
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_STOP);
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
    NVIC_ClearPendingIRQ(CCM_AAR_IRQn);
    memcpy(configuration.key, key, NEMU_LINK_AES_KEY_SIZE);
    memset(configuration.counter, 0, sizeof(configuration.counter));
    memcpy(configuration.counter, nonce, NEMU_LINK_COUNTER_SIZE);
    configuration.counter[NEMU_LINK_COUNTER_SIZE] =
        nonce[NEMU_LINK_CCM_DIRECTION_OFFSET] & NEMU_LINK_CCM_SOURCE_MASK;
    configuration.direction =
        nonce[NEMU_LINK_CCM_DIRECTION_OFFSET] >> NEMU_LINK_CCM_DIRECTION_SHIFT;
    memcpy(configuration.iv, nonce + NEMU_LINK_CCM_COUNTER_SIZE, NEMU_LINK_CCM_IV_SIZE);
    NRF_CCM->MODE = (decrypt ? CCM_MODE_MODE_Decryption : CCM_MODE_MODE_Encryption) |
                    (CCM_MODE_LENGTH_Extended << CCM_MODE_LENGTH_Pos) |
                    ((IS_ENABLED(CONFIG_ZMK_NEMU_LINK_RADIO_BLE_1M) ? CCM_MODE_DATARATE_1Mbit
                                                                    : CCM_MODE_DATARATE_2Mbit)
                     << CCM_MODE_DATARATE_Pos);
    /* Hardware minimum is 27, not the 1/3-byte mini plaintext length. */
    size_t plain_length = decrypt ? input[1] - NEMU_LINK_CCM_MIC_SIZE : input[1];
    NRF_CCM->MAXPACKETSIZE =
        plain_length > NEMU_LINK_CCM_MIN_PACKET_SIZE ? plain_length : NEMU_LINK_CCM_MIN_PACKET_SIZE;
    NRF_CCM->CNFPTR = (uintptr_t)&configuration;
    NRF_CCM->INPTR = (uintptr_t)input;
    NRF_CCM->OUTPTR = (uintptr_t)dma_output;
    NRF_CCM->SCRATCHPTR = (uintptr_t)scratch;
    NRF_CCM->SHORTS = crypt_now ? CCM_SHORTS_ENDKSGEN_CRYPT_Msk : 0;
    NRF_CCM->EVENTS_ENDKSGEN = NRF_CCM->EVENTS_ENDCRYPT = NRF_CCM->EVENTS_ERROR = 0;
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;
    __DSB();
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_KSGEN);
    irq_unlock(locked);
    return 0;
}

void nemu_link_nrf_ccm_radio_input(uint8_t *input) {
    if (radio_busy && !((uintptr_t)input & 3)) {
        radio_input = input;
        NRF_CCM->INPTR = (uintptr_t)input;
    }
}

int nemu_link_nrf_ccm_radio_crypt(void) {
    if (!radio_busy || !radio_input ||
        radio_input[1] < NEMU_LINK_MINI_KIND_SIZE + NEMU_LINK_CCM_MIC_SIZE ||
        radio_input[1] > NEMU_LINK_CCM_CIPHER_MAX)
        return -EINVAL;
    unsigned int locked = irq_lock();
    /* Never combine a late ENDKSGEN shortcut with a manual CRYPT trigger:
     * that can start the operation twice at the event boundary. */
    if (NRF_CCM->EVENTS_ENDKSGEN) {
        nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_START);
    } else {
        radio_crypt_pending = true;
        NRF_CCM->INTENSET = CCM_INTENSET_ENDKSGEN_Msk;
        irq_enable(CCM_AAR_IRQn);
    }
    irq_unlock(locked);
    return 0;
}

int nemu_link_nrf_ccm_radio_finish(uint8_t *out, size_t capacity) {
    if (!radio_busy)
        return -ENOENT;
    if (radio_crypt_pending && NRF_CCM->EVENTS_ENDKSGEN) {
        radio_crypt_pending = false;
        NRF_CCM->INTENCLR = CCM_INTENSET_ENDKSGEN_Msk;
        nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_START);
    }
    if (!NRF_CCM->EVENTS_ERROR && !NRF_CCM->EVENTS_ENDCRYPT)
        return -EINPROGRESS;
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_CCM_DONE, radio_decrypt);
    size_t expected = radio_decrypt ? radio_input[1] - NEMU_LINK_CCM_MIC_SIZE
                                    : radio_input[1] + NEMU_LINK_CCM_MIC_SIZE;
    int rc = NRF_CCM->EVENTS_ERROR                              ? -EIO
             : radio_decrypt && !NRF_CCM->MICSTATUS             ? -EACCES
             : expected > capacity || dma_output[1] != expected ? -EIO
                                                                : 0;
    if (!rc)
        memcpy(out, dma_output + NEMU_LINK_CCM_DMA_HEADER_SIZE, expected);
    radio_cleanup();
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_CCM_CLEANED, radio_decrypt);
    return rc;
}
#endif

#define CCM_INTERRUPTS (CCM_INTENSET_ENDCRYPT_Msk | CCM_INTENSET_ERROR_Msk)

static void ccm_isr(const void *arg) {
    (void)arg;
#if defined(CONFIG_ZMK_NEMU_LINK_CCM_RADIO) && CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    if (radio_busy) {
        if (radio_crypt_pending && NRF_CCM->EVENTS_ENDKSGEN) {
            radio_crypt_pending = false;
            NRF_CCM->INTENCLR = CCM_INTENSET_ENDKSGEN_Msk;
            nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_START);
        }
        return;
    }
#endif
    if (NRF_CCM->EVENTS_ENDCRYPT || NRF_CCM->EVENTS_ERROR) {
        nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_IRQ, NRF_CCM->EVENTS_ERROR ? 1 : 0);
        NRF_CCM->INTENCLR = CCM_INTERRUPTS;
        k_sem_give(&ccm_done);
    }
}

int nemu_link_nrf_ccm_crypt(bool decrypt, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                            const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *input,
                            size_t length, uint8_t *output, size_t capacity) {
    if (!key || !nonce || !input || !output || !length ||
        (decrypt && length <= NEMU_LINK_NRF_CCM_MIC_SIZE)) {
        return -EINVAL;
    }
    size_t plain_size = decrypt ? length - NEMU_LINK_NRF_CCM_MIC_SIZE : length;
    size_t output_size = decrypt ? plain_size : length + NEMU_LINK_NRF_CCM_MIC_SIZE;
    if (plain_size > NEMU_LINK_NRF_CCM_PLAIN_MAX || capacity < output_size) {
        return -EMSGSIZE;
    }
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_ENTER, decrypt);
    k_mutex_lock(&ccm_lock, K_FOREVER);
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_LOCKED, decrypt);
#if defined(CONFIG_BT)
    /* The mode worker stops BLE before enabling the link and stops the link
     * before restarting BLE. The link mutex covers all callers and shutdown. */
    if (bt_is_ready()) {
        k_mutex_unlock(&ccm_lock);
        return -EBUSY;
    }
#endif
    bool temporary_clock = !hf_clock_held;
    int err = temporary_clock ? nemu_link_clock_acquire(5) : 0;
    if (err) {
        k_mutex_unlock(&ccm_lock);
        return err;
    }
#if defined(CONFIG_ZMK_NEMU_LINK_CCM_RADIO) && CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    /* RADIO completion cannot signal a kernel semaphore at priority 0.
     * Yield only while competing with an active reservation; idle has no
     * timer. The worker rechecks under the same exclusion as acquisition. */
    unsigned radio_waits = 0;
    for (;;) {
        unsigned int locked = irq_lock();
        if (!radio_busy) {
            sync_busy = true;
            irq_unlock(locked);
            break;
        }
        irq_unlock(locked);
        if (radio_waits++ == 5) {
            if (temporary_clock)
                nemu_link_clock_release();
            k_mutex_unlock(&ccm_lock);
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(1));
    }
#endif
    if (!irq_connected) {
        IRQ_CONNECT(CCM_AAR_IRQn, 2, ccm_isr, NULL, 0);
        irq_connected = true;
    }
    /* Disable stale events before arming a new operation, including a timeout. */
    NRF_CCM->INTENCLR = UINT32_MAX;
    irq_disable(CCM_AAR_IRQn);
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_STOP);
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
    NVIC_ClearPendingIRQ(CCM_AAR_IRQn);
    k_sem_reset(&ccm_done);
    memcpy(configuration.key, key, NEMU_LINK_AES_KEY_SIZE);
    memset(configuration.counter, 0, sizeof(configuration.counter));
    memcpy(configuration.counter, nonce, NEMU_LINK_COUNTER_SIZE);
    configuration.counter[NEMU_LINK_COUNTER_SIZE] =
        nonce[NEMU_LINK_CCM_DIRECTION_OFFSET] & NEMU_LINK_CCM_SOURCE_MASK;
    configuration.direction =
        nonce[NEMU_LINK_CCM_DIRECTION_OFFSET] >> NEMU_LINK_CCM_DIRECTION_SHIFT;
    memcpy(configuration.iv, nonce + NEMU_LINK_CCM_COUNTER_SIZE, NEMU_LINK_CCM_IV_SIZE);
    dma_input[0] = 0; /* BLE AAD header is fixed; wire header is inside plaintext. */
    dma_input[1] = length;
    dma_input[2] = 0;
    memcpy(dma_input + NEMU_LINK_CCM_DMA_HEADER_SIZE, input, length);
    memset(dma_output, 0, sizeof(dma_output));
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;
    NRF_CCM->MODE = (decrypt ? CCM_MODE_MODE_Decryption : CCM_MODE_MODE_Encryption) |
                    (CCM_MODE_LENGTH_Extended << CCM_MODE_LENGTH_Pos) |
                    (CCM_MODE_DATARATE_2Mbit << CCM_MODE_DATARATE_Pos);
    NRF_CCM->MAXPACKETSIZE = NEMU_LINK_NRF_CCM_PLAIN_MAX;
    NRF_CCM->CNFPTR = (uintptr_t)&configuration;
    NRF_CCM->INPTR = (uintptr_t)dma_input;
    NRF_CCM->OUTPTR = (uintptr_t)dma_output;
    NRF_CCM->SCRATCHPTR = (uintptr_t)scratch;
    NRF_CCM->SHORTS = CCM_SHORTS_ENDKSGEN_CRYPT_Msk;
    NRF_CCM->EVENTS_ENDKSGEN = 0;
    NRF_CCM->EVENTS_ENDCRYPT = 0;
    NRF_CCM->EVENTS_ERROR = 0;
    NRF_CCM->INTENSET = CCM_INTERRUPTS;
    irq_enable(CCM_AAR_IRQn);
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_CONFIGURED, plain_size);
    __DSB();
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_START, decrypt);
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_KSGEN);
    err = k_sem_take(&ccm_done, K_MSEC(1));
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_RESUMED, (uint16_t)-err);
    if (err) {
        err = -ETIMEDOUT;
    } else if (NRF_CCM->EVENTS_ERROR || !NRF_CCM->EVENTS_ENDCRYPT) {
        err = -EIO;
    } else if (decrypt && NRF_CCM->MICSTATUS != CCM_MICSTATUS_MICSTATUS_CheckPassed) {
        err = -EACCES;
    } else if (dma_output[1] != output_size) {
        err = -EIO;
    }
    /* STOP and disable DMA/IRQ before wiping buffers or giving BLE the block. */
    NRF_CCM->INTENCLR = UINT32_MAX;
    irq_disable(CCM_AAR_IRQn);
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_STOP);
    NRF_CCM->SHORTS = 0;
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
    NVIC_ClearPendingIRQ(CCM_AAR_IRQn);
    NRF_CCM->CNFPTR = 0;
    NRF_CCM->INPTR = 0;
    NRF_CCM->OUTPTR = 0;
    NRF_CCM->SCRATCHPTR = 0;
    __DSB();
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_STOPPED, (uint16_t)-err);
    if (temporary_clock) {
        int clock_err = nemu_link_clock_release();
        if (!err) {
            err = clock_err;
        }
    }
    if (!err) {
        memcpy(output, dma_output + NEMU_LINK_CCM_DMA_HEADER_SIZE, output_size);
    }
    nemu_link_zeroize(&configuration, sizeof(configuration));
    nemu_link_zeroize(dma_input, sizeof(dma_input));
    nemu_link_zeroize(dma_output, sizeof(dma_output));
    nemu_link_zeroize(scratch, sizeof(scratch));
#if defined(CONFIG_ZMK_NEMU_LINK_CCM_RADIO) && CONFIG_ZMK_NEMU_LINK_CCM_RADIO
    sync_busy = false;
    if (!temporary_clock)
        nemu_link_link_ccm_ready();
#endif
    k_mutex_unlock(&ccm_lock);
    nemu_link_trace_mark(NEMU_LINK_TRACE_CCM_CLEANED, decrypt);
    return err;
}

int nemu_link_nrf_ccm_self_test(void) {
    static bool passed;
    if (passed) {
        return 0;
    }
    const uint8_t key[NEMU_LINK_AES_KEY_SIZE] = {0, 1, 2,  3,  4,  5,  6,  7,
                                                 8, 9, 10, 11, 12, 13, 14, 15};
    const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE] = {1, 0x23, 0x45, 0x67, 0x81, 0, 1,
                                                     2, 3,    4,    5,    6,    7};
    /* Independent cryptography AESCCM(tag_length=4), AAD={0}, plaintext=0..n-1. */
    static const uint8_t vector1[5] = {0x66, 0x9e, 0xe5, 0x71, 0x6d};
    static const uint8_t vector32[36] = {0x66, 0xb0, 0xa5, 0xc9, 0x7b, 0xed, 0x9e, 0xb7, 0x3e,
                                         0x2a, 0xdf, 0xc1, 0x88, 0xc3, 0xae, 0xae, 0xb7, 0x92,
                                         0x5d, 0x4d, 0x05, 0x1e, 0x47, 0xf1, 0xd7, 0x54, 0x6b,
                                         0x3f, 0x1c, 0x1f, 0x75, 0x27, 0x36, 0xd0, 0xae, 0xc3};
    static const uint8_t vector52[NEMU_LINK_CCM_CIPHER_MAX] = {
        0x66, 0xb0, 0xa5, 0xc9, 0x7b, 0xed, 0x9e, 0xb7, 0x3e, 0x2a, 0xdf, 0xc1, 0x88, 0xc3,
        0xae, 0xae, 0xb7, 0x92, 0x5d, 0x4d, 0x05, 0x1e, 0x47, 0xf1, 0xd7, 0x54, 0x6b, 0x3f,
        0x1c, 0x1f, 0x75, 0x27, 0x65, 0x53, 0x5e, 0x47, 0xf8, 0x11, 0xfa, 0x1e, 0x59, 0x39,
        0x24, 0x76, 0xd9, 0x9a, 0x8d, 0x5f, 0x12, 0x13, 0xb3, 0xc2, 0x3b, 0xcb, 0xa1, 0xea};
    const struct {
        const uint8_t *expected;
        size_t size;
    } vectors[] = {{vector1, 1}, {vector32, 32}, {vector52, 52}};
    uint8_t plain[NEMU_LINK_CCM_PLAIN_MAX], encrypted[NEMU_LINK_CCM_CIPHER_MAX],
        decoded[NEMU_LINK_CCM_PLAIN_MAX];
    for (size_t i = 0; i < sizeof(plain); i++) {
        plain[i] = i;
    }
    int err = 0;
    for (size_t i = 0; i < 3; i++) {
        size_t size = vectors[i].size;
        err = nemu_link_nrf_ccm_crypt(false, key, nonce, plain, size, encrypted, sizeof(encrypted));
        if (err || memcmp(encrypted, vectors[i].expected, size + NEMU_LINK_CCM_MIC_SIZE)) {
            err = err ? err : -EIO;
            break;
        }
        err = nemu_link_nrf_ccm_crypt(true, key, nonce, encrypted, size + NEMU_LINK_CCM_MIC_SIZE,
                                      decoded, sizeof(decoded));
        if (err || memcmp(decoded, plain, size)) {
            err = err ? err : -EIO;
            break;
        }
        encrypted[size + NEMU_LINK_CCM_MIC_SIZE - 1] ^= 1;
        if (nemu_link_nrf_ccm_crypt(true, key, nonce, encrypted, size + NEMU_LINK_CCM_MIC_SIZE,
                                    decoded, sizeof(decoded)) != -EACCES) {
            err = -EIO;
            break;
        }
    }
    if (err) {
        LOG_ERR("NRF_CCM self-test failed: %d", err);
    } else {
        passed = true;
        LOG_INF("NRF_CCM self-test passed: wire=%u MIC=%u lengths=1/32/%u", NEMU_LINK_WIRE_VERSION,
                NEMU_LINK_CCM_MIC_SIZE, NEMU_LINK_CCM_PLAIN_MAX);
    }
    return err;
}
