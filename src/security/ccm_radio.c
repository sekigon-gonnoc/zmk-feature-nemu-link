#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#ifdef NEMU_LINK_CCM_RADIO_HOST_TEST
#include "nrf_ccm_platform.h"
#include "ccm_radio_platform.h"
#else
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_radio.h>
#endif
#include <mbedtls/platform_util.h>
#include <zmk/nemu_link/ccm_radio.h>
#include <zmk/nemu_link/mini.h>
#include <zmk/nemu_link/nrf_ccm.h>
#include <zmk/nemu_link/power_trace.h>
#include <zmk/nemu_link/zeroize.h>

#define TX_STAGING_SPARES 2
#define JOBS (CONFIG_ZMK_NEMU_LINK_PROTO_MSGQ_ITEMS + TX_STAGING_SPARES)
#define CCM_FINISH_POLLS 200
#define RX_RESULTS 8

struct tx_job {
    bool used, done;
    uint8_t source, key[NEMU_LINK_AES_KEY_SIZE], nonce[NEMU_LINK_CCM_NONCE_SIZE],
        plain[NEMU_LINK_CCM_PLAIN_MAX], wire[NEMU_LINK_FRAME_MAX_SIZE], size, header, plain_size;
};
static struct tx_job jobs[JOBS];
static struct tx_job *active_tx;
static uint8_t *tx_wire, *tx_fifo_wire;
static uint8_t tx_input[NEMU_LINK_CCM_DMA_SIZE] __aligned(4);
static struct {
    bool valid, mini;
    uint8_t source, key[NEMU_LINK_AES_KEY_SIZE], iv[NEMU_LINK_CCM_IV_SIZE],
        full_key[NEMU_LINK_AES_KEY_SIZE], full_iv[NEMU_LINK_CCM_IV_SIZE];
    uint32_t previous, reference;
} snapshot, receive_snapshot;

static struct {
    bool armed, started, mutated;
    uint8_t *pdu, *prefix, source, saved[NEMU_LINK_CCM_DMA_HEADER_SIZE],
        nonce[NEMU_LINK_CCM_NONCE_SIZE], key[NEMU_LINK_AES_KEY_SIZE], size;
    uint32_t predicted;
} rx;
struct rx_result {
    bool used;
    int error;
    uint8_t source, key[NEMU_LINK_AES_KEY_SIZE], nonce[NEMU_LINK_CCM_NONCE_SIZE],
        wire[NEMU_LINK_FRAME_MAX_SIZE], plain[NEMU_LINK_CCM_PLAIN_MAX], size;
};
static struct rx_result results[RX_RESULTS];
static unsigned result_next;

static bool secured_wire(const uint8_t *wire, size_t size) {
    return size == NEMU_LINK_MINI_HEARTBEAT_SIZE ||
           (size >= NEMU_LINK_COMPACT_MIN_SIZE && size <= NEMU_LINK_FRAME_MAX_SIZE &&
            wire[0] == NEMU_LINK_SECURE_WIRE_VERSION && wire[1] >= NEMU_LINK_WIRE_FIRST_KIND &&
            wire[1] <= NEMU_LINK_WIRE_AUDIO_CONTROL);
}

static bool job_matches(const struct tx_job *job, uint8_t source, const uint8_t *wire,
                        size_t size) {
    return job->used && job->source == source && job->size == size &&
           !memcmp(job->wire, wire, job->header);
}

int nemu_link_ccm_radio_queue(uint8_t source, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                              const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *plain,
                              size_t size, uint8_t out[NEMU_LINK_FRAME_MAX_SIZE], size_t header) {
    if ((header != NEMU_LINK_MINI_COUNTER_SIZE && header != NEMU_LINK_COMPACT_HEADER_SIZE) ||
        !size || size > NEMU_LINK_CCM_PLAIN_MAX ||
        size + header + NEMU_LINK_CCM_MIC_SIZE > NEMU_LINK_FRAME_MAX_SIZE)
        return -EINVAL;
    unsigned int locked = irq_lock();
    for (unsigned i = 0; i < JOBS; i++) {
        if (jobs[i].used)
            continue;
        jobs[i].used = true;
        jobs[i].source = source;
        jobs[i].header = header;
        jobs[i].plain_size = size;
        jobs[i].size = size + header + NEMU_LINK_CCM_MIC_SIZE;
        memcpy(jobs[i].key, key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(jobs[i].nonce, nonce, NEMU_LINK_CCM_NONCE_SIZE);
        memcpy(jobs[i].plain, plain, size);
        memset(out + header, 0, size + NEMU_LINK_CCM_MIC_SIZE);
        memcpy(jobs[i].wire, out, jobs[i].size);
        irq_unlock(locked);
        return 0;
    }
    irq_unlock(locked);
    return -ENOMEM;
}

void nemu_link_ccm_radio_context(const struct nemu_link_secure_peer *peer, uint8_t source,
                                 uint32_t ack_reference) {
    unsigned int locked = irq_lock();
    snapshot.valid = nemu_link_secure_ready(peer);
    snapshot.mini = snapshot.valid && peer->mini.valid;
    snapshot.source = source;
    snapshot.previous = peer->rx_sequence;
    snapshot.reference = ack_reference;
    if (snapshot.valid) {
        memcpy(snapshot.full_key, peer->ccm_key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(snapshot.full_iv, peer->nonce_prefix, NEMU_LINK_CCM_IV_SIZE);
        memcpy(snapshot.key, peer->mini.key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(snapshot.iv, peer->mini.iv, NEMU_LINK_CCM_IV_SIZE);
    } else {
        nemu_link_zeroize(snapshot.key, sizeof(snapshot.key));
        nemu_link_zeroize(snapshot.iv, sizeof(snapshot.iv));
        nemu_link_zeroize(snapshot.full_key, sizeof(snapshot.full_key));
        nemu_link_zeroize(snapshot.full_iv, sizeof(snapshot.full_iv));
    }
    irq_unlock(locked);
}

void nemu_link_ccm_radio_retire(uint8_t source, const uint8_t *wire, size_t size) {
    if (!secured_wire(wire, size))
        return;
    unsigned int locked = irq_lock();
    for (unsigned i = 0; i < JOBS; i++) {
        if (job_matches(&jobs[i], source, wire, size)) {
            if (active_tx == &jobs[i])
                active_tx = NULL;
            nemu_link_zeroize(&jobs[i], sizeof(jobs[i]));
        }
    }
    irq_unlock(locked);
}

void nemu_link_ccm_radio_stop(void) {
    nemu_link_nrf_ccm_radio_cancel();
    /* Retired jobs are already wiped. Clear only queued/in-flight owners;
     * idle must not re-wipe the full 18-slot array on every heartbeat. */
    for (unsigned i = 0; i < JOBS; i++) {
        if (jobs[i].used)
            nemu_link_zeroize(&jobs[i], sizeof(jobs[i]));
    }
    nemu_link_zeroize(&rx, sizeof(rx));
    nemu_link_zeroize(tx_input, sizeof(tx_input));
    active_tx = NULL;
    tx_wire = tx_fifo_wire = NULL;
    /* Deferred RX results must survive a normal radio idle boundary. */
}

void nemu_link_ccm_radio_reset(void) {
    nemu_link_ccm_radio_stop();
    nemu_link_zeroize(&snapshot, sizeof(snapshot));
    nemu_link_zeroize(&receive_snapshot, sizeof(receive_snapshot));
    nemu_link_zeroize(results, sizeof(results));
}

bool nemu_link_ccm_radio_can_tx(const uint8_t *wire, size_t size, uint8_t source) {
    if (!secured_wire(wire, size))
        return true;
    for (unsigned i = 0; i < JOBS; i++) {
        if (job_matches(&jobs[i], source, wire, size)) {
            return jobs[i].done || nemu_link_nrf_ccm_radio_available();
        }
    }
    return true; /* The checked hook reports a missing staging job as failure. */
}

int nemu_link_ccm_radio_tx_prepare(uint8_t *wire, uint8_t *fifo_wire, size_t size, uint8_t source) {
    if (!secured_wire(wire, size))
        return 0;
    struct tx_job *job = NULL;
    for (unsigned i = 0; i < JOBS; i++) {
        if (job_matches(&jobs[i], source, wire, size)) {
            job = &jobs[i];
            break;
        }
    }
    if (!job)
        return -ENOENT; /* Never transmit an unencrypted staging buffer. */
    if (job->done) {
        memcpy(wire, job->wire, job->size);
        memcpy(fifo_wire, job->wire, job->size);
        return 0;
    }
    tx_input[0] = 0;
    tx_input[1] = job->plain_size;
    tx_input[2] = 0;
    memcpy(tx_input + NEMU_LINK_CCM_DMA_HEADER_SIZE, job->plain, job->plain_size);
    active_tx = job;
    tx_wire = wire;
    tx_fifo_wire = fifo_wire;
    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk;
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_TX_CCM_BEGIN, size);
    int rc = nemu_link_nrf_ccm_radio_begin(false, job->key, job->nonce, tx_input, true);
    if (rc) {
        NRF_RADIO->INTENCLR = RADIO_INTENCLR_READY_Msk;
        active_tx = NULL;
        return rc;
    }
    return 1;
}

static int finish_wait(uint8_t *out, size_t capacity) {
    int rc = -EINPROGRESS;
    /* Expected completion is during ramp/reception. Bound a slow/error case;
     * completion is polled in the RADIO ISR, not dependent on a lower IRQ. */
    for (unsigned i = 0; i < CCM_FINISH_POLLS; i++) {
        rc = nemu_link_nrf_ccm_radio_finish(out, capacity);
        if (rc != -EINPROGRESS)
            return rc;
        k_busy_wait(1);
    }
    nemu_link_nrf_ccm_radio_cancel();
    return -ETIMEDOUT;
}

int nemu_link_ccm_radio_tx_ready(void) {
    if (!active_tx)
        return -ENOENT;
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_TX_READY, 0);
    int rc = finish_wait(active_tx->wire + active_tx->header,
                         NEMU_LINK_FRAME_MAX_SIZE - active_tx->header);
    if (!rc) {
        active_tx->done = true;
        memcpy(tx_wire, active_tx->wire, active_tx->size);
        memcpy(tx_fifo_wire, active_tx->wire, active_tx->size);
    }
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_TX_CCM_END, (uint16_t)-rc);
    nemu_link_zeroize(tx_input, sizeof(tx_input));
    active_tx = NULL;
    tx_wire = tx_fifo_wire = NULL;
    return rc;
}

void nemu_link_ccm_radio_rx_arm(uint8_t *pdu, uint8_t source) {
    nemu_link_nrf_ccm_radio_cancel();
    memset(&rx, 0, sizeof(rx));
    if (!snapshot.valid || snapshot.source != source ||
        snapshot.previous >= NEMU_LINK_COUNTER_REKEY_AT - 1)
        return;
    receive_snapshot = snapshot;
    rx.pdu = pdu;
    rx.source = source;
    rx.predicted = snapshot.previous + 1;
    memcpy(rx.key, snapshot.mini ? snapshot.key : snapshot.full_key, NEMU_LINK_AES_KEY_SIZE);
    sys_put_le32(rx.predicted, rx.nonce);
    rx.nonce[NEMU_LINK_CCM_DIRECTION_OFFSET] = source | NEMU_LINK_CCM_DIRECTION_BIT;
    memcpy(rx.nonce + NEMU_LINK_CCM_COUNTER_SIZE, snapshot.mini ? snapshot.iv : snapshot.full_iv,
           NEMU_LINK_CCM_IV_SIZE);
    if (snapshot.mini) {
        rx.nonce[NEMU_LINK_CCM_COUNTER_SIZE] ^= (uint8_t)(snapshot.reference >> 16);
        rx.nonce[NEMU_LINK_CCM_COUNTER_SIZE + 1] ^= (uint8_t)(snapshot.reference >> 24);
    }
    /* The SDK RX PDU begins at aligned storage + 3. Its +1 pointer is an
     * aligned CCM header, and both DMA engines share the ciphertext bytes. */
    tx_input[0] = tx_input[2] = 0;
    tx_input[1] = snapshot.mini ? NEMU_LINK_MINI_ACK_PLAIN_SIZE + NEMU_LINK_CCM_MIC_SIZE
                                : NEMU_LINK_COMPACT_CIPHER_MAX;
    int rc = nemu_link_nrf_ccm_radio_begin(true, rx.key, rx.nonce, tx_input, false);
    if (rc)
        return; /* The worker retains the checked synchronous fallback. */
    rx.armed = true;
    NRF_RADIO->EVENTS_BCMATCH = 0;
    /* 8-bit length + 3-bit S1 + counter16 + first cipher byte8. The extra
     * byte ensures both counter bytes have already reached RAM. */
    NRF_RADIO->BCC = NEMU_LINK_RADIO_MINI_PREFIX_BITS;
    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_BCSTART_Msk;
    NRF_RADIO->INTENSET = RADIO_INTENSET_BCMATCH_Msk;
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_RX_CCM_ARM, 0);
}

void nemu_link_ccm_radio_rx_prefix(void) {
    if (!rx.armed)
        return;
    size_t header;
    uint32_t counter;
    uint8_t iv[NEMU_LINK_CCM_NONCE_SIZE], key[NEMU_LINK_AES_KEY_SIZE];
    if (rx.pdu[0] == NEMU_LINK_MINI_ACK_SIZE && receive_snapshot.mini) {
        header = NEMU_LINK_MINI_COUNTER_SIZE;
        uint16_t wire_counter = sys_get_le16(rx.pdu + NEMU_LINK_RADIO_PDU_HEADER_SIZE);
        uint32_t high;
        if (!nemu_link_mini_down_forward(receive_snapshot.previous, wire_counter, &counter) ||
            !nemu_link_mini_ack_high(receive_snapshot.reference, wire_counter, &high))
            return;
        memcpy(key, receive_snapshot.key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(iv + NEMU_LINK_CCM_COUNTER_SIZE, receive_snapshot.iv, NEMU_LINK_CCM_IV_SIZE);
        iv[NEMU_LINK_CCM_COUNTER_SIZE] ^= (uint8_t)high;
        iv[NEMU_LINK_CCM_COUNTER_SIZE + 1] ^= (uint8_t)(high >> 8);
    } else if (rx.pdu[0] >= NEMU_LINK_COMPACT_MIN_SIZE &&
               rx.pdu[0] <= NEMU_LINK_CCM_FULL_WIRE_MAX &&
               rx.pdu[NEMU_LINK_RADIO_PDU_HEADER_SIZE] == NEMU_LINK_SECURE_WIRE_VERSION &&
               rx.pdu[NEMU_LINK_RADIO_PDU_HEADER_SIZE + NEMU_LINK_COMPACT_KIND_OFFSET] >=
                   NEMU_LINK_WIRE_FIRST_KIND &&
               rx.pdu[NEMU_LINK_RADIO_PDU_HEADER_SIZE + NEMU_LINK_COMPACT_KIND_OFFSET] <=
                   NEMU_LINK_WIRE_AUDIO_CONTROL) {
        if (NRF_RADIO->BCC == NEMU_LINK_RADIO_MINI_PREFIX_BITS) {
            /* Version/kind arrived, wait for the complete clear counter. */
            NRF_RADIO->BCC = NEMU_LINK_RADIO_FULL_PREFIX_BITS;
            NRF_RADIO->EVENTS_BCMATCH = 0;
            NRF_RADIO->INTENSET = RADIO_INTENSET_BCMATCH_Msk;
            return;
        }
        header = NEMU_LINK_COMPACT_HEADER_SIZE;
        counter = sys_get_le32(rx.pdu + NEMU_LINK_RADIO_PDU_HEADER_SIZE +
                               NEMU_LINK_COMPACT_COUNTER_OFFSET);
        if (!counter || counter <= receive_snapshot.previous ||
            counter >= NEMU_LINK_COUNTER_REKEY_AT)
            return;
        memcpy(key, receive_snapshot.full_key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(iv + NEMU_LINK_CCM_COUNTER_SIZE, receive_snapshot.full_iv, NEMU_LINK_CCM_IV_SIZE);
    } else {
        return;
    }
    sys_put_le32(counter, iv);
    iv[NEMU_LINK_CCM_DIRECTION_OFFSET] = rx.source | NEMU_LINK_CCM_DIRECTION_BIT;
    rx.size = rx.pdu[0];
    rx.prefix = rx.pdu + NEMU_LINK_RADIO_PDU_HEADER_SIZE + header - NEMU_LINK_CCM_DMA_HEADER_SIZE;
    memcpy(rx.saved, rx.prefix, NEMU_LINK_CCM_DMA_HEADER_SIZE);
    rx.prefix[0] = 0;
    rx.prefix[1] = rx.size - header;
    rx.prefix[2] = 0;
    rx.mutated = true;
    bool predicted = !memcmp(key, rx.key, NEMU_LINK_AES_KEY_SIZE) &&
                     !memcmp(iv, rx.nonce, NEMU_LINK_CCM_NONCE_SIZE);
    if (predicted) {
        nemu_link_nrf_ccm_radio_input(rx.prefix);
    } else {
        nemu_link_nrf_ccm_radio_cancel();
        rx.armed = nemu_link_nrf_ccm_radio_begin(true, key, iv, rx.prefix, false) == 0;
    }
    memcpy(rx.key, key, NEMU_LINK_AES_KEY_SIZE);
    memcpy(rx.nonce, iv, NEMU_LINK_CCM_NONCE_SIZE);
    nemu_link_zeroize(key, sizeof(key));
    nemu_link_zeroize(iv, sizeof(iv));
    __DSB();
    rx.started = rx.armed && nemu_link_nrf_ccm_radio_crypt() == 0;
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_RX_CCM_BEGIN, rx.started);
}

void nemu_link_ccm_radio_rx_finish(bool crc_ok) {
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_RX_END, crc_ok);
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_BCMATCH_Msk;
    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_ADDRESS_BCSTART_Msk;
    if (!rx.armed && !rx.mutated)
        return;
    uint8_t plain[NEMU_LINK_CCM_PLAIN_MAX];
    int rc = rx.started && crc_ok ? finish_wait(plain, sizeof(plain)) : -ECANCELED;
    if (rx.mutated)
        memcpy(rx.prefix, rx.saved, NEMU_LINK_CCM_DMA_HEADER_SIZE);
    if (rx.started && crc_ok) {
        struct rx_result *r = &results[result_next++ % RX_RESULTS];
        /* Taken results are already wiped; still erase an unread entry when
         * the bounded ring overwrites it before reusing its storage. */
        if (r->used)
            nemu_link_zeroize(r, sizeof(*r));
        r->used = true;
        r->source = rx.source;
        r->error = rc;
        memcpy(r->key, rx.key, NEMU_LINK_AES_KEY_SIZE);
        memcpy(r->nonce, rx.nonce, NEMU_LINK_CCM_NONCE_SIZE);
        r->size = rx.size;
        memcpy(r->wire, rx.pdu + NEMU_LINK_RADIO_PDU_HEADER_SIZE, rx.size);
        if (!rc)
            memcpy(r->plain, plain,
                   rx.size -
                       (rx.size == NEMU_LINK_MINI_ACK_SIZE ? NEMU_LINK_MINI_COUNTER_SIZE
                                                           : NEMU_LINK_COMPACT_HEADER_SIZE) -
                       NEMU_LINK_CCM_MIC_SIZE);
    }
    nemu_link_nrf_ccm_radio_cancel();
    nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_RX_CCM_END, (uint16_t)-rc);
    nemu_link_zeroize(plain, sizeof(plain));
    nemu_link_zeroize(&rx, sizeof(rx));
}

int nemu_link_ccm_radio_rx_take(uint8_t source, const uint8_t key[NEMU_LINK_AES_KEY_SIZE],
                                const uint8_t nonce[NEMU_LINK_CCM_NONCE_SIZE], const uint8_t *wire,
                                size_t size, uint8_t *plain, size_t capacity) {
    if (size != NEMU_LINK_MINI_ACK_SIZE && !secured_wire(wire, size))
        return -ENOENT;
    unsigned int locked = irq_lock();
    for (unsigned i = 0; i < RX_RESULTS; i++) {
        struct rx_result *r = &results[i];
        if (!r->used || r->source != source || r->size != size || memcmp(r->wire, wire, size) ||
            memcmp(r->key, key, NEMU_LINK_AES_KEY_SIZE) ||
            memcmp(r->nonce, nonce, NEMU_LINK_CCM_NONCE_SIZE))
            continue;
        int rc = r->error;
        size_t plain_size = size -
                            (size == NEMU_LINK_MINI_ACK_SIZE ? NEMU_LINK_MINI_COUNTER_SIZE
                                                             : NEMU_LINK_COMPACT_HEADER_SIZE) -
                            NEMU_LINK_CCM_MIC_SIZE;
        if (!rc && capacity < plain_size)
            rc = -EMSGSIZE;
        if (!rc)
            memcpy(plain, r->plain, plain_size);
        nemu_link_zeroize(r, sizeof(*r));
        nemu_link_trace_mark(NEMU_LINK_TRACE_RADIO_RX_CCM_CACHED, (uint16_t)-rc);
        irq_unlock(locked);
        return rc;
    }
    irq_unlock(locked);
    return -ENOENT;
}
