#!/usr/bin/env python3
"""Generate checked PTX hooks in a build copy; never alter the Nordic SDK."""
import argparse
import hashlib
import re
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--scan", action="store_true")
parser.add_argument("--scan-only", action="store_true")
args = parser.parse_args()
source = args.source.read_text()
digest = hashlib.sha256(args.source.read_bytes()).hexdigest()


def replace(old, new, count=1):
    global source
    parts = re.split(r"(\s+)", old.strip())
    pattern = re.compile("".join(r"\s+" if part.isspace() else re.escape(part) for part in parts))
    actual = len(list(pattern.finditer(source)))
    if actual != count:
        raise SystemExit(f"ESB CCM hook anchor mismatch: expected {count}, got {actual}: {old!r}")
    source = pattern.sub(lambda _match: new, source, count=count)


if not args.scan_only:
    replace('#include <esb.h>', '#include <esb.h>\n#include <zmk/nemu_link/ccm_radio.h>')
    replace('static void start_tx_transaction(void)\n{',
            '''static void set_evt_interrupt(void);
    static void mini_ccm_tx_abort(void)
    {
        esb_ppi_for_txrx_clear(false, true);
        esb_ppi_for_wait_for_ack_clear();
        esb_ppi_for_retransmission_clear();
        esb_ppi_for_fem_clear();
        mpsl_fem_deactivate_now(MPSL_FEM_ALL);
        mpsl_fem_disable();
        nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
        nrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_READY_MASK | NRF_RADIO_INT_BCMATCH_MASK);
        on_radio_disabled = NULL;
        nrf_radio_shorts_set(NRF_RADIO, 0);
        nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
        interrupt_flags |= INT_TX_FAILED_MSK;
        last_tx_attempts = 0;
        esb_state = ESB_STATE_IDLE;
        errata216_off();
        set_evt_interrupt();
    }

    static void start_tx_transaction(void)
    {''')
    replace('static uint8_t rx_payload_buffer[CONFIG_ESB_MAX_PAYLOAD_LENGTH +\n'
            '\t\t\t\t sizeof(struct esb_radio_pdu)];',
            'static uint8_t rx_ccm_storage[CONFIG_ESB_MAX_PAYLOAD_LENGTH + NEMU_LINK_RADIO_RX_PDU_OFFSET + NEMU_LINK_RADIO_PDU_HEADER_SIZE] __aligned(4);\n'
            'static uint8_t *const rx_payload_buffer = rx_ccm_storage + NEMU_LINK_RADIO_RX_PDU_OFFSET;\n'
            'BUILD_ASSERT(CONFIG_ESB_MAX_PAYLOAD_LENGTH == NEMU_LINK_FRAME_MAX_SIZE, "mini BCC expects length8/S1=3");')
    replace('\tnrf_radio_packetptr_set(NRF_RADIO, pdu);\n',
            '\tnrf_radio_packetptr_set(NRF_RADIO, pdu);\n'
            '\tint mini_ccm = nemu_link_ccm_radio_tx_prepare(pdu->data, current_payload->data,\n'
            '\t\tcurrent_payload->length, current_payload->pipe);\n'
            '\tif (mini_ccm < 0) {\n'
            '\t\tmini_ccm_tx_abort();\n'
            '\t\treturn;\n'
            '\t}\n')
    # Arm only PTX reception, not the continuous multi-pipe PRX path.
    replace('\ton_radio_disabled = on_radio_disabled_tx_wait_for_ack;\n',
            '\tnemu_link_ccm_radio_rx_arm(rx_payload_buffer, current_payload->pipe);\n'
            '\ton_radio_disabled = on_radio_disabled_tx_wait_for_ack;\n')
    replace('static void on_radio_disabled_tx_wait_for_ack(void)\n{',
            'static void on_radio_disabled_tx_wait_for_ack(void)\n{\n'
            '\tnemu_link_ccm_radio_rx_finish(nrf_radio_event_check(NRF_RADIO, ESB_RADIO_EVENT_END) &&\n'
            '\t\tnrf_radio_crc_status_check(NRF_RADIO));')
    replace('static void radio_irq_handler(void)\n{',
            'static void radio_irq_handler(void)\n{\n'
            '\tif (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_READY_MASK) &&\n'
            '\t    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_READY)) {\n'
            '\t\tnrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_READY_MASK);\n'
            '\t\tnrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);\n'
            '\t\tif (nemu_link_ccm_radio_tx_ready() == 0) {\n'
            '\t\t\tnrf_radio_shorts_enable(NRF_RADIO, NRF_RADIO_SHORT_READY_START_MASK);\n'
            '\t\t\tnrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);\n'
            '\t\t} else {\n'
            '\t\t\tmini_ccm_tx_abort();\n'
            '\t\t}\n'
            '\t}\n'
            '\tif (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_BCMATCH_MASK) &&\n'
            '\t    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_BCMATCH)) {\n'
            '\t\tnrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_BCMATCH_MASK);\n'
            '\t\tnrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_BCMATCH);\n'
            '\t\tnemu_link_ccm_radio_rx_prefix();\n'
            '\t}\n')
if args.scan or args.scan_only:
    replace('int esb_start_rx(void)\n{',
            """/* Called with IRQs locked; preserve FIFOs and duplicate detection. */
bool nemu_link_esb_scan_ready(void)
{
    return esb_state == ESB_STATE_PRX && interrupt_flags == 0 && rx_fifo.count == 0 &&
        nrf_radio_state_get(NRF_RADIO) == NRF_RADIO_STATE_RX &&
        !nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS) &&
        !NVIC_GetPendingIRQ(ESB_RADIO_IRQ_NUMBER) && !NVIC_GetPendingIRQ(ESB_EVT_IRQ_NUMBER);
}

int esb_start_rx(void)
{""")
    for signature in ['static void clear_events_restart_rx(void)\n{',
                      'static void on_radio_disabled_rx_ack(void)\n{']:
        replace(signature, signature + '\n\tnrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);')

args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(f"/* Build copy of {args.source.name}, upstream sha256 {digest}. */\n" + source)
print(f"Nemu-link CCM: verified source anchors; upstream SHA256 {digest}")
