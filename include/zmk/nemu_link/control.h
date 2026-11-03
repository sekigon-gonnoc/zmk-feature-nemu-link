#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
struct zmk_position_state_changed;
struct sensor_value;
/* Register once before input. Callbacks must not block or re-enter the API.
 * source_lost runs before source release dispatch. */
struct zmk_nemu_link_observer {
    void (*source_lost)(uint8_t source);
    void (*mode_requested)(bool esb);
    void (*mode_changed)(bool esb, int error);
};
int zmk_nemu_link_register_observer(const struct zmk_nemu_link_observer *observer);
/* Thread context. Zero means accepted/no change, not radio transition complete. */
int zmk_nemu_link_request_mode(bool esb);
int zmk_nemu_link_request_peer_mode(bool esb);
bool zmk_nemu_link_owns_radio(void);
struct zmk_nemu_link_audio_callbacks {
    void (*fragment)(uint8_t source, uint16_t block_sequence, uint8_t fragment,
                     uint8_t fragment_count, const uint8_t *pcm, size_t size);
    /* origin is carried inside the authenticated control payload. */
    void (*control)(uint8_t origin, bool enabled);
    void (*source_lost)(uint8_t source);
};
int zmk_nemu_link_register_audio_callbacks(const struct zmk_nemu_link_audio_callbacks *callbacks);
int zmk_nemu_link_audio_control(bool enabled, uint8_t origin);
/* Send one authenticated audio fragment from an ESB peripheral. A zero return
 * means it was queued; it does not mean the receiver played it. */
int zmk_nemu_link_audio_send(uint16_t block_sequence, uint8_t fragment, uint8_t fragment_count,
                             const uint8_t *pcm, size_t size);
void zmk_nemu_link_position(const struct zmk_position_state_changed *event);
int zmk_nemu_link_sensor_event(uint8_t sensor_index, uint32_t channel,
                               const struct sensor_value *value);
/* USB-device readiness gate. Atomic notification; the worker starts the radio. */
void zmk_nemu_link_set_receiver_ready(bool ready);
/* Fixed-ESB Dev HID state bits: logical/radio/USB readiness and peer phase. */
uint8_t zmk_nemu_link_debug_state(void);
/* Fixed ZMK compatibility hooks, implemented by the explicit patch set. */
int zmk_nemu_link_ble_split_resume(void);
int zmk_nemu_link_ble_host_resume(void);

struct input_event;
int zmk_nemu_link_input(uint8_t reg, const struct input_event *event);
