#pragma once

#include <zmk/dynamic_settings/store.h>

struct nemu_link_studio_packet;
struct nemu_link_studio_route;

/* Apply one remote Settings operation to this firmware's local schema.
 * The caller must first authenticate the transport peer and validate its
 * target/session policy. Owner schema identity is added to the response when
 * the requested group is an OWNER group. */
int nemu_link_settings_relay_dispatch(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response);

/* Synchronous Nemu-Link BLE split relay used by a central Studio endpoint for
 * owner-scoped proxy groups. */
int nemu_link_settings_ble_relay_call(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response);

/* ESB owner-only relay. The link adapter sends authenticated Studio frames to target_source. */
int nemu_link_settings_esb_relay_call(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response);
int nemu_link_settings_esb_receive_request(const struct nemu_link_studio_packet *packet,
                                           const struct nemu_link_studio_route *route);
int nemu_link_settings_esb_receive_response(const struct nemu_link_studio_packet *packet,
                                            uint8_t source);
int nemu_link_link_settings_esb_send(const struct nemu_link_studio_packet *packet, uint8_t source);
int nemu_link_link_settings_esb_reply(const struct nemu_link_studio_route *route,
                                      const struct nemu_link_studio_packet *packet);

enum nemu_link_settings_esb_rx_drop_reason {
    NEMU_LINK_SETTINGS_ESB_RX_DROP_ROUTE,
    NEMU_LINK_SETTINGS_ESB_RX_DROP_EXPIRED,
    NEMU_LINK_SETTINGS_ESB_RX_DROP_DECODE,
};

void nemu_link_settings_esb_note_link_rx_drop(bool response,
                                              enum nemu_link_settings_esb_rx_drop_reason reason);
void nemu_link_settings_esb_diag_log(void);
