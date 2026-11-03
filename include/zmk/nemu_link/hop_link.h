#pragma once
#include <zmk/nemu_link/hop.h>
#include <zmk/nemu_link/link.h>
/* Validate both layers before exposing keys or renewing application liveness.
 * Even rejected controls must run the independent link expiry path. */
bool nemu_link_hop_link_central_receive(struct nemu_link_link_central *link,
                                        struct nemu_link_hop_central *hop, uint8_t source,
                                        const struct nemu_link_link_message *message,
                                        const struct nemu_link_hop_notice *notice, uint64_t now,
                                        struct nemu_link_link_result *out);
bool nemu_link_hop_link_peripheral_receive(struct nemu_link_link_peripheral *link,
                                           struct nemu_link_hop_peripheral *hop,
                                           const struct nemu_link_link_message *message,
                                           const struct nemu_link_hop_notice *notice, uint64_t now);
struct nemu_link_hop_notice nemu_link_hop_link_notice(const struct nemu_link_link_peripheral *link,
                                                      const struct nemu_link_hop_peripheral *hop);
