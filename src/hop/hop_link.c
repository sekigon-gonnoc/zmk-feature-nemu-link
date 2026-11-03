#include <zmk/nemu_link/hop_link.h>
bool nemu_link_hop_link_peripheral_receive(struct nemu_link_link_peripheral *link,
                                           struct nemu_link_hop_peripheral *hop,
                                           const struct nemu_link_link_message *message,
                                           const struct nemu_link_hop_notice *notice,
                                           uint64_t now) {
    nemu_link_link_peripheral_tick(link, now);
    if (!message) {
        return false;
    }
    /* Validate the small hop candidate first. Link receive rejects before
     * committing input/lease fields; once it accepts, no validation remains.
     * This preserves transactionality without copying the 344-byte ledger. */
    struct nemu_link_hop_peripheral authority = *hop;
    if (message->kind == NEMU_LINK_LINK_CHALLENGE) {
        if (!nemu_link_hop_peripheral_bind(&authority, message->epoch)) {
            return false;
        }
    }
    if (!nemu_link_hop_peripheral_receive(&authority, message->epoch, now, notice)) {
        return false;
    }
    if (!nemu_link_link_peripheral_receive(link, now, message)) {
        return false;
    }
    *hop = authority;
    return true;
}
struct nemu_link_hop_notice nemu_link_hop_link_notice(const struct nemu_link_link_peripheral *link,
                                                      const struct nemu_link_hop_peripheral *hop) {
    if (link->phase == NEMU_LINK_LINK_PERIPHERAL_HELLO) {
        /* New dongle boots may restart channel generations at 1. Discovery must
         * not require knowing that new boot before receiving its CHALLENGE. */
        return (struct nemu_link_hop_notice){.index = hop->index, .kind = NEMU_LINK_HOP_PROBE};
    }
    return nemu_link_hop_peripheral_notice(hop);
}
