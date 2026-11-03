#include <zmk/nemu_link/dongle_core_api.h>
#include <zmk/nemu_link/hop.h>

#define NEMU_LINK_DONGLE_CORE_API                                                                  \
    ((const struct nemu_link_dongle_core_api *)(uintptr_t)UINT32_C(0x00001100))

bool nemu_link_link_central_init(struct nemu_link_link_central *central, uint32_t epoch,
                                 uint32_t release_ms,
                                 const uint32_t buttons[NEMU_LINK_LINK_SOURCES]) {
    return NEMU_LINK_DONGLE_CORE_API->link_central_init(central, epoch, release_ms, buttons);
}

void nemu_link_link_central_tick(struct nemu_link_link_central *central, uint8_t source,
                                 uint64_t now, struct nemu_link_link_result *out) {
    NEMU_LINK_DONGLE_CORE_API->link_central_tick(central, source, now, out);
}

bool nemu_link_link_central_receive(struct nemu_link_link_central *central, uint8_t source,
                                    const struct nemu_link_link_message *message, uint64_t now,
                                    struct nemu_link_link_result *out) {
    return NEMU_LINK_DONGLE_CORE_API->link_central_receive(central, source, message, now, out);
}

bool nemu_link_hop_central_init(struct nemu_link_hop_central *central, uint32_t boot, uint64_t now,
                                const struct nemu_link_hop_timing *timing) {
    bool initialized = NEMU_LINK_DONGLE_CORE_API->hop_central_init(central, boot, now, timing);
    if (initialized) {
        central->channel_count = nemu_link_hop_channel_count;
    }
    return initialized;
}

bool nemu_link_hop_central_request_index(struct nemu_link_hop_central *central, uint64_t now,
                                         uint8_t index) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_request_index(central, now, index);
}

bool nemu_link_hop_central_request(struct nemu_link_hop_central *central, uint64_t now) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_request(central, now);
}

struct nemu_link_hop_notice
nemu_link_hop_central_notice(const struct nemu_link_hop_central *central) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_notice(central);
}

bool nemu_link_hop_central_receive(struct nemu_link_hop_central *central, uint8_t source,
                                   uint64_t now, const struct nemu_link_hop_notice *echo) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_receive(central, source, now, echo);
}

bool nemu_link_hop_central_next(const struct nemu_link_hop_central *central, uint64_t now,
                                struct nemu_link_hop_notice *target) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_next(central, now, target);
}

bool nemu_link_hop_central_applied(struct nemu_link_hop_central *central, uint64_t now,
                                   const struct nemu_link_hop_notice *target) {
    return NEMU_LINK_DONGLE_CORE_API->hop_central_applied(central, now, target);
}

bool nemu_link_hop_link_central_receive(struct nemu_link_link_central *link,
                                        struct nemu_link_hop_central *hop, uint8_t source,
                                        const struct nemu_link_link_message *message,
                                        const struct nemu_link_hop_notice *notice, uint64_t now,
                                        struct nemu_link_link_result *out) {
    return NEMU_LINK_DONGLE_CORE_API->hop_link_central_receive(link, hop, source, message, notice,
                                                               now, out);
}
