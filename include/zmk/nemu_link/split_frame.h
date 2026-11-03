#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define NEMU_LINK_SPLIT_FRAME_SIZE 64
#define NEMU_LINK_SPLIT_DATA_SIZE 35
#define NEMU_LINK_SPLIT_EVENT 0x81
#define NEMU_LINK_SPLIT_COMMAND 0x82
struct nemu_link_split_frame {
    uint8_t kind, source, size;
    uint32_t epoch, session, nonce, sequence;
    uint8_t data[NEMU_LINK_SPLIT_DATA_SIZE];
};
/* Encode returns transmitted byte count, or zero on error. Decode takes actual RX length. */
size_t nemu_link_split_encode(uint8_t wire[64], const struct nemu_link_split_frame *frame,
                              uint32_t (*crc)(const uint8_t *, size_t));
bool nemu_link_split_decode(struct nemu_link_split_frame *frame, const uint8_t *wire, size_t size,
                            uint32_t (*crc)(const uint8_t *, size_t));

struct nemu_link_split_rx {
    uint32_t epoch, session, nonce, sequence;
};
/* Reject replay across reconnect/boot; gaps are accepted without delta replay. */
bool nemu_link_split_accept(struct nemu_link_split_rx *rx,
                            const struct nemu_link_split_frame *expected,
                            const struct nemu_link_split_frame *frame);
