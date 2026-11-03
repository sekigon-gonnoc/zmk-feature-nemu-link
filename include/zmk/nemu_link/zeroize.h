#pragma once
#include <stddef.h>

/* Erase exactly size bytes, including on unaligned buffers. Stores are volatile. */
void nemu_link_zeroize(void *buffer, size_t size);
