#include <stdint.h>
#include <zmk/nemu_link/zeroize.h>

/* GCC/Clang aliasing extension permits clearing any object representation with
 * aligned word stores. Volatile prevents dead-store elimination, including LTO. */
typedef uint32_t erase_word __attribute__((__may_alias__));

void nemu_link_zeroize(void *buffer, size_t size) {
    volatile uint8_t *bytes = buffer;
    while (size && ((uintptr_t)bytes & 3U)) {
        *bytes++ = 0;
        size--;
    }
    while (size >= sizeof(erase_word)) {
        *(volatile erase_word *)bytes = 0;
        bytes += sizeof(erase_word);
        size -= sizeof(erase_word);
    }
    while (size--) {
        *bytes++ = 0;
    }
}
