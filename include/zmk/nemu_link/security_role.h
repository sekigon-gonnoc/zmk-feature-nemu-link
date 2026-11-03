#pragma once
#include <stdbool.h>

/* A firmware image has one ESB radio role, even when the keyboard switches to
 * BLE. Portable simulations may compile both roles into a single endpoint model. */
#if defined(CONFIG_ZMK_NEMU_LINK) && CONFIG_ZMK_NEMU_LINK
#if defined(CONFIG_BOARD_NEMU_LINK_DONGLE) && CONFIG_BOARD_NEMU_LINK_DONGLE
#define NEMU_LINK_SECURITY_CENTRAL(role) true
#else
#define NEMU_LINK_SECURITY_CENTRAL(role) false
#endif
#else
#define NEMU_LINK_SECURITY_CENTRAL(role) (role)
#endif
