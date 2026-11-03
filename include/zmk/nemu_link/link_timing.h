#ifndef ZMK_NEMU_LINK_TIMING_H
#define ZMK_NEMU_LINK_TIMING_H

/* Firmware configuration lives here, outside the portable protocol core.
 * Host integration tests supply the same CONFIG values as firmware builds. */
#define NEMU_LINK_LINK_INTERVAL_MS CONFIG_ZMK_NEMU_LINK_LINK_INTERVAL_MS
#define NEMU_LINK_LINK_RELEASE_MS                                                                  \
    (NEMU_LINK_LINK_INTERVAL_MS * CONFIG_ZMK_NEMU_LINK_LINK_TIMEOUT_INTERVALS)
/* Keep three regular snapshots available to echo a received BLE command.
 * BLE -> ESB startup retains its independent 350ms settling delay. */
#define NEMU_LINK_LINK_MODE_SETTLE_MS (3 * NEMU_LINK_LINK_INTERVAL_MS + 50)

#endif
