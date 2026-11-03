#pragma once
#include <stdbool.h>
#include <stdint.h>

#define NEMU_LINK_BATTERY_STATUS_SIZE 7
#define NEMU_LINK_BATTERY_STATUS_MAGIC 0x42
#define NEMU_LINK_BATTERY_STATUS_VERSION 1
#define NEMU_LINK_BATTERY_SOURCE_COUNT 2

/* Receiver updates these only from fresh authenticated STATE snapshots. */
void nemu_link_battery_update(uint8_t source, uint8_t level, bool connected);
void nemu_link_battery_disconnect(uint8_t source);
/* GET metadata: magic, version, count, known mask, connected mask, levels[2]. */
void nemu_link_battery_status_get(uint8_t out[NEMU_LINK_BATTERY_STATUS_SIZE]);
