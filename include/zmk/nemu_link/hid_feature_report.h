#pragma once
#include <stddef.h>
#include <stdint.h>

/* Vendor feature report shared by keyboard and standalone USB HID interfaces. */
#define NEMU_LINK_HID_FEATURE_REPORT_ID 0x7e
#define NEMU_LINK_HID_FEATURE_REPORT_SIZE 37
#define NEMU_LINK_HID_FEATURE_PROTOCOL_VERSION 1
#define NEMU_LINK_HID_FEATURE_MAGIC "NMLK"
#define NEMU_LINK_HID_FEATURE_ROLE_PERIPHERAL 1
#define NEMU_LINK_HID_FEATURE_ROLE_CENTRAL 2

/* GET: key registration status, identity, optional Dongle battery, shared radio
 * address and channel list. Bytes 31..35 report capabilities, count, and channels.
 * SET: key/address, channel count, and up to three channel numbers.
 * Size includes the report ID. */
int nemu_link_hid_feature_report_get(uint8_t out[NEMU_LINK_HID_FEATURE_REPORT_SIZE]);
int nemu_link_hid_feature_report_set(const uint8_t *report, size_t length);
