# Nordic SoC nodes share unit addresses (power/clock and ACL/flash-controller).
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
