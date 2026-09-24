#pragma once
//
// Whether this board has to count bytes.
//
// A board with PSRAM (Heltec V4) has room to spare. One without (Heltec V3)
// runs LoRa, BLE and the Wi-Fi driver side by side in internal SRAM, and BLE
// fails outright when it cannot allocate: on build 36 a V3 logged
// "BLE_INIT: Malloc failed" in the middle of a phone's config download and
// stopped answering its reads. On those boards the tables here are sized to a
// full ride and no further, and the Wi-Fi driver is started lean.
//
// BOARD_HAS_PSRAM comes from the board JSON, the same switch Meshtastic's
// Router.cpp sizes its packet pool on. Host tests use the roomy sizes unless the
// environment sets TOUGE_LEAN_RAM=1 (env:native-lean).

#ifndef TOUGE_LEAN_RAM
#if defined(BOARD_HAS_PSRAM) || defined(TOUGE_NATIVE)
#define TOUGE_LEAN_RAM 0
#else
#define TOUGE_LEAN_RAM 1
#endif
#endif
