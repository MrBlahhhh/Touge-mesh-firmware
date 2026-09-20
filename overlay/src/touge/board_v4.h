#pragma once
//
// Heltec WiFi LoRa 32 V4 pin map.
//
// Taken from the Meshtastic variant for board `heltec_v4`, which is the only
// map anyone has verified against real hardware. Guessing here is expensive:
// a wrong SPI pin is a radio that never answers, and a wrong front-end pin is
// a radio that answers, claims to transmit, and is heard by nobody.
//
// Anything below marked EXPANSION is not on the bare board.

#include <stdint.h>

// ---- SX1262 ----------------------------------------------------------------
#define PIN_LORA_CS 8
#define PIN_LORA_SCK 9
#define PIN_LORA_MOSI 10
#define PIN_LORA_MISO 11
#define PIN_LORA_RESET 12
#define PIN_LORA_BUSY 13
#define PIN_LORA_DIO1 14

// DIO2 drives the front end's transmit/receive select directly, and DIO3 feeds
// the TCXO at 1.8 V. Both are wired on the PCB, so RadioLib is told about them
// rather than given GPIOs to toggle.
#define LORA_TCXO_VOLTAGE 1.8f

// ---- RF front end ----------------------------------------------------------
//
// The V4 puts a PA and LNA between the SX1262 and the antenna. It is powered
// through an LDO on GPIO 7 and has a chip-enable on GPIO 2, and both revisions
// share those. What differs is the third pin:
//
//   V4.2  GC1109    GPIO 46 = CPS, high selects the full PA, low bypasses it
//   V4.3  KCT8103L  GPIO 5  = CTX, high bypasses the receive LNA, low uses it
//
// Note they mean opposite things. On a 4.2 the third pin must be HIGH to get
// any transmit gain; on a 4.3 it must be LOW to get any receive gain. Driving
// a 4.3 as though it were a 4.2 costs about 21 dB of receive sensitivity and
// presents as "the other car has to be within 200 metres".
#define PIN_FEM_POWER 7  // LDO enable for the front end, active high
#define PIN_FEM_ENABLE 2 // CSD, chip enable, active high
#define PIN_FEM_V42_CPS 46
#define PIN_FEM_V43_CTX 5

#define TOUGE_FEM_AUTO 0
#define TOUGE_FEM_V42 42
#define TOUGE_FEM_V43 43

// The SX1262 feeds the PA, so the number handed to the radio is not the number
// that leaves the antenna. Measured net gain is non-linear because the PA
// compresses: +11 dB up to 15 dBm in, falling to +7 dB by 21 dBm in. 21 in is
// 28 out, which is the high-power part's ceiling and the most a V4 will do.
#define LORA_PA_MAX_INPUT_DBM 21

// ---- OLED (SSD1315, driven as an SSD1306) ----------------------------------
#define PIN_OLED_SDA 17
#define PIN_OLED_SCL 18
#define PIN_OLED_RESET 21

// ---- Power and user IO -----------------------------------------------------
// Vext is active LOW and gates both the display and the LoRa antenna boost, so
// it has to come up before either is touched.
#define PIN_VEXT 36
#define VEXT_ON LOW
#define PIN_LED 35
#define PIN_BUTTON 0 // PRG

#define PIN_BATTERY 1
#define PIN_ADC_CTRL 37 // active high; gates the divider so it does not leak
#define BATTERY_MULTIPLIER (4.9f * 1.045f)

// ---- GNSS (EXPANSION) ------------------------------------------------------
//
// The SH1.25-8pin header is on the bare board but the receiver is not. The V4
// expansion kit carries a CM121 at 9600 baud. With nothing plugged in, these
// pins float and the NMEA reader simply never sees a sentence, which is the
// behaviour we want: standalone mode degrades to "no fix", not to a hang.
#define PIN_GPS_TX 38 // toward the GNSS
#define PIN_GPS_RX 39 // toward the CPU
#define PIN_GPS_RESET 42
#define PIN_GPS_ENABLE 34 // active low
#define PIN_GPS_STANDBY 40
#define PIN_GPS_PPS 41
#define GPS_BAUD 9600
