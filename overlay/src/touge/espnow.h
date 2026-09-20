#pragma once
//
// The 2.4 GHz radio.
//
// ESP-NOW rather than a Wi-Fi network, because there is no access point on a
// mountain road and nobody wants to elect one. It is connectionless: a frame
// goes out to the broadcast address and every board on the same channel hears
// it, which is exactly the shape of a convoy.
//
// Range is the obvious objection. A 2.4 GHz receiver bottoms out near -95 dBm
// where LoRa at SF11 hears down to about -137, so this will never reach over a
// ridge the way LoRa does. It does not have to. Cars on the same road are in
// line of sight nearly all the time, and there this is hundreds of times
// faster and costs nothing in airtime that LoRa needs.
//
// Espressif's long-range PHY buys some of that back, and since every node here
// is an ESP32 running this same firmware, there is nobody else to stay
// compatible with.

#include <stdint.h>
#include <stddef.h>
#include "ride.h"
#include "frame.h"

namespace touge {

// Espressif's LR mode: a lower symbol rate that trades throughput for roughly
// 10 dB of link budget, which is a bit over three times the distance. It is
// proprietary, so an LR node and a plain node cannot hear each other at all.
// Every board on the ride runs this firmware, so that costs us nothing.
#ifndef TOUGE_FAST_LONG_RANGE
#define TOUGE_FAST_LONG_RANGE 1
#endif

struct FastRx {
  uint8_t data[FRAME_MAX];
  uint16_t len = 0;
  int8_t rssi = 0;
};

class FastRadio {
 public:
  // Brings up the Wi-Fi driver in station mode, parks it on the ride's
  // channel, and registers the broadcast peer. False if ESP-NOW would not
  // start, in which case the module carries on over LoRa alone rather than
  // taking the whole node down.
  bool begin(const FastNet& net);

  // Follows the ride onto a different channel when the PSK changes under us,
  // which happens when the phone reconfigures the primary channel mid-ride.
  bool retune(const FastNet& net);

  void end();

  bool ready() const { return ready_; }
  uint8_t channel() const { return channel_; }

  // Broadcast. Returns false if the driver rejected it, which on a busy
  // channel mostly means the transmit queue is full.
  bool send(const uint8_t* buf, size_t len);

  // Non-blocking. False when nothing is waiting.
  bool poll(FastRx& out);

  // Frames the driver handed us that we had nowhere to put. Worth watching:
  // a number that climbs means the module is not draining fast enough.
  uint32_t dropped() const;

 private:
  bool ready_ = false;
  uint8_t channel_ = 0;
};

// One radio, because there is one 2.4 GHz transceiver and the receive callback
// has to reach it from the Wi-Fi task without being handed a context pointer.
extern FastRadio fastRadio;

} // namespace touge
