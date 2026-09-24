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
  // Stamped in the driver callback, not in the polling loop. The schedule
  // recovers its clock from this, and the five milliseconds a poll can sit
  // waiting would be a fifth of a slot.
  uint32_t rxMs = 0;
  // The channel this frame actually arrived on.
  //
  // For the same reason as rxMs: a hop can happen between the driver taking
  // the frame and the module getting round to it, and asking the radio which
  // channel it is on at that point answers about the new one. Frames received
  // just before a hop were being filed under the channel the ride had moved
  // to, which is then counted as evidence that cars are already there.
  uint8_t chan = 0;
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

  // Move to a channel by number, for hopping and for searching. Separate from
  // retune because that one is about a change of ride and this one is not.
  bool retuneTo(uint8_t channel);

  void end();

  // end(), and the Wi-Fi driver and the receive queue handed back as well, for
  // a board that cannot spare the heap. begin() can start it all again.
  void shutdown();

  bool ready() const { return ready_; }
  // The Wi-Fi driver is initialised, whether or not ESP-NOW is running on it.
  bool driverUp() const;
  uint8_t channel() const { return channel_; }

  // Broadcast. Returns false if the driver rejected it, which on a busy
  // channel mostly means the transmit queue is full.
  bool send(const uint8_t* buf, size_t len);

  // Non-blocking. False when nothing is waiting.
  bool poll(FastRx& out);

  // Why begin() last failed (an esp_err_t, ESP_OK once it has started), and
  // the heap it had to work with at the time. A board that cannot start the
  // lane otherwise just says "LoRa only", which reads the same whether the
  // cause is memory, a Wi-Fi association or a bad key.
  int beginError() const;
  uint32_t beginFreeHeap() const;
  uint32_t beginLargestBlock() const;

  // Frames the driver handed us that we had nowhere to put. Worth watching:
  // a number that climbs means the module is not draining fast enough.
  uint32_t dropped() const;

  /**
   * Frames the driver refused to accept for transmission.
   *
   * A broadcast has no acknowledgement, so the send callback reports success
   * whatever happens in the air and is worth nothing here. The return of
   * esp_now_send is worth something: under saturation it is
   * ESP_ERR_ESPNOW_NO_MEM, and that was being discarded at every level - the
   * driver's return, send()'s return, and the module's. A board that had
   * stopped getting anything out looked perfectly healthy to itself, and the
   * only number on the status line counted receive drops.
   */
  uint32_t sendFailed() const;

  /**
   * What the radio says it is actually transmitting at, in whole dBm.
   *
   * Read back rather than remembered, because the interesting case is the one
   * where the driver did not give us what we asked for.
   */
  int8_t txPowerDbm() const;

  /** The last error the driver gave, for the status line. */
  int lastSendError() const;

 private:
  bool ready_ = false;
  uint8_t channel_ = 0;
};

// One radio, because there is one 2.4 GHz transceiver and the receive callback
// has to reach it from the Wi-Fi task without being handed a context pointer.
extern FastRadio fastRadio;

// Internal DMA-capable RAM, the pool BLE's controller and the Wi-Fi driver both
// allocate from: free now, the least it has been since boot, and the largest
// block one allocation could get.
struct DramHeap {
  uint32_t freeBytes = 0;
  uint32_t minFreeBytes = 0;
  uint32_t largestBlock = 0;
};
DramHeap dramHeap();

} // namespace touge
