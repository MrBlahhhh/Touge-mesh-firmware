#pragma once
//
// The cycle clock, disciplined to GPS.
//
// Recovering the clock from a reference car's beacons works, but it buys two
// problems: the accuracy is only as good as the jitter on the path the beacon
// took, and the whole ride depends on one car staying in range. When that car
// turns off down a side road everyone has to notice, re-elect, and resync.
//
// A GNSS receiver's pulse-per-second output solves both. Every board already
// has one, the edge marks the UTC second to well under a microsecond, and
// there is nothing to elect: cars that have never heard each other are already
// in step before they meet.
//
// ## Why no UTC seconds are needed
//
// Only the position inside the cycle matters, never the absolute date. As long
// as the cycle divides a second exactly, every second boundary is also a cycle
// boundary, so the time since the last pulse is enough on its own. That also
// makes a missed pulse harmless: 1200 ms after the last edge is still 200 ms
// into a cycle.
//
// Which is why the cycle length is not free. See cycleDividesSecond.

#include <stdint.h>

namespace touge {

// One missed pulse is tolerated, two is not. A receiver that has lost its fix
// keeps the last edge frozen, and a frozen clock is worse than no clock: it
// looks locked while drifting further out of step every second.
static const uint64_t PULSE_STALE_US = 2500000;

// A cycle that does not divide a second would put the cycle boundary in a
// different place after every pulse, and the schedule would walk.
bool cycleDividesSecond(uint32_t cycleMs);

class RideClock {
 public:
  void reset();

  // From the interrupt handler, with the microsecond count at the edge.
  void onPulse(uint64_t atMicros);

  bool locked(uint64_t nowMicros) const;

  // Where we are inside the current cycle. False when there has been no recent
  // pulse, which is the caller's cue to fall back to the beacon-recovered
  // clock rather than to stop transmitting.
  bool phaseMs(uint64_t nowMicros, uint32_t cycleMs, uint32_t& out) const;

  uint32_t pulses() const { return pulses_; }

 private:
  // Written in an interrupt and read in a task, and a 64-bit value is two
  // stores on a 32-bit core. A torn read here would put the clock a very long
  // way out rather than slightly out, so the reader retries instead: the
  // counter is odd for as long as a write is in progress, and a reader that
  // sees it move across the read starts again.
  bool beginRead(uint32_t& seq) const;
  bool endRead(uint32_t seq) const;

  volatile uint32_t seq_ = 0;
  volatile uint64_t lastPulse_ = 0;
  volatile uint32_t pulses_ = 0;
};

} // namespace touge
