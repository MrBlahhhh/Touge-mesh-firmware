#include "rideclock.h"

namespace touge {

bool cycleDividesSecond(uint32_t cycleMs) {
  return cycleMs > 0 && cycleMs <= 1000 && (1000 % cycleMs) == 0;
}

void RideClock::reset() {
  seq_ = 0;
  lastPulse_ = 0;
  pulses_ = 0;
}

void RideClock::onPulse(uint64_t atMicros) {
  // Odd while writing. A reader that catches it here will see the counter move
  // and try again rather than read half of the old value and half of the new.
  seq_ = seq_ + 1;
  lastPulse_ = atMicros;
  pulses_ = pulses_ + 1;
  seq_ = seq_ + 1;
}

bool RideClock::beginRead(uint32_t& seq) const {
  seq = seq_;
  return (seq & 1) == 0;
}

bool RideClock::endRead(uint32_t seq) const { return seq_ == seq; }

bool RideClock::locked(uint64_t nowMicros) const {
  uint32_t dummy;
  return phaseMs(nowMicros, 1000, dummy);
}

bool RideClock::phaseMs(uint64_t nowMicros, uint32_t cycleMs, uint32_t& out) const {
  if (!cycleDividesSecond(cycleMs)) return false;

  uint64_t pulse = 0;
  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t seq;
    if (!beginRead(seq)) continue;
    pulse = lastPulse_;
    if (endRead(seq)) {
      if (seq == 0) return false; // no pulse has ever arrived
      // A clock that has stopped still reports the same edge forever, which
      // would look locked while drifting a second further out every second.
      if (nowMicros < pulse) return false;
      uint64_t since = nowMicros - pulse;
      if (since > PULSE_STALE_US) return false;

      out = (uint32_t)((since / 1000ULL) % (uint64_t)cycleMs);
      return true;
    }
  }
  // Pulses are a second apart, so four collisions in a row is not contention.
  return false;
}

} // namespace touge
