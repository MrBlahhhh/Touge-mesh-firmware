#include "rideclock.h"

namespace touge {

bool cycleDividesSecond(uint32_t cycleMs) {
  return cycleMs > 0 && cycleMs <= 1000 && (1000 % cycleMs) == 0;
}

void RideClock::reset() {
  seq_ = 0;
  lastPulse_ = 0;
  pulses_ = 0;
  good_ = 0;
}

void RideClock::onPulse(uint64_t atMicros) {
  // Does this edge look like a second after the last one? A run of pulses that
  // do is the only thing separating a receiver from a floating pin.
  uint32_t run = 0;
  if (pulses_ > 0 && atMicros > lastPulse_) {
    uint64_t gap = atMicros - lastPulse_;
    for (uint32_t skip = 1; skip <= PULSE_MAX_SKIP; skip++) {
      uint64_t want = PULSE_PERIOD_US * skip;
      uint64_t off = gap > want ? gap - want : want - gap;
      if (off <= PULSE_TOLERANCE_US) {
        run = good_ + 1;
        break;
      }
    }
  }

  // Odd while writing. A reader that catches it here will see the counter move
  // and try again rather than read half of the old value and half of the new.
  seq_ = seq_ + 1;
  lastPulse_ = atMicros;
  pulses_ = pulses_ + 1;
  good_ = run;
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
  uint32_t good = 0;
  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t seq;
    if (!beginRead(seq)) continue;
    pulse = lastPulse_;
    good = good_;
    if (endRead(seq)) {
      if (seq == 0) return false; // no pulse has ever arrived
      // Not yet proved itself. A pin with nothing on it fires on noise, and a
      // board that believed it would announce itself as the one keeping time
      // for the ride.
      if (good < PULSE_LOCK_RUN) return false;
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
