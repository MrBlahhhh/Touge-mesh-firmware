#include "ownfix.h"

namespace touge {
namespace {

bool sameFix(const Fix& a, const Fix& b) {
  return a.lat == b.lat && a.lon == b.lon && a.trackE5 == b.trackE5 && a.speedKmh == b.speedKmh &&
         a.fixSec == b.fixSec && a.fixMs == b.fixMs && a.external == b.external;
}

}  // namespace

void setMeasured(Fix& fix, uint32_t timestamp, int32_t millisAdjust, uint32_t time) {
  if (timestamp == 0) {
    fix.fixSec = time;
    fix.fixMs = 0;
    return;
  }
  // The adjustment is signed and meant to be under a second; anything else is
  // folded into the seconds rather than trusted as milliseconds.
  int64_t ms = (int64_t)timestamp * 1000 + millisAdjust;
  if (ms < 0) ms = 0;
  fix.fixSec = (uint32_t)(ms / 1000);
  fix.fixMs = (uint16_t)(ms % 1000);
}

bool measuredAfter(const Fix& a, const Fix& b) {
  if (a.fixSec == 0 || b.fixSec == 0) return false;
  return a.fixSec != b.fixSec ? a.fixSec > b.fixSec : a.fixMs > b.fixMs;
}

void OwnFix::observe(const Fix& reading, uint32_t entropy) {
  if (reading.lat == 0 && reading.lon == 0) {
    has_ = false;
    return;
  }
  // The same fix read again, on the next pass or back after a lost lock.
  if (seq_ != 0 && sameFix(reading, fix_)) {
    has_ = true;
    return;
  }
  if (session_ == 0) {
    session_ = (uint16_t)(entropy ^ (entropy >> 16));
    if (session_ == 0) session_ = 1;
  }
  fix_ = reading;
  seq_++;
  has_ = true;
}

FixId OwnFix::id() const {
  FixId named;
  named.session = session_;
  named.seq = seq_;
  named.fixSec = fix_.fixSec;
  named.fixMs = fix_.fixMs;
  return named;
}

}  // namespace touge
