#include "schedule.h"

namespace touge {

void Schedule::reset() {
  selfId_ = 0;
  referenceId_ = 0;
  slot_ = 0;
  known_ = 0;
  epochMs_ = 0;
  haveEpoch_ = false;
}

void Schedule::rebuild(uint32_t selfId, const Rider* riders, size_t maxRiders) {
  selfId_ = selfId;
  referenceId_ = selfId;

  // Our slot is our rank: how many node numbers on the ride sort below ours.
  // No sorting, no list, no allocation, and every car computing it over the
  // same roster gets the same answer.
  uint8_t rank = 0;
  uint8_t count = 1; // ourselves
  for (size_t i = 0; i < maxRiders; i++) {
    if (!riders[i].used) continue;
    // A rider whose id equals ours would make two cars share a slot forever.
    // That is a node number collision and there is nothing to be done about
    // it here, but it must not also corrupt the rank.
    if (riders[i].id == selfId) continue;
    count++;
    if (riders[i].id < selfId) rank++;
    if (riders[i].id < referenceId_) referenceId_ = riders[i].id;
  }

  // More cars than slots. The tail doubles up rather than falling off the end
  // of the cycle, which costs those two a collision and keeps everyone else
  // on schedule.
  slot_ = (uint8_t)(rank % MAX_SLOTS);
  known_ = count;
}

void Schedule::syncTo(uint32_t heardAtMs) {
  epochMs_ = heardAtMs;
  haveEpoch_ = true;
}

bool Schedule::inSlot(uint32_t nowMs, uint32_t cycleMs) const {
  if (cycleMs == 0) return false;

  // Alone, or holding slot zero and nobody to take the clock from: free-run.
  // There is no schedule worth keeping to and nothing to collide with, and
  // waiting for a sync that will never arrive would mean never transmitting.
  if (known_ <= 1) return true;
  if (weAreReference()) return true;
  if (!haveEpoch_) return true;

  uint32_t width = slotWidthMs(cycleMs);
  if (width == 0) return true;

  uint32_t phase = (uint32_t)(nowMs - epochMs_) % cycleMs;
  uint32_t start = (uint32_t)slot_ * width;
  return phase >= start && phase < start + width;
}

} // namespace touge
