#include "schedule.h"

namespace touge {

void Schedule::reset() {
  selfId_ = 0;
  referenceId_ = 0;
  slot_ = 0;
  referenceSlot_ = 0;
  known_ = 0;
  epochMs_ = 0;
  haveEpoch_ = false;
}

void Schedule::rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders) {
  selfId_ = selfId;

  // Our slot is our rank: how many node numbers on the ride sort below ours.
  // No sorting, no list, no allocation, and every car computing it over the
  // same roster gets the same answer.
  uint8_t rank = 0;
  uint8_t count = 1; // ourselves

  // The reference is picked twice over, because on a mixed ride it has to be a
  // car whose own clock is locked to GPS.
  //
  // Cars with a fix take their cycle from the pulse. Cars without take it from
  // the reference car's beacons. If the reference is itself free-running, those
  // two groups end up on cycles that have nothing to do with each other and
  // collide systematically rather than occasionally, which is worse than
  // having no schedule at all. So a locked car always wins the job, and the
  // lowest node number only breaks the tie.
  uint32_t lowestLocked = selfLocked ? selfId : 0;
  bool anyLocked = selfLocked;
  uint32_t lowestAny = selfId;

  for (size_t i = 0; i < maxRiders; i++) {
    if (!riders[i].used) continue;
    // A rider whose id equals ours would make two cars share a slot forever.
    // That is a node number collision and there is nothing to be done about
    // it here, but it must not also corrupt the rank.
    if (riders[i].id == selfId) continue;
    count++;
    if (riders[i].id < selfId) rank++;
    if (riders[i].id < lowestAny) lowestAny = riders[i].id;
    if (riders[i].pos.clockLocked && (!anyLocked || riders[i].id < lowestLocked)) {
      lowestLocked = riders[i].id;
      anyLocked = true;
    }
  }

  // Nobody has a fix: everyone is on the beacon-recovered cycle, so the lowest
  // number does the job and they all agree because none of them has anything
  // better to agree on.
  referenceId_ = anyLocked ? lowestLocked : lowestAny;

  // Where the reference sits in the running order. It used to be slot zero by
  // construction, back when it was simply the lowest number; now that a locked
  // car can outrank a lower-numbered unlocked one, its beacon marks its own
  // slot rather than the start of the cycle, and syncTo has to subtract it.
  uint8_t refRank = 0;
  if (referenceId_ != selfId && selfId < referenceId_) refRank++;
  for (size_t i = 0; i < maxRiders; i++) {
    if (!riders[i].used) continue;
    if (riders[i].id == selfId) continue;
    if (riders[i].id == referenceId_) continue;
    if (riders[i].id < referenceId_) refRank++;
  }
  referenceSlot_ = (uint8_t)(refRank % MAX_SLOTS);

  // More cars than slots. The tail doubles up rather than falling off the end
  // of the cycle, which costs those two a collision and keeps everyone else
  // on schedule.
  slot_ = (uint8_t)(rank % MAX_SLOTS);
  known_ = count;
}

void Schedule::syncTo(uint32_t heardAtMs, uint32_t cycleMs) {
  // Back out the reference's own slot to get the start of the cycle. When the
  // reference holds slot zero this subtracts nothing, which is the case it
  // used to be able to assume.
  epochMs_ = heardAtMs - (uint32_t)referenceSlot_ * slotWidthMs(cycleMs);
  haveEpoch_ = true;
}

bool Schedule::inSlotAtPhase(uint32_t phaseMs, uint32_t cycleMs) const {
  if (cycleMs == 0) return false;
  // Alone on the channel there is nothing to collide with, so there is no
  // reason to sit out the other eight slots.
  if (known_ <= 1) return true;

  uint32_t width = slotWidthMs(cycleMs);
  if (width == 0) return true;

  uint32_t start = (uint32_t)slot_ * width;
  return phaseMs >= start && phaseMs < start + width;
}

bool Schedule::inSlot(uint32_t nowMs, uint32_t cycleMs) const {
  if (cycleMs == 0) return false;

  // Alone, or holding slot zero and nobody to take the clock from: free-run.
  // There is no schedule worth keeping to and nothing to collide with, and
  // waiting for a sync that will never arrive would mean never transmitting.
  if (known_ <= 1) return true;
  if (weAreReference()) return true;
  if (!haveEpoch_) return true;

  return inSlotAtPhase((uint32_t)(nowMs - epochMs_) % cycleMs, cycleMs);
}

} // namespace touge
