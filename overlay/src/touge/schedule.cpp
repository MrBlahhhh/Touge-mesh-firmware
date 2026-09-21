#include "schedule.h"

namespace touge {

void Schedule::reset() {
  selfId_ = 0;
  referenceId_ = 0;
  slot_ = SLOT_NONE;
  referenceSlot_ = 0;
  known_ = 0;
  epochMs_ = 0;
  haveEpoch_ = false;
}

void Schedule::rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders,
                       uint32_t nowMs) {
  selfId_ = selfId;

  // Who keeps time. A car whose own clock is locked to GPS always wins the
  // job, because cars with a fix follow the pulse and cars without follow the
  // reference's beacons; a free-running reference would put those two groups
  // on unrelated cycles. The node number only breaks the tie.
  uint32_t lowestLocked = selfLocked ? selfId : 0;
  bool anyLocked = selfLocked;
  uint32_t lowestAny = selfId;
  uint8_t count = 1; // ourselves

  // Who holds which slot, as advertised. A slot goes to the lowest node number
  // claiming it, which is the whole of the conflict resolution: two cars that
  // pick the same slot before hearing each other both apply the same rule to
  // the same facts and one of them moves.
  uint32_t owner[MAX_SLOTS];
  for (size_t i = 0; i < MAX_SLOTS; i++) owner[i] = 0;

  for (size_t i = 0; i < maxRiders; i++) {
    if (!riders[i].used) continue;
    // A rider whose id equals ours is a node number collision. Nothing here
    // can fix that, but it must not also corrupt the count or the slot table.
    if (riders[i].id == selfId) continue;
    count++;

    // A car that has gone quiet keeps its seat but loses the job of keeping
    // time.
    //
    // The roster holds a car for ten minutes, which is right for the map: a
    // car over a ridge should not vanish off it. It was also, until now, how
    // long a departed car went on winning the election for reference, because
    // the vote only ever looked at who was on the roster. So switching off the
    // lowest-numbered car left everyone timing off a radio that was in
    // somebody's pocket, with nothing re-disciplining the epoch - and an ESP32
    // crystal drifts far enough in ten minutes to slide one slot into the next.
    //
    // Two different questions, one constant. This is the other one: quiet for
    // a few beacons and you are still on the map, just not the clock.
    // Only the vote. A quiet car keeps its slot, because a slot it may still
    // be using must not be handed to somebody else on the strength of a few
    // missed beacons - that is how two cars end up transmitting together.
    const bool audible = (uint32_t)(nowMs - riders[i].atMs) < REFERENCE_LAPSE_MS;

    if (audible && riders[i].id < lowestAny) lowestAny = riders[i].id;
    if (audible && riders[i].pos.clockLocked && (!anyLocked || riders[i].id < lowestLocked)) {
      lowestLocked = riders[i].id;
      anyLocked = true;
    }

    uint8_t s = riders[i].pos.slot;
    if (s >= MAX_SLOTS) continue; // has not claimed one yet
    if (owner[s] == 0 || riders[i].id < owner[s]) owner[s] = riders[i].id;
  }

  referenceId_ = anyLocked ? lowestLocked : lowestAny;
  known_ = count;

  // Keep the slot we hold unless somebody with a better claim is on it. Slots
  // that survive a car joining are the entire point of claiming them: derived
  // from rank, every arrival shuffled everyone above it onto a new slot at
  // once, and a convoy that gains a car would spend a cycle colliding.
  bool mustMove = slot_ >= MAX_SLOTS;
  if (!mustMove && owner[slot_] != 0 && owner[slot_] < selfId) mustMove = true;

  if (mustMove) {
    slot_ = SLOT_NONE;
    for (uint8_t s = 0; s < MAX_SLOTS; s++) {
      if (owner[s] == 0) {
        slot_ = s;
        break;
      }
    }
    // More cars than slots. Doubling up costs those two a collision and keeps
    // everyone else on schedule, which beats falling off the end of the cycle
    // and never transmitting at all.
    if (slot_ >= MAX_SLOTS) slot_ = (uint8_t)(selfId % MAX_SLOTS);
  }

  // The reference advertises its own slot, so there is nothing to derive. It
  // is not necessarily slot zero: a GPS-locked car outranks a lower-numbered
  // one for the timekeeping job, and it holds whatever slot it claimed.
  referenceSlot_ = 0;
  if (referenceId_ == selfId) {
    referenceSlot_ = (slot_ < MAX_SLOTS) ? slot_ : 0;
  } else {
    for (size_t i = 0; i < maxRiders; i++) {
      if (!riders[i].used || riders[i].id != referenceId_) continue;
      if (riders[i].pos.slot < MAX_SLOTS) referenceSlot_ = riders[i].pos.slot;
      break;
    }
  }
}

void Schedule::syncTo(uint32_t heardAtMs, uint32_t cycleMs) {
  // Back out the reference's own slot to get the start of the cycle.
  epochMs_ = heardAtMs - (uint32_t)referenceSlot_ * slotWidthMs(cycleMs);
  haveEpoch_ = true;
}

void Schedule::startEpoch(uint32_t nowMs, uint32_t cycleMs) {
  if (cycleMs == 0 || haveEpoch_) return;
  // Put our own slot where it already is, so declaring the epoch does not move
  // this car's transmissions the moment it takes effect.
  const uint8_t s = (slot_ < MAX_SLOTS) ? slot_ : 0;
  epochMs_ = nowMs - (uint32_t)s * slotWidthMs(cycleMs);
  haveEpoch_ = true;
}

bool Schedule::inSlotAtPhase(uint32_t phaseMs, uint32_t cycleMs) const {
  if (cycleMs == 0) return false;
  // Alone on the channel there is nothing to collide with, so there is no
  // reason to sit out the other eight slots.
  if (known_ <= 1) return true;
  // No slot yet. A car has to be heard before it can be given one, and it has
  // to transmit to be heard, so an unclaimed car free-runs until its first
  // beacon has told everyone else it exists.
  if (slot_ >= MAX_SLOTS) return true;

  uint32_t width = slotWidthMs(cycleMs);
  if (width == 0) return true;

  uint32_t start = (uint32_t)slot_ * width;
  return phaseMs >= start && phaseMs < start + width;
}

bool Schedule::inSlot(uint32_t nowMs, uint32_t cycleMs) const {
  if (cycleMs == 0) return false;

  // Alone, or holding the clock and nobody to take it from: free-run. There is
  // no schedule worth keeping to, and waiting for a sync that will never
  // arrive would mean never transmitting.
  if (known_ <= 1) return true;
  if (!haveEpoch_) return true;
  // The timekeeper keeps to its slot like everybody else.
  //
  // This returned true for the reference unconditionally, on the reasoning
  // that the car defining the epoch cannot be out of step with it. True of a
  // PPS-disciplined board, which goes through inSlotAtPhase anyway. Not true
  // of a phone-tethered one: with no pulse it wins the vote on node number and
  // then transmits the moment it wants to, on top of whichever cars share its
  // slot - and with more cars than slots there are always some.
  //
  // It still free-runs before it has an epoch, which is the case above.
  return inSlotAtPhase((uint32_t)(nowMs - epochMs_) % cycleMs, cycleMs);
}


} // namespace touge
