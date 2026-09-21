#include "schedule.h"

namespace touge {

void Schedule::reset() {
  selfId_ = 0;
  referenceId_ = 0;
  slot_ = SLOT_NONE;
  parentId_ = 0;
  syncSlot_ = 0;
  refHops_ = REF_UNREACHABLE;
  referenceLocked_ = false;
  known_ = 0;
  epochMs_ = 0;
  haveEpoch_ = false;
}

namespace {

// Locked beats unlocked; among equals, the lowest node number wins.
//
// A car whose own cycle is disciplined by GPS always takes the job, because
// cars with a fix follow the pulse and cars without follow the reference's
// beacons, and a free-running reference would put those two groups on
// unrelated cycles.
bool betterReference(bool aLocked, uint32_t aId, bool bLocked, uint32_t bId) {
  if (aLocked != bLocked) return aLocked;
  return aId < bId;
}

} // namespace

void Schedule::rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders,
                       uint32_t nowMs) {
  selfId_ = selfId;

  // Who keeps time. A car whose own clock is locked to GPS always wins the
  // job, because cars with a fix follow the pulse and cars without follow the
  // reference's beacons; a free-running reference would put those two groups
  // on unrelated cycles. The node number only breaks the tie.
  // The best reference anyone knows of, not the best one we can hear.
  //
  // Locked beats unlocked, then the lowest node number breaks the tie - the
  // rule has not changed. What has changed is the candidate set: as well as
  // every car we can hear, it now includes every car those cars have told us
  // about. The head's claim reaches the tail through the cars between them,
  // one hop per beacon, so a convoy strung out past radio range converges on a
  // single timekeeper instead of quietly electing one per neighbourhood and
  // then hopping channel independently.
  uint32_t bestId = selfId;
  bool bestLocked = selfLocked;
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

    if (audible) {
      // The car itself.
      if (betterReference(riders[i].pos.clockLocked, riders[i].id, bestLocked, bestId)) {
        bestLocked = riders[i].pos.clockLocked;
        bestId = riders[i].id;
      }
      // And whatever it believes in, which may be a car we cannot hear.
      // Ignored past the hop cap: see MAX_REF_HOPS.
      if (riders[i].pos.refId != 0 && riders[i].pos.refHops < MAX_REF_HOPS &&
          betterReference(riders[i].pos.refLocked, riders[i].pos.refId, bestLocked, bestId)) {
        bestLocked = riders[i].pos.refLocked;
        bestId = riders[i].pos.refId;
      }
    }

    // A claim lapses well before the roster forgets the car.
    //
    // Keeping the seat and keeping the slot are different questions with
    // different answers, and both were being read off the same ten-minute
    // roster. Five cars leaving held five of nine slots long after they were
    // out of earshot, and the cars still on the road crowded into the rest.
    if ((uint32_t)(nowMs - riders[i].atMs) >= SLOT_LAPSE_MS) continue;

    uint8_t s = riders[i].pos.slot;
    if (s >= MAX_SLOTS) continue; // has not claimed one yet
    if (owner[s] == 0 || riders[i].id < owner[s]) owner[s] = riders[i].id;
  }

  referenceId_ = bestId;
  referenceLocked_ = bestLocked;
  known_ = count;

  // Keep the slot we hold unless somebody with a better claim is on it. Slots
  // that survive a car joining are the entire point of claiming them: derived
  // from rank, every arrival shuffled everyone above it onto a new slot at
  // once, and a convoy that gains a car would spend a cycle colliding.
  bool mustMove = slot_ >= MAX_SLOTS;
  if (!mustMove && owner[slot_] != 0 && owner[slot_] < selfId) mustMove = true;

  if (mustMove) {
    slot_ = SLOT_NONE;
    // Start the search at our own node number rather than at zero.
    //
    // With a populated table this is the same answer as scanning from zero:
    // the first free slot, just entered from a different point on the ring.
    // With an empty one it is the whole difference. Every board boots with a
    // roster it has not filled yet, every board found slot 0 free, and every
    // board claimed it - so twenty-eight cars switched on together all
    // transmitted in the same slot and then spent nine beacon rounds
    // unpicking it by node number, colliding on the low slots the entire
    // time. Entering the ring at selfId spreads that first guess across all
    // nine before anyone has heard anybody.
    const uint8_t start = (uint8_t)(selfId % MAX_SLOTS);
    for (uint8_t k = 0; k < MAX_SLOTS; k++) {
      const uint8_t s = (uint8_t)((start + k) % MAX_SLOTS);
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

  // Who we take the clock from, and how far that is from the reference.
  //
  // The reference itself when we can hear it, which is the case this has
  // always handled. Otherwise the neighbour with the shortest route to it:
  // that car's epoch is already the reference's, so pinning to its beacon puts
  // us on the same cycle as a car we have never heard. Its slot is subtracted
  // rather than the reference's, because its slot is what its beacon landed
  // in.
  parentId_ = 0;
  syncSlot_ = 0;
  refHops_ = REF_UNREACHABLE;

  if (referenceId_ == selfId) {
    // We are the anchor. Nothing to sync to, and zero hops from ourselves.
    refHops_ = 0;
    syncSlot_ = (slot_ < MAX_SLOTS) ? slot_ : 0;
  } else {
    uint8_t bestVia = REF_UNREACHABLE;
    for (size_t i = 0; i < maxRiders; i++) {
      if (!riders[i].used) continue;
      if (riders[i].id == selfId) continue;
      if ((uint32_t)(nowMs - riders[i].atMs) >= REFERENCE_LAPSE_MS) continue;

      uint8_t via;
      if (riders[i].id == referenceId_) {
        via = 0; // it is the reference; we are one hop from it
      } else if (riders[i].pos.refId == referenceId_ && riders[i].pos.refHops < MAX_REF_HOPS) {
        via = riders[i].pos.refHops;
      } else {
        continue; // knows nothing useful about where the reference is
      }

      // Nearest first, lowest node number to break a tie so that two cars at
      // equal distance do not oscillate between parents beacon by beacon.
      if (via < bestVia || (via == bestVia && riders[i].id < parentId_)) {
        bestVia = via;
        parentId_ = riders[i].id;
        syncSlot_ = (riders[i].pos.slot < MAX_SLOTS) ? riders[i].pos.slot : 0;
      }
    }
    if (parentId_ != 0) refHops_ = (uint8_t)(bestVia + 1);
  }
}

void Schedule::syncTo(uint32_t heardAtMs, uint32_t cycleMs, uint32_t senderLateMs) {
  // Back out the sender's own slot to get the start of the cycle, and the time
  // it is expected to have spent waiting for a tick inside that slot.
  epochMs_ = heardAtMs - senderLateMs - (uint32_t)syncSlot_ * slotWidthMs(cycleMs);
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
