#include "schedule.h"

namespace touge {

uint32_t slotStartMs(uint8_t slot) {
  const uint32_t block = slot % BLOCKS;
  const uint32_t inBlock = slot / BLOCKS;
  return block * BLOCK_MS + inBlock * SLOT_MS;
}

bool leaseOlder(uint16_t a, uint16_t b) { return (int16_t)(uint16_t)(a - b) < 0; }

uint8_t slotTag(uint32_t id) {
  const uint8_t tag = (uint8_t)((id * 2654435761u) >> 24);
  return tag != 0 ? tag : 1;
}

namespace {

// Locked beats unlocked; among equals, the lowest node number wins. A locked
// car has to keep time, or the cars following their own pulse and the cars
// following the reference's beacons end up on unrelated clocks.
bool betterReference(bool aLocked, uint32_t aId, bool bLocked, uint32_t bId) {
  if (aLocked != bLocked) return aLocked;
  return aId < bId;
}

// Which of two claims on one slot keeps it: the older lease, then the lower
// node number.
bool claimBeats(uint16_t aGen, uint32_t aId, uint16_t bGen, uint32_t bId) {
  if (aGen != bGen) return leaseOlder(aGen, bGen);
  return aId < bId;
}

bool liveLease(const Rider& r, uint32_t selfId, uint32_t nowMs) {
  return r.used && r.id != selfId && (uint32_t)(nowMs - r.atMs) < LEASE_MS;
}

} // namespace

void Schedule::reset() { *this = Schedule(); }

void Schedule::rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders,
                       uint32_t nowMs) {
  selfId_ = selfId;
  // Lease first: only a leased car may keep time, and a car that claims its
  // slot in this pass should be able to win the job in the same pass.
  settleLease(riders, maxRiders, nowMs);
  electReference(selfLocked, riders, maxRiders, nowMs);
  chooseParent(riders, maxRiders, nowMs);
  planExtras(riders, maxRiders, nowMs);
}

void Schedule::electReference(bool selfLocked, const Rider* riders, size_t maxRiders,
                              uint32_t nowMs) {
  // The best reference anyone knows of, not only the best one we can hear. The
  // head's claim reaches the tail one hop per beacon through the cars between,
  // so a convoy past radio range still converges on one timekeeper.
  //
  // Only a car that holds a lease, or keeps GPS time, can do the job: the
  // others sync to the reference's beacon, and a beacon from the shared window
  // marks no point in the second. Without this the lowest node number kept the
  // job through a reboot while it listened for its lease, nobody could sync to
  // it, and the rest synced to each other in loops. When nobody qualifies (a
  // car park switching on) the old rule applies to everyone.
  // Neighbours go on naming a reference that has rebooted until they hear it
  // themselves, so a belief naming a car we can hear to be unleased, or us
  // while we are unleased, does not count either.
  auto unfit = [&](uint32_t id) {
    if (id == selfId_) return !selfLocked && !claimed();
    for (size_t i = 0; i < maxRiders; i++) {
      const Rider& r = riders[i];
      if (r.used && r.id == id && (uint32_t)(nowMs - r.atMs) < REFERENCE_LAPSE_MS)
        return !r.pos.clockLocked && r.pos.slot >= MAX_SLOTS;
    }
    return false;
  };

  for (int pass = 0; pass < 2; pass++) {
    const bool leasedOnly = pass == 0;
    bool found = !leasedOnly || selfLocked || claimed();
    uint32_t bestId = selfId_;
    bool bestLocked = selfLocked;

    for (size_t i = 0; i < maxRiders; i++) {
      const Rider& r = riders[i];
      // An id equal to ours is a node number collision; it must not vote.
      if (!r.used || r.id == selfId_) continue;
      // A car quiet for a few beacons keeps its lease but loses the vote, so
      // switching the reference off does not leave the ride timing off a
      // radio in somebody's pocket for the roster's ten minutes.
      if ((uint32_t)(nowMs - r.atMs) >= REFERENCE_LAPSE_MS) continue;
      // An unleased car's belief is only what the leased cars around it
      // already say, or itself.
      if (leasedOnly && !r.pos.clockLocked && r.pos.slot >= MAX_SLOTS) continue;

      if (!found || betterReference(r.pos.clockLocked, r.id, bestLocked, bestId)) {
        bestLocked = r.pos.clockLocked;
        bestId = r.id;
        found = true;
      }
      // Past the hop cap a route is not believed; see MAX_REF_HOPS.
      if (r.pos.refId != 0 && r.pos.refHops < MAX_REF_HOPS &&
          !(leasedOnly && unfit(r.pos.refId)) &&
          betterReference(r.pos.refLocked, r.pos.refId, bestLocked, bestId)) {
        bestLocked = r.pos.refLocked;
        bestId = r.pos.refId;
      }
    }

    if (!found) continue;
    referenceId_ = bestId;
    referenceLocked_ = bestLocked;
    return;
  }
}

void Schedule::settleLease(const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  // Who holds each slot among the cars we can hear, directly or relayed, and
  // the newest generation any of them has heard of.
  bool held[MAX_SLOTS] = {};
  uint32_t holder[MAX_SLOTS] = {};
  uint16_t holderGen[MAX_SLOTS] = {};
  uint8_t live = 0;

  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs)) continue;
    live++;
    if (!genKnown_ || leaseOlder(schedGen_, r.pos.schedGen)) schedGen_ = r.pos.schedGen;
    genKnown_ = true;

    const uint8_t s = r.pos.slot;
    if (s >= MAX_SLOTS) continue;
    if (!held[s] || claimBeats(r.pos.leaseGen, r.id, holderGen[s], holder[s])) {
      held[s] = true;
      holder[s] = r.id;
      holderGen[s] = r.pos.leaseGen;
    }
  }
  known_ = (uint8_t)(1 + live);

  someoneWaiting_ = false;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs)) continue;
    if (r.pos.slot >= MAX_SLOTS || holder[r.pos.slot] != r.id) someoneWaiting_ = true;
  }

  // Nobody heard for a whole lease. Everyone else has let our slot go by now,
  // so we let it go too and rejoin as a newcomer when the ride comes back,
  // rather than walking back in and taking a slot that has been reissued.
  if (live == 0) {
    slot_ = SLOT_NONE;
    heardAnyone_ = false;
    return;
  }
  if (!heardAnyone_) {
    heardAnyone_ = true;
    firstHeardMs_ = nowMs;
  }

  // What the cars we hear directly report hearing in each slot. A slot any of
  // them hears is busy even if its holder is out of our own range. And our own
  // slot, as they report it, is the only way to learn that somebody we cannot
  // hear is transmitting on top of us.
  const uint8_t ourTag = slotTag(selfId_);
  uint32_t reportedBusy = 0;
  uint8_t judges = 0;
  uint8_t hearUs = 0;
  uint8_t hearOther = 0;
  uint8_t otherTag = 0;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs) || r.hopsAway != 0) continue;
    if ((uint32_t)(nowMs - r.atMs) >= HEARD_WINDOW_MS) continue;
    for (uint8_t s = 0; s < MAX_SLOTS; s++)
      if (r.pos.slotMap[s] != 0) reportedBusy |= 1u << s;
    // Only a map covering a whole window since we took the slot can say
    // whether our beacons landed.
    if (!claimed() || (int32_t)(r.atMs - leasedAtMs_) < (int32_t)HEARD_WINDOW_MS) continue;
    judges++;
    const uint8_t tag = r.pos.slotMap[slot_];
    if (tag == ourTag) {
      hearUs++;
    } else if (tag != 0) {
      hearOther++;
      if (otherTag == 0 || tag < otherTag) otherTag = tag;
    }
  }

  if (claimed()) {
    const bool outranked =
        held[slot_] && !claimBeats(leaseGen_, selfId_, holderGen[slot_], holder[slot_]);
    // Somebody else on our slot gets through to more of our neighbours than we
    // do; on a tie the higher tag yields, so exactly one of a pair moves. Or
    // nobody gets through at all, which is what three cars on one slot look
    // like. Two judges at least, so one neighbour's lost frame moves nobody.
    const bool clash = hearOther > hearUs ||
                       (hearOther > 0 && hearOther == hearUs && otherTag < ourTag) ||
                       (judges >= 2 && hearUs == 0 && hearOther == 0);
    if (clash && !clashing_) clashSinceMs_ = nowMs;
    clashing_ = clash;
    const bool established = (uint32_t)(nowMs - leasedAtMs_) >= ESTABLISHED_LEASE_MS;
    const bool drowned =
        clash && (!established || (uint32_t)(nowMs - clashSinceMs_) >= CLASH_PATIENCE_MS);
    if (!outranked && !drowned) return;
    clashing_ = false;
    if (drowned) {
      // Give the slot up and listen again as a joiner. Our unleased beacons
      // stop everybody's extras and show the cars we clashed with that we
      // need a slot, so the queue ranks us instead of both of us picking the
      // same free slot again. Moving straight to another slot landed in some
      // car's extras, where the clash hid again.
      slot_ = SLOT_NONE;
      firstHeardMs_ = nowMs;
      return;
    }
    // Outranked: the cars that outrank us already know we need a slot, so
    // move now without listening again.
  } else if ((uint32_t)(nowMs - firstHeardMs_) < JOIN_LISTEN_MS) {
    return; // still learning who holds what
  }
  slot_ = SLOT_NONE;

  // Free slots in lease order: block 0 first, row by row, then block 1, 2, 3,
  // so the first eight cars each get a row of their own for extras.
  uint8_t freeSlots[MAX_SLOTS];
  uint8_t freeCount = 0;
  for (uint8_t block = 0; block < BLOCKS; block++) {
    for (uint8_t row = 0; row < SLOTS_PER_BLOCK; row++) {
      const uint8_t s = (uint8_t)(row * BLOCKS + block);
      if (!held[s] && !(reportedBusy & (1u << s))) freeSlots[freeCount++] = s;
    }
  }

  // Cars ahead of us in the queue: unleased, or on a slot somebody else has
  // won, and with a lower node number. We take the free slot after theirs, so
  // everyone with the same roster lands on a different slot without first
  // colliding.
  uint8_t pick = 0;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs)) continue;
    const bool needsSlot = r.pos.slot >= MAX_SLOTS || holder[r.pos.slot] != r.id;
    if (needsSlot && r.id < selfId_) pick++;
  }
  // No free slot for us. Never double up on a held one: stay unleased and
  // transmit in the shared window.
  if (pick >= freeCount) return;

  slot_ = freeSlots[pick];
  leasedAtMs_ = nowMs;
  // One past anything we have heard of, so every lease we could clash with
  // is older than ours and keeps its slot.
  leaseGen_ = (uint16_t)(schedGen_ + 1);
  schedGen_ = leaseGen_;
  genKnown_ = true;
}

void Schedule::chooseParent(const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  parentId_ = 0;
  refHops_ = REF_UNREACHABLE;

  leasedInEarshot_ = false;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (r.used && r.id != selfId_ && r.hopsAway == 0 && r.pos.slot < MAX_SLOTS &&
        (uint32_t)(nowMs - r.atMs) < REFERENCE_LAPSE_MS)
      leasedInEarshot_ = true;
  }

  if (referenceId_ == selfId_) {
    refHops_ = 0;
    return;
  }

  // The reference if we hear it; otherwise the neighbour with the shortest
  // route to it, whose epoch is already the reference's.
  uint8_t bestVia = REF_UNREACHABLE;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!r.used || r.id == selfId_) continue;
    if ((uint32_t)(nowMs - r.atMs) >= REFERENCE_LAPSE_MS) continue;
    // Its beacons only mark time if they come straight from it and we know
    // which slot they were sent in. A relayed copy carries the relay's jitter,
    // and the module only syncs to direct copies anyway.
    if (r.hopsAway != 0 || r.pos.slot >= MAX_SLOTS) continue;

    uint8_t via;
    if (r.id == referenceId_) {
      via = 0;
    } else if (r.pos.refId == referenceId_ && r.pos.refHops < MAX_REF_HOPS) {
      via = r.pos.refHops;
    } else {
      continue;
    }

    // Lowest node number breaks a tie, so equal routes do not flap.
    if (via < bestVia || (via == bestVia && r.id < parentId_)) {
      bestVia = via;
      parentId_ = r.id;
    }
  }
  if (parentId_ != 0) refHops_ = (uint8_t)(bestVia + 1);
}

void Schedule::planExtras(const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  extraMask_ = 0;
  // A joiner or a car that lost its slot is about to claim one of the free
  // slots, and it has to find it silent. Nothing to gain alone, and nothing
  // safe to add while our own slot looks clashed.
  if (!claimed() || known_ <= 1 || someoneWaiting_ || clashing_) return;
  // A full roster may be missing a car, whose lease an extra could land on.
  if (known_ - 1 >= MAX_RIDERS) return;

  const uint8_t row = (uint8_t)(slot_ / BLOCKS);
  const uint8_t ourBlock = (uint8_t)(slot_ % BLOCKS);
  // Blocks of our row leased by anyone: cars we hear, directly or relayed,
  // and cars only our neighbours hear, from their slot maps.
  uint8_t leased = (uint8_t)(1u << ourBlock);
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs)) continue;
    if (r.pos.slot < MAX_SLOTS && r.pos.slot / BLOCKS == row) leased |= (uint8_t)(1u << (r.pos.slot % BLOCKS));
    if (r.hopsAway != 0 || (uint32_t)(nowMs - r.atMs) >= HEARD_WINDOW_MS) continue;
    for (uint8_t b = 0; b < BLOCKS; b++)
      if (r.pos.slotMap[row * BLOCKS + b] != 0) leased |= (uint8_t)(1u << b);
  }

  for (uint8_t free = 0; free < BLOCKS; free++) {
    if (leased & (1u << free)) continue;
    // Opposite first: with two leases in adjacent blocks each gets the block
    // 500 ms after its own, so both beacon evenly at 2 Hz.
    uint8_t owner = (uint8_t)((free + 2) % BLOCKS);
    if (!(leased & (1u << owner))) {
      for (uint8_t back = 1; back < BLOCKS; back++) {
        owner = (uint8_t)((free + BLOCKS - back) % BLOCKS);
        if (leased & (1u << owner)) break;
      }
    }
    if (owner == ourBlock) extraMask_ |= 1u << (row * BLOCKS + free);
  }
}

bool Schedule::inExtraSlotAtPhase(uint32_t phaseMs) const {
  phaseMs %= SCHEDULE_MS;
  for (uint8_t s = 0; s < MAX_SLOTS; s++) {
    if (!(extraMask_ & (1u << s))) continue;
    const uint32_t start = slotStartMs(s);
    if (phaseMs >= start && phaseMs + SLOT_GUARD_MS <= start + SLOT_MS) return true;
  }
  return false;
}

bool Schedule::inExtraSlot(uint32_t nowMs) const {
  if (!haveEpoch_ || extraMask_ == 0) return false;
  return inExtraSlotAtPhase((uint32_t)(nowMs - epochMs_) % SCHEDULE_MS);
}

void Schedule::heardSlot(uint8_t slot, uint32_t senderId, uint32_t nowMs) {
  if (slot >= MAX_SLOTS) return;
  slotHeardMs_[slot] = nowMs;
  slotHeardTag_[slot] = slotTag(senderId);
}

void Schedule::fillSlotMap(uint32_t nowMs, uint8_t map[SLOT_MAP_LEN]) const {
  for (uint8_t s = 0; s < MAX_SLOTS; s++) {
    const bool fresh = (uint32_t)(nowMs - slotHeardMs_[s]) < HEARD_WINDOW_MS;
    map[s] = fresh ? slotHeardTag_[s] : 0;
  }
}

bool Schedule::takesClockFrom(uint32_t src) const {
  // Without an epoch any leased beacon is better than free-running on top of
  // somebody's slot, which is what a rebooted car did for as long as it had
  // no usable parent. syncTo ignores the unleased ones.
  if (!haveEpoch_) return true;
  return src == (parentId_ != 0 ? parentId_ : referenceId_);
}

void Schedule::syncTo(uint32_t heardAtMs, uint8_t senderSlot, uint32_t senderLateMs) {
  // A beacon sent in the shared window marks no particular point in the
  // second, so it cannot set the clock.
  if (senderSlot >= MAX_SLOTS) return;
  epochMs_ = heardAtMs - senderLateMs - slotStartMs(senderSlot);
  haveEpoch_ = true;
}

void Schedule::startEpoch(uint32_t nowMs) {
  // Alone the epoch means nothing, and a declared one would stop us taking the
  // ride's when it appears. With a leased car in earshot, its next beacon will
  // give us the ride's epoch; declaring our own would drag the ride onto it.
  if (haveEpoch_ || known_ <= 1 || leasedInEarshot_) return;
  // Put our own slot where we are now, so declaring the epoch does not move
  // this car's next transmission.
  epochMs_ = nowMs - (claimed() ? slotStartMs(slot_) : 0);
  haveEpoch_ = true;
}

bool Schedule::inSlotAtPhase(uint32_t phaseMs) const {
  // Alone there is nothing to collide with.
  if (known_ <= 1) return true;
  phaseMs %= SCHEDULE_MS;

  if (claimed()) {
    const uint32_t start = slotStartMs(slot_);
    return phaseMs >= start && phaseMs + SLOT_GUARD_MS <= start + SLOT_MS;
  }
  if (phaseMs / BLOCK_MS != sharedBlock_) return false;
  const uint32_t inBlock = phaseMs % BLOCK_MS;
  return inBlock >= SHARED_OFFSET_MS + sharedOffsetMs_ && inBlock + SLOT_GUARD_MS <= BLOCK_MS;
}

void Schedule::drawSharedTurn(uint32_t random) {
  sharedBlock_ = (uint8_t)(random % BLOCKS);
  sharedOffsetMs_ = (random / BLOCKS) % SHARED_SPREAD_MS;
}

bool Schedule::inSlot(uint32_t nowMs) const {
  if (known_ <= 1) return true;
  if (!haveEpoch_) return true;
  return inSlotAtPhase((uint32_t)(nowMs - epochMs_) % SCHEDULE_MS);
}

} // namespace touge
