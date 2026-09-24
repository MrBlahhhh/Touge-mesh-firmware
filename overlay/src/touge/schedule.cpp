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

// A car in the running to keep time, by its own advertised flags.
struct Candidate {
  bool locked;
  bool fit;
  uint32_t id;
};

// Locked beats unlocked: a locked car has to keep time, or the cars following
// their own pulse and the cars following the reference's beacons end up on
// unrelated clocks. Then a car that hears the ride well, then the lowest number.
bool betterReference(const Candidate& a, const Candidate& b) {
  if (a.locked != b.locked) return a.locked;
  if (a.fit != b.fit) return a.fit;
  return a.id < b.id;
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

// A slot's beacon record lined up with now, `sinceMs` after its last beacon:
// one beacon counts missing only once it is a quarter second overdue, so a
// read just before it lands does not see a gap.
uint8_t upToNow(uint8_t bits, uint32_t sinceMs) {
  const uint32_t graceMs = SCHEDULE_MS / 4;
  if (sinceMs <= SCHEDULE_MS + graceMs) return bits;
  const uint32_t missed = (sinceMs - graceMs) / SCHEDULE_MS;
  return missed >= LINK_SECONDS ? 0 : (uint8_t)(bits << missed);
}

// Seconds the record covers, from its oldest beacon to now.
uint8_t spanOf(uint8_t bits) { return bits == 0 ? 0 : (uint8_t)(32 - __builtin_clz((unsigned)bits)); }

// Under three in four heard, over at least two seconds of record.
bool poorRecord(uint8_t bits) {
  const uint8_t span = spanOf(bits);
  return span >= 2 && __builtin_popcount(bits) * 4 < span * 3;
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
  updateFitness(nowMs);

  // The best reference anyone knows of, not only the best one we can hear. The
  // head's claim reaches the tail one hop per beacon through the cars between,
  // so a convoy past radio range still converges on one timekeeper.
  //
  // Only a car that holds a lease, or keeps GPS time, can do the job: the
  // others sync to the reference's beacon, and a beacon from the shared window
  // marks no point in the second. Without this the lowest node number kept the
  // job through a reboot while it listened for its lease, nobody could sync to
  // it, and the rest synced to each other in loops. When nobody qualifies (a
  // car park switching on) every car stands.
  //
  // A neighbour's word about a car we hear ourselves (or about us) is older
  // news than that car's own beacon, so only the beacon counts. That also
  // keeps out a rebooted reference that neighbours still name while it listens.
  auto current = [&](uint32_t id) {
    if (id == selfId_) return true;
    for (size_t i = 0; i < maxRiders; i++) {
      const Rider& r = riders[i];
      if (r.used && r.id == id && (uint32_t)(nowMs - r.atMs) < REFERENCE_LAPSE_MS) return true;
    }
    return false;
  };

  for (int pass = 0; pass < 2; pass++) {
    const bool leasedOnly = pass == 0;
    bool found = !leasedOnly || selfLocked || claimed();
    Candidate best{selfLocked, announcedFit_, selfId_};

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

      const Candidate itself{r.pos.clockLocked, r.pos.fitToKeepTime, r.id};
      if (!found || betterReference(itself, best)) {
        best = itself;
        found = true;
      }
      // Past the hop cap a route is not believed; see MAX_REF_HOPS.
      if (r.pos.refId == 0 || r.pos.refHops >= MAX_REF_HOPS || current(r.pos.refId)) continue;
      const Candidate belief{r.pos.refLocked, r.pos.refFit, r.pos.refId};
      if (betterReference(belief, best)) best = belief;
    }

    if (!found) continue;
    referenceId_ = best.id;
    referenceLocked_ = best.locked;
    referenceFit_ = best.fit;
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
  const uint8_t ourTag = slotTag(selfId_);

  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs)) continue;
    live++;
    if (!genKnown_ || leaseOlder(schedGen_, r.pos.schedGen)) schedGen_ = r.pos.schedGen;
    genKnown_ = true;
    // Any map showing us on our slot, relayed or not, is somebody hearing us.
    if (claimed() && r.pos.slotMap[slot_] == ourTag && (int32_t)(r.atMs - lastHeardUsMs_) > 0)
      lastHeardUsMs_ = r.atMs;

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
    listenMs_ = JOIN_LISTEN_MS;
  }

  // A car that hears the ride poorly has older maps from its neighbours and
  // may not have caught every car yet, so it claims on maps up to
  // MAP_TRUST_MS old, and waits while a leased neighbour's is older still.
  const bool poor = hearingPoorly(nowMs);
  const uint32_t mapTrustMs = poor ? MAP_TRUST_MS : HEARD_WINDOW_MS;
  bool mapOverdue = false;

  // What the cars we hear directly report hearing in each slot. A slot any of
  // them hears is busy even if its holder is out of our own range. And our own
  // slot, as they report it, is the only way to learn that somebody we cannot
  // hear is transmitting on top of us.
  uint32_t reportedBusy = 0;
  uint8_t wellHeardJudges = 0;
  uint8_t hearUs = 0;
  uint8_t hearOther = 0;
  uint8_t otherTag = 0;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& r = riders[i];
    if (!liveLease(r, selfId_, nowMs) || r.hopsAway != 0) continue;
    const uint32_t ageMs = nowMs - r.atMs;
    if (ageMs >= mapTrustMs) {
      if (poor && r.pos.slot < MAX_SLOTS && ageMs < REFERENCE_LAPSE_MS) mapOverdue = true;
      continue;
    }
    for (uint8_t s = 0; s < MAX_SLOTS; s++)
      if (r.pos.slotMap[s] != 0) reportedBusy |= 1u << s;
    if (ageMs >= HEARD_WINDOW_MS) continue;
    // Only a map covering a whole window since we took the slot can say
    // whether our beacons landed; the neighbours with one judge our slot.
    if (!claimed() || (int32_t)(r.atMs - leasedAtMs_) < (int32_t)HEARD_WINDOW_MS) continue;
    if (!hearsPoorly(r.pos.slot, r.id, nowMs)) wellHeardJudges++;
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
    // like: two judges at least, so one neighbour's lost frame moves nobody,
    // and ones we hear well, since a car we hear poorly misses us as often.
    const bool clash = hearOther > hearUs ||
                       (hearOther > 0 && hearOther == hearUs && otherTag < ourTag) ||
                       (wellHeardJudges >= 2 && hearUs == 0 && hearOther == 0);
    if (clash && !clashing_) clashSinceMs_ = nowMs;
    clashing_ = clash;
    const bool established = (uint32_t)(nowMs - leasedAtMs_) >= ESTABLISHED_LEASE_MS;
    const bool drowned =
        clash && (!established || (uint32_t)(nowMs - clashSinceMs_) >= CLASH_PATIENCE_MS);
    const bool unheard = (uint32_t)(nowMs - lastHeardUsMs_) >= UNHEARD_MS;
    if (!outranked && !drowned && !unheard) {
      // Settled and heard: whatever went wrong before is behind us.
      if (established && !clash && (int32_t)(lastHeardUsMs_ - leasedAtMs_) > 0) strikes_ = 0;
      return;
    }
    clashing_ = false;
    slot_ = SLOT_NONE;
    if (strikes_ < 255) strikes_++;
    // Outranked, with a good view: the cars that outrank us already know we
    // need a slot, so move now without listening again. That is what settles
    // a merge, however many times it takes: listening instead stops everyone's
    // extras, and the extras are how the other half of a hidden clash learns
    // it has one (host sim: a merge went from 5 s to over 10). A poor listener
    // most likely landed there for want of a view, so it listens again, as
    // does any car on a drowned or unheard slot: our unleased beacons show
    // the cars we clashed with that we need a slot, so the queue ranks us
    // instead of both of us picking the same free slot again. Moving straight
    // to another slot landed in some car's extras, where the clash hid again.
    if (drowned || unheard || poor) {
      listenAgain(nowMs);
      return;
    }
  } else if ((uint32_t)(nowMs - firstHeardMs_) < listenMs_ + (poor ? JOIN_LISTEN_MS : 0) ||
             mapOverdue) {
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
  lastHeardUsMs_ = nowMs;
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

void Schedule::heardBeacon(uint32_t senderId, const Position& p, uint32_t nowMs) {
  const uint8_t s = p.slot;
  if (s >= MAX_SLOTS) return;
  const uint8_t tag = slotTag(senderId);
  // Whole beacons since the last one in this slot, so the gaps are misses.
  const uint32_t beacons = (nowMs - slotHeardMs_[s] + SCHEDULE_MS / 2) / SCHEDULE_MS;
  if (slotHeardTag_[s] != tag || beacons >= LINK_SECONDS) {
    slotSeen_[s] = 0;
    slotMutual_[s] = 0;
  } else {
    slotSeen_[s] = (uint8_t)(slotSeen_[s] << beacons);
    slotMutual_[s] = (uint8_t)(slotMutual_[s] << beacons);
  }
  slotSeen_[s] |= 1;
  if (claimed() && p.slotMap[slot_] == slotTag(selfId_)) slotMutual_[s] |= 1;
  slotHeardMs_[s] = nowMs;
  slotHeardTag_[s] = tag;
}

void Schedule::updateFitness(uint32_t nowMs) {
  uint8_t heard = 0;
  uint8_t solid = 0;
  for (uint8_t s = 0; s < MAX_SLOTS; s++) {
    const uint32_t sinceMs = nowMs - slotHeardMs_[s];
    // A car heard at least twice in the window. One that has just left still
    // counts for a few seconds, against us, which errs the safe way.
    if (__builtin_popcount(upToNow(slotSeen_[s], sinceMs)) < 2) continue;
    heard++;
    if (__builtin_popcount(upToNow(slotMutual_[s], sinceMs)) >= SOLID_LINK_SECONDS) solid++;
  }
  // Half of them solid to become fit, a third to stay fit.
  const bool shouldBeFit = solid > 0 && solid * (fit_ ? 3 : 2) >= heard;
  if (shouldBeFit == fit_) {
    fitSettledMs_ = nowMs;
    return;
  }
  // Either way only after FIT_HOLD_MS on end. A radio heard one frame in two
  // has runs where a few links look solid at once, and must not win on one.
  // And when two groups meet, cars on the same slot numbers reset each other's
  // records for a few seconds; a group that lost fitness on that alone handed
  // the clock to the other half at once, before the hidden clashes had been
  // found (host sim: 912 leased-slot collisions against 96).
  if ((uint32_t)(nowMs - fitSettledMs_) < FIT_HOLD_MS) return;
  fit_ = shouldBeFit;
  fitSettledMs_ = nowMs;
}

bool Schedule::hearingPoorly(uint32_t nowMs) const {
  uint32_t heard = 0;
  uint32_t due = 0;
  for (uint8_t s = 0; s < MAX_SLOTS; s++) {
    const uint8_t seen = upToNow(slotSeen_[s], nowMs - slotHeardMs_[s]);
    // One beacon says nothing about the ones in between, and a slot silent for
    // three seconds has been left, which says nothing about our hearing.
    if (spanOf(seen) < 2 || (seen & 0x07) == 0) continue;
    heard += (uint32_t)__builtin_popcount(seen);
    due += spanOf(seen);
  }
  return heard * 4 < due * 3;
}

bool Schedule::hearsPoorly(uint8_t slot, uint32_t id, uint32_t nowMs) const {
  if (slot >= MAX_SLOTS || slotHeardTag_[slot] != slotTag(id)) return false;
  return poorRecord(upToNow(slotSeen_[slot], nowMs - slotHeardMs_[slot]));
}

void Schedule::listenAgain(uint32_t nowMs) {
  firstHeardMs_ = nowMs;
  uint32_t listenMs = JOIN_LISTEN_MS;
  // The first loss is the ordinary newcomer's yield. From the second before a
  // lease settles, a random extra of up to a second per loss so far.
  if (strikes_ > 1) {
    uint32_t spreadMs = (uint32_t)(strikes_ - 1) * SCHEDULE_MS;
    if (spreadMs > BACKOFF_MAX_MS) spreadMs = BACKOFF_MAX_MS;
    listenMs += spareRandom_ % spreadMs;
  }
  listenMs_ = (uint16_t)listenMs;
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
  spareRandom_ = (uint16_t)(random >> 16);
}

bool Schedule::inSlot(uint32_t nowMs) const {
  if (known_ <= 1) return true;
  if (!haveEpoch_) return true;
  return inSlotAtPhase((uint32_t)(nowMs - epochMs_) % SCHEDULE_MS);
}

} // namespace touge
