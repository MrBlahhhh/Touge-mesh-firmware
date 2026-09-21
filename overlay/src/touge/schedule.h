#pragma once
//
// Who transmits when.
//
// Random jitter stops cars colliding by accident. It does not stop them
// colliding on purpose, which is what four cars beaconing at 4 Hz on one
// channel amounts to: the collisions get more likely with every car added, and
// the ones that matter most are the beacons, because they are the only traffic
// that is both periodic and constant.
//
// So beacons get slots. Every car works out the same ordering from the same
// roster and transmits only in its own, which means there is nothing to
// contend with rather than a polite scheme for contending.
//
// Cardo's DMC does this (US10277748) with a leader election and a designated
// synchronizer. This is the same idea with the negotiation taken out: the
// lowest node number on the ride is the reference by definition, so there is
// nothing to elect and nothing to agree on beyond the roster itself.
//
// ## What this is not
//
// It is not real TDMA. ESP-NOW sits on 802.11, whose MAC does its own carrier
// sense and backoff underneath and cannot be turned off, and the clock is
// recovered from received beacons rather than from GPS, which is good to a
// couple of milliseconds and no better. Against a 27 ms slot that is plenty.
// What this buys is the removal of self-collision, which is the loss that
// grows with the size of the group. It does not make the channel exclusive.

#include <stdint.h>
#include <stddef.h>
#include "mesh.h"

namespace touge {

// Riders plus ourselves. Fixed rather than sized to the roster, because a slot
// width that changed as cars joined would move everybody else at once, and the
// moment a roster is in flux is exactly when you want the schedule stable.
// Transmit slots in one cycle. An airtime budget, not a headcount.
//
// This was MAX_RIDERS + 1, on the assumption that every car gets its own slot.
// That assumption breaks the moment a ride is bigger than a bench group:
// twenty-nine slots in a 250 ms cycle is 8.6 ms each, and an ESP-NOW LR frame
// at 250 kbps is already about 8 ms. There would be no room for the frame,
// never mind guard time.
//
// So the two are now separate. The slot count comes from what the air can
// carry; the roster comes from how many cars are on the ride. With more cars
// than slots, the cars without one free-run - which is what `inSlotAtPhase`
// already does for an unclaimed car, and degrades to carrier sense rather
// than to silence.
//
// The real answer is spatial reuse: a slot only has to be unique within a
// car's two-hop interference neighbourhood, so the front and the tail of a
// half-mile train can share one. That needs the neighbour gossip and two-hop
// colouring described in firmware/docs/REDESIGN-30-CARS.md. Until then this
// is an honest nine.
//
// Fifteen is the wire's ceiling regardless: the slot travels in a nibble
// alongside the flags (frame.cpp packs `(slot & 0x0F) << 4`), so anything
// above fifteen needs a frame format change as well.
static const uint8_t MAX_SLOTS = 9;

// The wire carries the slot in a nibble beside the flags, so fifteen is the
// ceiling until the frame format changes. Worth failing the build over rather
// than discovering it as slots silently wrapping into the flag bits.
static_assert(MAX_SLOTS <= 15, "the slot travels in a nibble; see frame.cpp");

// How long a car may be quiet and still keep the job of keeping time.
//
// Eight beacons at the idle heartbeat: long enough that a handful of losses
// change nothing, short enough that
// switching off the reference does not leave everyone timing off a radio in
// somebody's pocket. The roster's own ten minutes is right for the map and far
// too long for the clock.
static const uint32_t REFERENCE_LAPSE_MS = 8000;

// No slot held yet. Nine slots leave four bits with values to spare, so this
// rides in the same nibble as the slot itself and costs nothing.
static const uint8_t SLOT_NONE = 0x0F;

class Schedule {
 public:
  void reset();

  // Claim a slot, and keep it.
  //
  // Slots used to be derived from rank among the known node numbers, which
  // needed no protocol at all but meant a car joining pushed everyone above it
  // onto a new slot at the same moment. Now each car advertises the slot it
  // holds, keeps it unless a lower node number is already on it, and otherwise
  // takes the lowest free one. Two cars that pick the same slot before hearing
  // each other apply the same rule to the same facts, so one of them moves.
  /**
   * @param nowMs so a car that has gone quiet can stop being the reference.
   */
  void rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders,
               uint32_t nowMs);

  // SLOT_NONE until the first roster arrives.
  uint8_t slot() const { return slot_; }
  bool claimed() const { return slot_ < MAX_SLOTS; }
  uint8_t known() const { return known_; }
  uint32_t referenceId() const { return referenceId_; }
  uint8_t referenceSlot() const { return referenceSlot_; }
  bool weAreReference() const { return known_ > 0 && referenceId_ == selfId_; }

  // Pin the cycle to the reference car's beacon. The reference does not
  // necessarily hold slot zero, because a GPS-locked car outranks a
  // lower-numbered one that is free-running, so its own slot is subtracted to
  // recover the start of the cycle.
  void syncTo(uint32_t heardAtMs, uint32_t cycleMs);
  bool synced() const { return haveEpoch_; }

  // True while we are inside our own slot, with the cycle recovered from the
  // reference car's beacons. Falls back to free-running when there is no
  // schedule to speak of: one car alone on a channel has nothing to collide
  // with, and waiting for a sync that will never come means never speaking.
  bool inSlot(uint32_t nowMs, uint32_t cycleMs) const;

  // The same question when a GPS pulse has already told us where in the cycle
  // we are. Nothing about the reference car enters into it, which is the whole
  // point: there is no node whose leaving costs everyone their clock, and two
  // cars that have never heard each other are in step before they meet.
  //
  // The roster is still what decides which slot is ours. GPS says when the
  // slots are, not whose they are.
  bool inSlotAtPhase(uint32_t phaseMs, uint32_t cycleMs) const;

  // How wide one slot is. Exposed for the caller's jitter, which has to fit
  // inside it or it would push a transmission into the next car's slot.
  static uint32_t slotWidthMs(uint32_t cycleMs) { return cycleMs / MAX_SLOTS; }

 private:
  uint32_t selfId_ = 0;
  uint32_t referenceId_ = 0;
  uint8_t slot_ = SLOT_NONE;
  uint8_t referenceSlot_ = 0;
  uint8_t known_ = 0;
  uint32_t epochMs_ = 0;
  bool haveEpoch_ = false;
};

} // namespace touge
