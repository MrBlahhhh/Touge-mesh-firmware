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
// width that changed as cars joined would move everybody's slot at once, and
// the moment a roster is in flux is exactly when you want the schedule stable.
static const uint8_t MAX_SLOTS = MAX_RIDERS + 1;

class Schedule {
 public:
  void reset();

  // Work out our slot from the roster. Every car runs the same sort over the
  // same ids and lands on the same answer, so no slot has to be handed out.
  // Cars that have not heard each other yet will disagree for a cycle or two;
  // that is what the jitter inside the slot is for.
  void rebuild(uint32_t selfId, const Rider* riders, size_t maxRiders);

  uint8_t slot() const { return slot_; }
  uint8_t known() const { return known_; }
  uint32_t referenceId() const { return referenceId_; }
  bool weAreReference() const { return known_ > 0 && referenceId_ == selfId_; }

  // Pin the cycle to the reference car's beacon. It holds slot zero by
  // construction, so the moment its beacon lands is the start of a cycle.
  void syncTo(uint32_t heardAtMs);
  bool synced() const { return haveEpoch_; }

  // True while we are inside our own slot. False when there is no schedule to
  // speak of, in which case the caller should fall back to free-running: one
  // car alone on a channel has nothing to collide with.
  bool inSlot(uint32_t nowMs, uint32_t cycleMs) const;

  // How wide one slot is. Exposed for the caller's jitter, which has to fit
  // inside it or it would push a transmission into the next car's slot.
  static uint32_t slotWidthMs(uint32_t cycleMs) { return cycleMs / MAX_SLOTS; }

 private:
  uint32_t selfId_ = 0;
  uint32_t referenceId_ = 0;
  uint8_t slot_ = 0;
  uint8_t known_ = 0;
  uint32_t epochMs_ = 0;
  bool haveEpoch_ = false;
};

} // namespace touge
