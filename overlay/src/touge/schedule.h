#pragma once
//
// Who transmits when on the 2.4 GHz lane.
//
// Every car beacons once a second, so the schedule is one second long and
// every car that holds a lease has one slot in it to itself. Leases are taken,
// not handed out: each car applies the same rule to the same roster, the way
// the reference is elected, so there is no coordinator whose loss costs the
// ride its schedule. The reference car only supplies the clock.
//
// It is not real TDMA. ESP-NOW sits on 802.11, whose MAC does its own carrier
// sense underneath, and without GPS the clock is recovered from beacons to a
// few milliseconds. What it buys is that cars stop colliding with each other on
// purpose, which is the loss that grows with the size of the ride.
//
// ## The second, laid out
//
// Four 250 ms blocks. Each block is eight 27 ms leased slots and then a 34 ms
// shared window:
//
//   | s0 s4 s8 ... s28 | shared | s1 s5 ... s29 | shared | s2 ... | s3 ... |
//      0             216   250
//
// Slot s is in block s % 4 at row s / 4, so the four slots of a row sit at
// the same offset in each block, exactly 250 ms apart. Leases fill block 0
// first (slots 0, 4, 8 ... 28), then block 1, 2, 3: the first eight cars get
// a row each.
//
// ## Extra beacons in slots nobody holds
//
// A car may also transmit in the unleased slots of its own row, so with few
// cars each beacons up to four times a second, evenly spaced, and drops back
// to once a second as the rows fill. Which car gets a free slot depends only
// on which blocks of that row are leased, so every car that can hear the row
// derives the same answer; see Schedule::extraSlots. Every car's extras put
// together can never use more than the 32 slots, so the total is min(32, 4N)
// frames a second, and the shared window is untouched:
//
//   cars   per car (Hz)                          frames/s
//   1      1 (alone: nobody to send extras to)   1
//   2-8    4                                     4N
//   10     4 x 6, 2 x 4 (two rows hold two)      32
//   16     2                                     32
//   20     2 x 12, 1 x 8                         32
//   25     2 x 7, 1 x 18                         32
//   32     1                                     32
//
// Extras are sent with no hops left, so nobody forwards them, and flagged, so
// they never set a clock and never enter a slot map (clash detection sees
// lease beacons only). Extras stop everywhere as soon as any car in earshot
// needs a slot, so a joiner always finds the free slots silent, and while
// the roster is full (MAX_RIDERS others), since it may be hiding a lease.
// The 32-car row above therefore assumes a roster bigger than today's 28.
//
// The shared window is for cars without a lease: a car still listening before
// its first claim, and anything beyond MAX_SLOTS. It is also the room step 5 of
// docs/SCALE-PLAN.md will put relays in. Forwards are still unslotted today.
//
// ## Why 32 slots and not more
//
// A slot has to hold a full frame plus the error of the clock handed down the
// sync chain. A 250 byte ESP-NOW LR frame is about 8 ms at 250 kbps, and the
// module asserts a slot is at least three of those (24 ms) and that a car
// MAX_REF_HOPS down the chain still finishes inside it (3 ms a hop, 6 hops,
// plus 8 ms of frame: 26 ms). 27 ms is the narrowest slot that passes both.
// 1000 / 27 is 37 slots with nothing left over for joins or relays; 32 slots
// is 864 ms and leaves 136 ms a second shared.
//
// 32 is at least the 25-car target and at least a full roster (MAX_RIDERS
// others plus us). At the old 250 ms cycle a car's slot came round four times
// a second and it used one of them, so the nine slots there were really 36
// transmit chances of which 27 went unused. This spends them.

#include <stdint.h>
#include <stddef.h>
#include "mesh.h"

namespace touge {

// One position per car per second, so one lease per car per second.
static const uint32_t SCHEDULE_MS = 1000;
static const uint8_t BLOCKS = 4;
static const uint32_t BLOCK_MS = SCHEDULE_MS / BLOCKS;
static const uint8_t SLOTS_PER_BLOCK = 8;
static const uint32_t SLOT_MS = 27;
static const uint8_t MAX_SLOTS = BLOCKS * SLOTS_PER_BLOCK;

// Where in each block the shared window opens, and how long it runs.
static const uint32_t SHARED_OFFSET_MS = SLOTS_PER_BLOCK * SLOT_MS;
static const uint32_t SHARED_MS = BLOCK_MS - SHARED_OFFSET_MS;

// Roughly how long a full frame takes on the air: 250 bytes at ESP-NOW LR's
// 250 kbps. Unmeasured (see firmware/docs/REDESIGN-30-CARS.md); a position
// frame is nearer 120 bytes with the 802.11 overhead, about 4 ms.
static const uint32_t FRAME_AIRTIME_MS = 8;

// Room a frame needs in front of it before it may start. Holding a slot is
// permission to finish inside it, not just to begin. A frame and 2 ms margin.
static const uint32_t SLOT_GUARD_MS = 10;

// Unleased cars spread their start over this much of the shared window, so
// several joiners in one window do not all key up on the same tick.
static const uint32_t SHARED_SPREAD_MS = 16;

static_assert(BLOCKS * BLOCK_MS == SCHEDULE_MS, "the blocks must tile the second");
static_assert(SHARED_OFFSET_MS + SHARED_MS == BLOCK_MS, "slots and the shared window tile a block");
static_assert(SLOT_MS >= 3 * FRAME_AIRTIME_MS,
              "slots must be several frame times wide; fewer slots or a longer schedule");
static_assert(SHARED_MS >= 3 * FRAME_AIRTIME_MS,
              "the shared window has to fit more than one joiner");
static_assert(SLOT_GUARD_MS >= FRAME_AIRTIME_MS, "the guard has to cover a whole frame");
static_assert(SHARED_SPREAD_MS + SLOT_GUARD_MS < SHARED_MS,
              "the latest spread start still has to finish inside the window");
static_assert(MAX_SLOTS >= 25, "the target is a 25-car ride with a slot each");
static_assert(MAX_SLOTS >= MAX_RIDERS + 1,
              "every car the roster can hold, plus us, must be able to hold a lease");
static_assert(MAX_SLOTS < SLOT_NONE, "SLOT_NONE must not be a real slot");
static_assert(MAX_SLOTS == SLOT_MAP_LEN, "Position::slotMap has one entry per slot");

// How far back a car's slot map reaches. Every holder transmits once a
// second, so a second and a half always spans at least one of its beacons.
static const uint32_t HEARD_WINDOW_MS = SCHEDULE_MS + SCHEDULE_MS / 2;

/**
 * How long an established lease rides out a clash before moving.
 *
 * A clash is nearly always a newcomer landing on a slot it could not see was
 * taken, and the newcomer moves at once. The incumbent waits two map windows
 * so it is the newcomer that moves, not both; if the clash outlasts that, it
 * moves too. A lease counts as established after ESTABLISHED_LEASE_MS.
 */
static const uint32_t CLASH_PATIENCE_MS = 2 * HEARD_WINDOW_MS;
static const uint32_t ESTABLISHED_LEASE_MS = 10000;

/** A car's one-byte tag in slot maps. Never 0, which means nobody. */
uint8_t slotTag(uint32_t id);

// How long a car may be quiet and still keep the job of keeping time. Eight
// idle beacons.
static const uint32_t REFERENCE_LAPSE_MS = 8000;

/**
 * How long a lease outlives the last time its holder was heard.
 *
 * Longer than a handful of missed beacons, because reassigning a slot on a
 * short dropout puts two cars in it. Far shorter than the roster's ten
 * minutes, which is right for the map and wrong for airtime. A car that has
 * heard nobody for this long drops its own lease too, so one that drives back
 * into range rejoins as a newcomer instead of reclaiming a slot that may have
 * been given away.
 */
static const uint32_t LEASE_MS = 30000;

static_assert(LEASE_MS > REFERENCE_LAPSE_MS, "a car should lose the clock before it loses its slot");
static_assert(LEASE_MS < RIDER_DROP_MS, "a lease held until the roster forgets the car is the old bug");

/**
 * How long a car listens after first hearing the ride before it claims a slot.
 *
 * Two whole schedules, so every car in earshot has beaconed at least twice and
 * one lost beacon does not hide a lease. Bounded, so a returning car never
 * waits indefinitely: after this it takes a free slot or, if there is none,
 * stays in the shared window.
 */
static const uint32_t JOIN_LISTEN_MS = 2 * SCHEDULE_MS + 500;

static_assert(JOIN_LISTEN_MS >= 2 * SCHEDULE_MS, "listen through two beacons from everyone");

/** Where slot `slot` opens, in ms from the start of the schedule. */
uint32_t slotStartMs(uint8_t slot);

/** True when generation `a` was issued before `b`. Wrap-safe. */
bool leaseOlder(uint16_t a, uint16_t b);

class Schedule {
 public:
  void reset();

  /**
   * Re-read the roster: who keeps time, and whether our lease stands.
   *
   * Leases are ordered by generation. Every car advertises the highest
   * generation it knows of (the schedule generation), and a new lease is
   * stamped one past that, so any lease a car could have heard of is older
   * than the one it takes. Two claims on one slot go to the older lease, then
   * the lower node number. So a newcomer or a returning car always yields to
   * an incumbent, and after a merge exactly one of each clashing pair moves.
   *
   * Cars that need a slot (unleased, or holding one they have lost) take the
   * free slots in node-number order: every car with the same roster computes
   * the same assignment, so 25 cars joining together land on 25 slots
   * without colliding first. A slot is free when no car we hear holds it and
   * no neighbour's slot map shows anyone in it.
   *
   * When rosters differ that can still put two cars on one slot, and they
   * never hear each other. So a holder also reads its own slot in its
   * neighbours' slot maps: if they hear somebody else there more than us, or
   * nobody at all, it gives the slot up and listens again as a joiner. Its
   * unleased beacons stop everybody's extras and put it back in the queue,
   * where the cars it clashed with now see it and rank it.
   */
  void rebuild(uint32_t selfId, bool selfLocked, const Rider* riders, size_t maxRiders,
               uint32_t nowMs);

  /**
   * A position frame arrived straight from `senderId` (not forwarded),
   * advertising `slot`. Feeds the slot map.
   */
  void heardSlot(uint8_t slot, uint32_t senderId, uint32_t nowMs);

  /** Who we heard in each slot within HEARD_WINDOW_MS, as slot tags. */
  void fillSlotMap(uint32_t nowMs, uint8_t map[SLOT_MAP_LEN]) const;

  /**
   * The unleased slots of our row that are ours to send extra beacons in, one
   * bit per slot. Empty unless we hold a lease, have company, are not in a
   * clash, and no car in earshot is waiting for a slot.
   *
   * A free block f of the row goes to the lease in the block opposite it,
   * (f + 2) % 4, if that is held; otherwise to the nearest lease before it in
   * time. One lease takes all three free blocks (4 Hz). Two leases in
   * adjacent blocks take one each, 500 ms after their own (2 Hz, evenly
   * spaced). Three leases leave one block, which goes to the middle one. The
   * rule only reads which blocks are leased, from the roster and the
   * neighbours' slot maps, so cars that can collide agree on it.
   */
  uint32_t extraSlots() const { return extraMask_; }

  /** Whether the phase is inside one of our extra slots with room to finish. */
  bool inExtraSlotAtPhase(uint32_t phaseMs) const;

  /** The same on the beacon-recovered clock. False without an epoch. */
  bool inExtraSlot(uint32_t nowMs) const;

  // SLOT_NONE while listening, or when there are more cars than slots.
  uint8_t slot() const { return slot_; }
  bool claimed() const { return slot_ < MAX_SLOTS; }
  uint16_t leaseGeneration() const { return leaseGen_; }
  uint16_t generation() const { return schedGen_; }

  // Us plus every car heard within LEASE_MS.
  uint8_t known() const { return known_; }
  uint32_t referenceId() const { return referenceId_; }

  /**
   * The car whose beacon sets our clock: the reference when we hear it
   * directly, otherwise the direct neighbour nearest it. It must hold a lease,
   * because a beacon is only a time mark if we know which slot it was sent in.
   * Zero when we are the reference or nobody suitable is in earshot.
   */
  uint32_t parentId() const { return parentId_; }

  /**
   * Whether a position frame heard directly from `src` should set our clock:
   * the parent's (or the reference's) once we have an epoch, any leased car's
   * before that.
   */
  bool takesClockFrom(uint32_t src) const;

  /** How many hops away the reference is, or REF_UNREACHABLE. */
  uint8_t hopsToReference() const { return refHops_; }
  bool referenceLocked() const { return referenceLocked_; }
  bool weAreReference() const { return known_ > 0 && referenceId_ == selfId_; }

  /**
   * Pin the schedule to a beacon heard directly from the parent.
   *
   * @param senderSlot the slot the beacon itself advertises. Taken from the
   *   frame, not the roster, so a parent that has just moved slot does not
   *   put us a slot out for a second. Ignored if it is not a leased slot.
   * @param senderLateMs how far into its slot the sender is expected to have
   *   transmitted, which is half a module tick on average.
   */
  void syncTo(uint32_t heardAtMs, uint8_t senderSlot, uint32_t senderLateMs = 0);

  /**
   * Declare the schedule ourselves. Only for the reference with no epoch yet:
   * it never hears itself, so without this it would free-run and drag every
   * other car's slots along with it. Does nothing while alone, or while a
   * leased car is in earshot to take the ride's epoch from instead.
   */
  void startEpoch(uint32_t nowMs);
  bool synced() const { return haveEpoch_; }

  /**
   * Whether we may start a frame now, on the beacon-recovered clock.
   *
   * Alone, or with no epoch yet: yes, since there is nothing to keep to and
   * waiting for a sync that may not come means never being heard.
   */
  bool inSlot(uint32_t nowMs) const;

  /**
   * The same, when a GPS pulse already says where in the second we are. GPS
   * says when the slots are, the roster says whose they are.
   *
   * Leased: inside our own slot with room to finish. Unleased: inside the
   * shared window of the block drawn for this beacon, at the drawn offset or
   * later.
   */
  bool inSlotAtPhase(uint32_t phaseMs) const;

  /**
   * Draw where an unleased car's next beacon goes: which block's shared
   * window, and how far into it. Once per beacon, with any random number.
   *
   * Fixed per car, the draw repeated every second, and boards switched on
   * together (so on the same 1 s grid) collided in the same window every time
   * and never heard each other while listening.
   */
  void drawSharedTurn(uint32_t random);

 private:
  void electReference(bool selfLocked, const Rider* riders, size_t maxRiders, uint32_t nowMs);
  void settleLease(const Rider* riders, size_t maxRiders, uint32_t nowMs);
  void chooseParent(const Rider* riders, size_t maxRiders, uint32_t nowMs);
  void planExtras(const Rider* riders, size_t maxRiders, uint32_t nowMs);

  uint32_t selfId_ = 0;
  uint32_t referenceId_ = 0;
  bool referenceLocked_ = false;
  uint32_t parentId_ = 0;
  uint8_t refHops_ = REF_UNREACHABLE;
  bool leasedInEarshot_ = false;

  uint8_t slot_ = SLOT_NONE;
  uint32_t leasedAtMs_ = 0;
  // Since when the slot maps have said somebody else is on our slot.
  bool clashing_ = false;
  uint32_t clashSinceMs_ = 0;
  uint16_t leaseGen_ = 0;
  uint16_t schedGen_ = 0;
  // False until some car has told us a generation. A fresh board adopts the
  // ride's outright, since a wrap-safe compare against its own zero is
  // meaningless once the ride is more than 32768 leases in.
  bool genKnown_ = false;
  // When we first heard another car since last being alone.
  uint32_t firstHeardMs_ = 0;
  bool heardAnyone_ = false;

  uint32_t slotHeardMs_[MAX_SLOTS] = {};
  uint8_t slotHeardTag_[MAX_SLOTS] = {}; // 0 until somebody is heard there

  // Some car in earshot is unleased or has lost its slot. Set by settleLease.
  bool someoneWaiting_ = false;
  uint32_t extraMask_ = 0;

  uint8_t sharedBlock_ = 0;
  uint32_t sharedOffsetMs_ = 0;

  uint8_t known_ = 0;
  uint32_t epochMs_ = 0;
  bool haveEpoch_ = false;
};

} // namespace touge
