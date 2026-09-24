#pragma once
//
// Who is on the ride, and which packets have already been through here.
//
// Platform-free on purpose: time comes in as an argument rather than being
// read from millis(). Flood suppression and roster ageing are the two things
// most likely to be subtly wrong, and both are far easier to get right when
// they can be tested at a thousand times real speed on a laptop.

#include <stdint.h>
#include <stddef.h>
#include "frame.h"
#include "hmac.h"
#include "ram.h"

namespace touge {

// How many cars the roster can hold. Rides run to twenty-eight, so this stays
// at 28 on every board; it is not the slot count (see MAX_SLOTS). A Rider is
// 96 bytes, 2.7 KB for the lot.
static const size_t MAX_RIDERS = 28;

// Distinct frames a second with the ride full: up to 32 slotted beacons, the
// shared window, and one talker's voice. Forwards of a frame are the same
// frame, so they add nothing here.
static const size_t PEAK_FRAMES_PER_SEC = 55;

// How long a packet is remembered, which has to outlive every echo of it. A
// forward waits at most FORWARD_JITTER_MAX_MS a hop (asserted below), so even
// the lean second is several times the real window. The table must hold the
// whole window at peak, or eviction silently shortens it (assert below).
// A Seen is 16 bytes: 1.5 KB roomy, 1 KB lean.
#if TOUGE_LEAN_RAM
static const uint32_t SEEN_TTL_MS = 1000;
static const size_t SEEN_SLOTS = 64;
#else
static const uint32_t SEEN_TTL_MS = 1500;
static const size_t SEEN_SLOTS = 96;
#endif

static_assert(SEEN_SLOTS >= PEAK_FRAMES_PER_SEC * SEEN_TTL_MS / 1000,
              "the dedupe table must be able to hold the window it claims, or "
              "eviction silently shortens it and the TTL means nothing");

// Forwarding a packet the instant it arrives is the wrong thing to do, and it
// gets worse the more cars are on the ride. Three nodes that all hear one
// frame rebroadcast in the same microsecond and collide, so nobody downstream
// gets it. Each forward waits before going out.
//
// ## Why the wait is not just noise
//
// It used to be a flat random slice of fifteen milliseconds, and the copy
// suppression underneath it could not do its job. Copies are counted as they
// arrive, and a forward only arrives after its sender has waited: so in the
// first few milliseconds after a frame lands, nobody has forwarded it yet,
// every hearer still counts exactly one copy, and every hearer whose slice
// expires in that window transmits. With the drain running at five
// milliseconds that was about a third of them - nine forwards of every frame
// in a full ride, roughly eight hundred and fifty milliseconds of air per
// second from positions alone. The suppression was not failing; it was never
// being consulted in time.
//
// Two changes, and the wait carries information instead of noise.
//
// Ordered by signal: the car that heard the frame most weakly goes first,
// because it is the one furthest out and the one whose forward reaches
// somewhere the original did not. Everyone nearer hears that forward while
// still holding their own, and drops it. The most useful forwarder is also
// the earliest, so the frame travels outward rather than in a random order.
//
// Spread by density: the window widens with the number of cars in earshot,
// because density is what broke it. Nine neighbours get ninety milliseconds
// to sort themselves out instead of fifteen, so the first bucket is a twentieth
// of them rather than a third. Bounded, because a forward that arrives after
// the next beacon is worth nothing.
static const uint32_t FORWARD_JITTER_MS = 30;
static const uint32_t FORWARD_JITTER_MAX_MS = 120;

// The ends of the useful signal range. Below the far end every frame is "as
// far away as it gets"; above the near end, "right here".
static const int16_t FORWARD_FAR_DBM = -95;
static const int16_t FORWARD_NEAR_DBM = -40;

// Two cars at the same distance must not transmit together, and nothing about
// their signal separates them.
static const uint32_t FORWARD_TIE_MS = 4;

static_assert(FORWARD_JITTER_MAX_MS >= FORWARD_JITTER_MS,
              "the ceiling cannot be below the floor");
static_assert(SEEN_TTL_MS >= 4 * (FORWARD_JITTER_MAX_MS + FORWARD_TIE_MS),
              "a packet must be remembered until well after its last forward could arrive");

// Frames waiting their turn to be forwarded. Twelve is enough that a busy relay
// does not drop forwards in normal use.
static const size_t FORWARD_SLOTS = 12;

// The longest position frame on the air: header, body with a full name, tag.
// 93 bytes, against the 250 a voice or text frame may need.
static const size_t POSITION_FRAME_MAX = FRAME_HEADER + POSITION_MIN + sizeof(Position::name) + TAG_LEN;

// How many of the slots hold a whole frame; the rest hold a position frame.
// A full-size slot is 250 bytes and nearly everything forwarded is a position,
// so a lean board keeps three for voice and text: one talker's frames are 60 ms
// apart and wait at most FORWARD_JITTER_MAX_MS, so about two are ever held.
// 1.6 KB of frame storage instead of 3 KB.
#if TOUGE_LEAN_RAM
static const size_t FORWARD_FULL_SLOTS = 3;
#else
static const size_t FORWARD_FULL_SLOTS = FORWARD_SLOTS;
#endif
static const size_t FORWARD_SMALL_BYTES = POSITION_FRAME_MAX;
static const size_t FORWARD_BYTES =
    FORWARD_FULL_SLOTS * FRAME_MAX + (FORWARD_SLOTS - FORWARD_FULL_SLOTS) * FORWARD_SMALL_BYTES;

static_assert(FORWARD_FULL_SLOTS >= 1 && FORWARD_FULL_SLOTS <= FORWARD_SLOTS,
              "at least one slot must hold a whole frame");
static_assert(FORWARD_SMALL_BYTES < FRAME_MAX, "a small slot that fits everything is a full one");

// Having heard this many copies of a packet, everyone within earshot already
// has it and adding another transmission helps nobody. In a four-car convoy
// in line of sight nearly every forward is already redundant; this is what
// stops them costing anything.
static const uint8_t SUPPRESS_AFTER = 3;

// A car nobody has heard from in two minutes is stale, not gone. It stays on
// the roster greyed out, because a car that vanishes off the screen every time
// it dips behind a ridge is worse than one that says "last seen 90s".
static const uint32_t RIDER_STALE_MS = 120000;
static const uint32_t RIDER_DROP_MS = 600000;

/**
 * How wide the forwarding window should be with this many cars in earshot.
 *
 * Widens with the neighbourhood, because the whole failure is a density one: a
 * window that gives three cars time to suppress each other gives nine cars no
 * time at all. Bounded so that a forward still beats the next beacon.
 */
uint32_t forwardSpreadMs(size_t neighbours);

/**
 * How long this particular hearer holds this particular frame.
 *
 * Weakest signal first. [tieBreak] is any random number; only its low few bits
 * are used, to separate cars the signal cannot.
 */
uint32_t forwardDelayMs(int16_t rssi, uint32_t spreadMs, uint32_t tieBreak);

/**
 * A deadline on a fixed [periodMs] grid, moved on only once it has passed.
 *
 * A deadline still ahead of [nowMs] comes back unchanged, so an extra send
 * before it (a movement-triggered beacon) cannot push the next one later. A
 * passed one moves to the first grid point after [nowMs]. Signed compare, so
 * the millis() wrap is harmless.
 */
uint32_t nextOnGrid(uint32_t deadlineMs, uint32_t periodMs, uint32_t nowMs);

enum Heard : uint8_t {
  HEARD_NONE = 0,
  HEARD_LORA = 1,
  HEARD_FAST = 2, // ESP-NOW, so line of sight
};

// A frame whose jitter is up, as Mesh::nextDue hands it back for rebroadcast.
struct Forward {
  uint8_t wire[FRAME_MAX];
  uint16_t len = 0;
  uint32_t src = 0;
  uint32_t id = 0;
  uint32_t dueMs = 0;
};

struct Rider {
  uint32_t id = 0;
  Position pos;
  uint32_t atMs = 0;
  uint8_t via = HEARD_NONE;
  int16_t rssi = 0;
  uint8_t hopsAway = 0;
  /**
   * The 2.4 GHz channel this car was last heard on.
   *
   * Without it, "heard" counted anyone noticed in the last few seconds while
   * the channel was read at the instant of asking - so a sweeping board could
   * hear a car on channel 11, move to channel 1, and report ch=1 heard=1. Both
   * halves true, the line as a whole impossible, and no way to tell a real
   * contact from that.
   */
  uint8_t chan = 0;
  bool used = false;
};

class Mesh {
 public:
  void reset();

  // True the first time a given (src, id) is offered, false every time after
  // within the TTL. This is what stops four cars in a bowl from echoing one
  // packet around each other until the battery dies.
  //
  // Every call counts, including the ones that return false, because how many
  // copies arrived is what tells us whether a forward is still worth making.
  bool firstSight(uint32_t src, uint32_t id, uint32_t nowMs);

  // How many copies of a packet have come past. Zero if it is unknown or has
  // aged out.
  uint8_t copies(uint32_t src, uint32_t id, uint32_t nowMs) const;

  // Hold a frame to forward once the jitter has elapsed. False if there is no
  // room, which means the ride is busier than this can keep up with and the
  // frame is dropped rather than delaying the ones already queued. A position
  // takes a small slot while there is one, keeping the full ones for voice.
  bool defer(const uint8_t* wire, size_t len, uint32_t src, uint32_t id, uint32_t dueMs);

  // The next held frame whose time has come and which is still worth sending.
  // Frames overtaken by neighbours while they waited are discarded here rather
  // than being handed back, so the caller only ever sees what it should send.
  bool nextDue(uint32_t nowMs, Forward& out);

  // Forwards thrown away because enough neighbours beat us to them. In a tight
  // convoy this should be most of them.
  uint32_t suppressed() const { return suppressed_; }

  // Fold a heard position into the roster. Returns the slot, or nullptr when
  // the roster is full and this rider is not already on it.
  Rider* note(uint32_t src, const Position& p, uint8_t via, int16_t rssi, uint8_t hopsAway,
              uint32_t nowMs, uint8_t chan);

  // Drop riders nobody has heard from in a very long time.
  void age(uint32_t nowMs);

  const Rider* riders() const { return riders_; }
  size_t count() const;

  /**
   * Cars heard on one channel inside a window. The denominator for the hop
   * decision.
   *
   * count() is every seat filled in the last ten minutes on any channel, which
   * is the right answer for a map and a badly wrong one for airtime. After a
   * car park gathering of twenty-eight that rolls out as eight, count() stays
   * at twenty-eight for ten minutes, so the reference expects the airtime of
   * twenty-eight cars, receives the airtime of eight, reads twenty-eight
   * percent against a forty percent threshold, and moves the whole ride off a
   * channel that was working.
   */
  size_t countOn(uint8_t chan, uint32_t windowMs, uint32_t nowMs) const;
  const Rider* find(uint32_t id) const;

  // Packet ids must never be reused under one channel key, because the id is
  // half the AES-CTR nonce. Counting up from zero would restart the sequence
  // on every reboot, so the counter is seeded from NVS at boot with a value
  // safely ahead of anything this node has already sent.
  void seedIds(uint32_t from) { lastId_ = from; }
  uint32_t nextId() { return ++lastId_; }
  uint32_t lastId() const { return lastId_; }

 private:
  struct Seen {
    uint32_t src = 0;
    uint32_t id = 0;
    uint32_t atMs = 0;
    // Saturates rather than wrapping. A packet seen 255 times and one seen 260
    // times mean the same thing, and wrapping to zero would un-suppress it.
    uint8_t count = 0;
    bool used = false;
  };

  // A frame waiting out its jitter. Its bytes are held at slotBytes(i).
  struct Held {
    uint32_t src = 0;
    uint32_t id = 0;
    uint32_t dueMs = 0;
    uint16_t len = 0;
    bool used = false;
  };

  Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs);
  const Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs) const;

  // Slots 0 .. FORWARD_FULL_SLOTS-1 are full-size, the rest position-sized.
  static size_t slotCapacity(size_t i);
  uint8_t* slotBytes(size_t i);
  bool holdIn(size_t i, const uint8_t* wire, size_t len, uint32_t src, uint32_t id, uint32_t dueMs);

  Seen seen_[SEEN_SLOTS];
  Rider riders_[MAX_RIDERS];
  Held held_[FORWARD_SLOTS];
  uint8_t forwardBytes_[FORWARD_BYTES];
  uint32_t lastId_ = 0;
  uint32_t suppressed_ = 0;
};

} // namespace touge
