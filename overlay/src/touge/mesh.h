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
// 108 bytes, 3.0 KB for the lot.
static const size_t MAX_RIDERS = 28;

// Distinct frames a second with the ride full: up to 32 slotted beacons, the
// shared window, and one talker's voice. Forwards of a frame are the same
// frame, so they add nothing here.
static const size_t PEAK_FRAMES_PER_SEC = 55;

// How long a packet is remembered, which has to outlive every echo of it. A
// forward waits at most FORWARD_DELAY_MAX_MS a hop (asserted below). The table
// must hold the whole window at peak, or eviction silently shortens it (assert
// below). A Seen is 16 bytes: 1.5 KB roomy, 1 KB lean.
static const uint32_t SEEN_TTL_LEAN_MS = 1000;
#if TOUGE_LEAN_RAM
static const uint32_t SEEN_TTL_MS = SEEN_TTL_LEAN_MS;
static const size_t SEEN_SLOTS = 64;
#else
static const uint32_t SEEN_TTL_MS = 1500;
static const size_t SEEN_SLOTS = 96;
#endif

static_assert(SEEN_SLOTS >= PEAK_FRAMES_PER_SEC * SEEN_TTL_MS / 1000,
              "the dedupe table must be able to hold the window it claims, or "
              "eviction silently shortens it and the TTL means nothing");

// Forwarding a packet the instant it arrives is the wrong thing to do: every
// car that heard it transmits together and the copies collide. Each forward
// waits, and the wait carries information.
//
// A flat random 15 ms left the copy suppression below nothing to count: no
// forward had arrived before a third of the hearers' waits ran out, so a full
// ride forwarded every frame about nine times, some 850 ms of air a second from
// positions alone. So:
// - Weakest signal first. That car is furthest out, and its forward reaches
//   somewhere the original did not; everyone nearer hears it and drops theirs.
// - Wider with more cars in earshot, from FORWARD_JITTER_MS to
//   FORWARD_JITTER_MAX_MS: nine neighbours get 90 ms instead of 15. Bounded,
//   since a forward that arrives after the next beacon is worth nothing.
// - In whole steps of a pass and a frame (forwardStepMs), from build 51. Two
//   cars whose waits differ by less than that both send before either hears
//   the other. The tie was 0-3 ms against a 5 ms pass, and every car at -40 dBm
//   or stronger had the same wait, so three cars on the bench put every voice
//   frame on the air three times. Now two waits are equal or a step apart, and
//   a tie of up to FORWARD_TIE_STEPS - 1 steps spreads cars of similar signal:
//   two equal hearers share a step one time in two. That cuts the bench's
//   copies only at two copies a frame; voice keeps three (VOICE_SUPPRESS_AFTER).
static const uint32_t FORWARD_JITTER_MS = 30;
static const uint32_t FORWARD_JITTER_MAX_MS = 120;

// The ends of the useful signal range. Below the far end every frame is "as
// far away as it gets"; above the near end, "right here".
static const int16_t FORWARD_FAR_DBM = -95;
static const int16_t FORWARD_NEAR_DBM = -40;

// How often the module drains the radio and sends what is due: TougeFastModule's
// TICK_MS, asserted equal there. A forward leaves on a pass, not between them.
static const uint32_t FORWARD_PASS_MS = 5;

// Steps the tie spreads cars of similar signal over.
// 2B sim, 192 seeds: 3 cost a clean line's 3-hop positions 0.8 points (SE 0.2; 0.7 on 512 more), 4 cost 1.3;
// 1 costs a lossy park's 1-hop 0.6-0.8. 2 costs a clean park air instead: 1.8 copies a lease frame and 20 %,
// against 1.3 and 15 % at 3.
static const uint32_t FORWARD_TIE_STEPS = 2;

// 802.11 and ESP-NOW framing around our bytes: MAC header 24, category and OUI
// 4, random 4, vendor element 7, FCS 4.
static const size_t FAST_FRAME_OVERHEAD = 43;
// Assumed ESP-NOW long range at 250 kbit/s. Nothing sets a rate (espnow.cpp
// allows 11b/g/n and LR; "tr" in {"fr"} says what the driver used), and the LR
// preamble is left out. Plan 1C.8 measures the real figure.
static const uint32_t FAST_BITS_PER_MS = 250;

// On-air time of a 2.4 GHz frame of `wireLen` bytes, rounded up to a whole ms:
// 5 for a position, 6 for today's 107 B voice packet, 10 for a full frame.
constexpr uint32_t fastAirtimeMs(size_t wireLen) {
  return (uint32_t)(((wireLen + FAST_FRAME_OVERHEAD) * 8 + FAST_BITS_PER_MS - 1) / FAST_BITS_PER_MS);
}

// How far apart two forwarders' waits must be for the later one to hear the
// earlier's copy before its own pass: the pass the earlier leaves on, then the
// airtime, rounded up to whole passes. 10 ms for a position, 15 for voice.
constexpr uint32_t forwardStepMs(size_t wireLen) {
  return FORWARD_PASS_MS + (fastAirtimeMs(wireLen) + FORWARD_PASS_MS - 1) / FORWARD_PASS_MS * FORWARD_PASS_MS;
}

// The longest a forward waits: the widest window, then the last tie step of a
// full frame. 135 ms.
static const uint32_t FORWARD_DELAY_MAX_MS =
    FORWARD_JITTER_MAX_MS + (FORWARD_TIE_STEPS - 1) * forwardStepMs(FRAME_MAX);

static_assert(FORWARD_JITTER_MAX_MS >= FORWARD_JITTER_MS,
              "the ceiling cannot be below the floor");
static_assert(SEEN_TTL_MS >= 4 * FORWARD_DELAY_MAX_MS,
              "a packet must be remembered until well after its last forward could arrive");

// Frames waiting their turn to be forwarded. Twelve is enough that a busy relay
// does not drop forwards in normal use.
static const size_t FORWARD_SLOTS = 12;

// The longest position frame on the air: header, body with a full name, tag.
// 105 bytes, against the 250 a voice or text frame may need.
static const size_t POSITION_FRAME_MAX = FRAME_HEADER + POSITION_MIN + sizeof(Position::name) + TAG_LEN;

// How many of the slots hold a whole frame; the rest hold a position frame.
// A full-size slot is 250 bytes and nearly everything forwarded is a position,
// so a lean board keeps three for voice and text: one talker's frames are 60 ms
// apart and wait at most FORWARD_DELAY_MAX_MS (135 ms), so three at most are
// ever held. 1.7 KB of frame storage instead of 3 KB.
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
// stops them costing anything. Positions; voice has its own below.
static const uint8_t SUPPRESS_AFTER = 3;

// The hops a position leaves with. ESP-NOW reaches roughly as far as you can
// see, so two covers a convoy strung out far enough that the front and back
// cannot hear each other but the middle hears both. Three would mostly buy
// duplicate transmissions.
static const uint8_t FAST_HOPS = 2;

// Voice (FRAME_VOICE, the test talker's frames included) keeps the positions'
// hops and copies. The 2B ride sim (plan 2B, Results) found no pair that carries
// voice further without costing positions somewhere.
// 2B sim, 192 seeds: 3 hops (2 copies) lift cars 8-16 of a line from 17 to 29 % on time but cost its 2-hop
// positions 0.8 points at p = 0.4; 1 hop reaches 4 %.
static const uint8_t VOICE_HOPS = 2;
// 2B sim, 192 seeds: 2 copies cut a 25-car park from 10.7 to 9.2 copies a voice frame but cost cars 8-16 of
// a line 1.4 points.
static const uint8_t VOICE_SUPPRESS_AFTER = 3;

static_assert(SUPPRESS_AFTER >= 2 && VOICE_SUPPRESS_AFTER >= 2,
              "the copy we heard counts as one, so at 1 every forward would suppress itself");
static_assert(FAST_HOPS >= 1 && VOICE_HOPS >= 1, "a frame with no hops is never forwarded");
static_assert(VOICE_HOPS * (FORWARD_DELAY_MAX_MS + FORWARD_PASS_MS + fastAirtimeMs(FRAME_MAX)) < SEEN_TTL_LEAN_MS,
              "a lean board must still remember a voice frame when its last hop's copy arrives, "
              "or that copy is new to it and goes round again");

// What a frame of `frameType` leaves with, and the copies after which a held
// forward of it is dropped.
inline uint8_t startHopsFor(uint8_t frameType) { return frameType == FRAME_VOICE ? VOICE_HOPS : FAST_HOPS; }
inline uint8_t suppressAfterFor(uint8_t frameType) {
  return frameType == FRAME_VOICE ? VOICE_SUPPRESS_AFTER : SUPPRESS_AFTER;
}

// How many hops a copy has come: 0 straight from its origin. Relayed is above 0,
// whatever the type starts with.
inline uint8_t hopsTravelled(uint8_t frameType, uint8_t hops) {
  const uint8_t start = startHopsFor(frameType);
  return hops < start ? (uint8_t)(start - hops) : 0;
}

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
 * How long this particular hearer holds this particular frame, from when it
 * arrived.
 *
 * Weakest signal first, in whole forwardStepMs(wireLen) steps. [tieBreak] is
 * any random number; tieBreak % FORWARD_TIE_STEPS more steps separate cars the
 * signal cannot. [wireLen] is the frame's length on the wire, which sets the
 * step.
 */
uint32_t forwardDelayMs(int16_t rssi, uint32_t spreadMs, uint32_t tieBreak,
                        size_t wireLen = POSITION_FRAME_MAX);

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
  // When a copy straight from this car (hopsAway 0) was last heard on 2.4 GHz.
  uint32_t directMs = 0;
};

// How long ago a rider was last heard, wrap-safe. A stamp later than `nowMs`
// reads as just heard: a caller on an older time than the note, not a car gone
// quiet. Mesh::note evicts by this, and fastrelay.cpp reads the roster by it.
inline uint32_t riderAgeMs(const Rider& r, uint32_t nowMs) {
  const int32_t age = (int32_t)(nowMs - r.atMs);
  return age > 0 ? (uint32_t)age : 0;
}

// How long a car heard directly stays direct when only a forwarded copy of its
// next frame gets through. Its direct copy is lost 35-50 % of the time on the
// bench, and each loss used to make it a hop away for a second (review B7): not
// a judge, not a parent, its slot map unread. The schedule's map window.
static const uint32_t DIRECT_HOLD_MS = 1500;

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
  // Dropped unsent once [suppressAfter] copies have come past: pass
  // suppressAfterFor(the frame's type).
  bool defer(const uint8_t* wire, size_t len, uint32_t src, uint32_t id, uint32_t dueMs,
             uint8_t suppressAfter = SUPPRESS_AFTER);

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
    uint8_t suppressAfter = SUPPRESS_AFTER;
    bool used = false;
  };

  Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs);
  const Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs) const;

  // Slots 0 .. FORWARD_FULL_SLOTS-1 are full-size, the rest position-sized.
  static size_t slotCapacity(size_t i);
  uint8_t* slotBytes(size_t i);
  bool holdIn(size_t i, const uint8_t* wire, size_t len, uint32_t src, uint32_t id, uint32_t dueMs,
              uint8_t suppressAfter);

  Seen seen_[SEEN_SLOTS];
  Rider riders_[MAX_RIDERS];
  Held held_[FORWARD_SLOTS];
  uint8_t forwardBytes_[FORWARD_BYTES];
  uint32_t lastId_ = 0;
  uint32_t suppressed_ = 0;
};

} // namespace touge
