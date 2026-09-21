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

namespace touge {

// Eight cars is more than any group ride that still works as a group ride.
// How many cars the roster can hold.
//
// Rides run to twenty-eight cars. Eight was the bench group this was built
// against, and a fixed array of eight silently drops the twenty-ninth car -
// and the twelfth, and the ninth. A Rider is about fifty-six bytes, so
// twenty-eight is under two kilobytes on an ESP32-S3, which is nothing.
//
// This is deliberately not the slot count any more. See MAX_SLOTS.
static const size_t MAX_RIDERS = 28;

// How busy the air gets with the ride full.
//
// Twenty-eight cars beaconing, plus the forwards their neighbours make of
// those beacons. Measured as distinct frames, since that is what the table
// below stores one of each.
static const size_t PEAK_FRAMES_PER_SEC = 55;

// A packet is remembered long enough to outlive every echo of itself.
//
// Thirty seconds was the old figure and it was a fiction. Sixty-four entries
// against fifty-odd distinct frames a second turns the whole table over in
// about a second and a quarter, so nothing ever survived to be forgotten by
// the TTL: eviction got there first, every time, and the number described a
// window that did not exist whenever it mattered. No packet was mishandled
// because of it - forwards settle inside about twenty milliseconds, which is
// two orders of magnitude inside even the real window - but a constant that
// cannot be true under load is one nobody can reason from, and the test that
// pinned it was pinning the fiction.
//
// So: a window short enough to be honest, and a table large enough to hold
// it. Three seconds is twelve cycles and a hundred and fifty times longer than
// a forward takes to settle. The assert below is what keeps the two from
// drifting apart again.
static const uint32_t SEEN_TTL_MS = 3000;
static const size_t SEEN_SLOTS = 192;

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
// Frames waiting their turn to be forwarded.
//
// Eight, chosen when the roster was eight. Twenty-eight cars that all hear one
// frame all try to defer it and twenty are refused - and the one refused may
// be the only board that can reach the tail. The queue holds a frame each, so
// thirty-two is about eight kilobytes of heap, which is affordable and a great
// deal cheaper than a silently dropped relay.
static const size_t FORWARD_SLOTS = 32;

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

enum Heard : uint8_t {
  HEARD_NONE = 0,
  HEARD_LORA = 1,
  HEARD_FAST = 2, // ESP-NOW, so line of sight
};

// A frame waiting out its jitter before being rebroadcast.
struct Forward {
  uint8_t wire[FRAME_MAX];
  uint16_t len = 0;
  uint32_t src = 0;
  uint32_t id = 0;
  uint32_t dueMs = 0;
  bool used = false;
};

struct Rider {
  uint32_t id = 0;
  Position pos;
  uint32_t atMs = 0;
  // When this car's position was last handed to the phone. Zero means never.
  uint32_t phoneMs = 0;
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
  // frame is dropped rather than delaying the ones already queued.
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

  /**
   * Whether this car's position is due to go to the phone, stamping it if so.
   *
   * The 2.4 GHz lane carries a position per car four times a second, and each
   * one was becoming its own MeshPacket on the BLE queue. Twenty-eight cars is
   * a hundred and twelve packets a second down a link that does about thirty,
   * so the queue backed up at its thirty-two packet ceiling and the radio
   * started dropping the newest - positions, fast-lane status and voice alike.
   *
   * The screen cannot use more than a few a second and the roster is already
   * being kept at full rate underneath, so this is purely about what crosses
   * the wire. A car whose frames are being dropped for want of queue space is
   * worse off at four a second than at one.
   */
  bool phoneDue(uint32_t id, uint32_t everyMs, uint32_t nowMs);
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

  Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs);
  const Seen* lookup(uint32_t src, uint32_t id, uint32_t nowMs) const;

  Seen seen_[SEEN_SLOTS];
  Rider riders_[MAX_RIDERS];
  Forward forwards_[FORWARD_SLOTS];
  uint32_t lastId_ = 0;
  uint32_t suppressed_ = 0;
};

} // namespace touge
