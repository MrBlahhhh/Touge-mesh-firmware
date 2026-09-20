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
static const size_t MAX_RIDERS = 8;

// A packet is remembered long enough to outlive every echo of itself. Three
// hops at SHORT_FAST with back-off is under two seconds, so thirty is a wide
// margin that still forgets a rider who drops out and comes back.
static const uint32_t SEEN_TTL_MS = 30000;
static const size_t SEEN_SLOTS = 64;

// Forwarding a packet the instant it arrives is the wrong thing to do, and it
// gets worse the more cars are on the ride. Three nodes that all hear one
// frame rebroadcast in the same microsecond and collide, so nobody downstream
// gets it. Each forward waits a random slice of this instead.
static const uint32_t FORWARD_JITTER_MS = 15;
static const size_t FORWARD_SLOTS = 8;

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
  uint8_t via = HEARD_NONE;
  int16_t rssi = 0;
  uint8_t hopsAway = 0;
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
              uint32_t nowMs);

  // Drop riders nobody has heard from in a very long time.
  void age(uint32_t nowMs);

  const Rider* riders() const { return riders_; }
  size_t count() const;
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
