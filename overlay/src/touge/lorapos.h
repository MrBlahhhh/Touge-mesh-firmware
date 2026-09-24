#pragma once
//
// LoRa positions: the identity they carry, and how the queues keep one per car
// (SCALE-PLAN 5b and 5c); how ours goes out (build 43).
//
// The radio sends its car's LoRa position itself, from the fix it holds
// (ownfix.h), at the interval the measured load allows (loraload.h).
// Meshtastic's two queues that hold other cars' positions, the one to the air
// and the one to the phone, keep only the newest position per origin, judged by
// fix identity (rankFix), so a backlog carries each car's latest fix rather
// than its history.
//
// Platform-free: the module and the core patches read the packets and hand
// over plain numbers.

#include <stddef.h>
#include <stdint.h>
#include "frame.h"

namespace touge {

// ---- Our own position on the air (build 43) ----------------------------------

// Meshtastic's MeshPacket priorities (mesh.proto; the module asserts they
// match). A relay queues at DEFAULT, and the TX queue sends the highest first.
static const uint8_t PRIORITY_BACKGROUND = 10;
static const uint8_t PRIORITY_DEFAULT = 64;
static const uint8_t PRIORITY_RELIABLE = 70;

// Our position goes one step above relays, and on its own contention delay
// rather than what is left of the SNR-weighted one a relay started, up to
// 2.2 s on SHORT_FAST (core-patches/0014). At BACKGROUND, as PositionModule's,
// it waited behind every relay: bench, build 41, median 0.6 s, p90 4.5 s, max
// 6.2 s from queued to on the air. Priority is local, in neither the LoRa header
// nor the encrypted Data. One position an interval, so relays still go between
// ours; texts, admin and acks stay above it.
static const uint8_t OWN_POSITION_PRIORITY = PRIORITY_RELIABLE;

// Meshtastic's POSITION_APP port (the module asserts it matches).
static const uint32_t PORT_POSITION = 3;

// Whether a packet this radio originates goes without Meshtastic's XEdDSA
// signature (core-patches/0015): our position, broadcast on a channel with a
// real key while the fast lane sends it ([ownsPositions]). The signature field
// is 66 bytes: signed, a position is 130-133 bytes on the air, 112-114 ms on
// SHORT_FAST (111 ms measured on the bench); unsigned, 64-67 bytes and 63-66
// ms. Texts, NodeInfo, reach summaries, admin and routing stay signed, and so
// does everything from a radio in stock mode, or a licensed (ham) one, where
// the signature is all that authenticates a packet.
bool sendsUnsigned(uint32_t portnum, bool broadcast, bool ownsPositions, bool privateChannel, bool licensed);

// Whether an unsigned packet from a node Meshtastic has seen sign passes its
// Balanced policy anyway (core-patches/0015): a position on a channel with a
// real key, which only the ride's radios can send. Everything else keeps the
// downgrade protection, and a licensed radio keeps it all.
bool passesUnsigned(uint32_t portnum, bool privateChannel, bool licensed);

// The identity a Meshtastic Position carries: sensor_id is the session,
// seq_number the sequence, and the fix time is timestamp plus its millisecond
// adjustment, else time (setMeasured in ownfix.h). A position from a stock
// sender comes out with session 0.
FixId fixIdOf(uint32_t sensorId, uint32_t seqNumber, uint32_t timestamp, int32_t millisAdjust, uint32_t time);

// ---- The newest position per origin in a queue (5c) --------------------------

// How [incoming] stands against [queued], two positions from one car waiting
// in a queue. Unless both carry an identity, the later arrival counts as
// newer, which is what the queues did before.
FixRank rankQueued(const FixId& incoming, const FixId& queued);

// A packet in Meshtastic's TX queue, as far as the choice below needs it.
struct QueuedPacket {
  uint32_t from = 0;
  uint32_t id = 0;
  uint8_t hopLimit = 0;
};

enum class TxPlace : uint8_t {
  QUEUE,    // not a position with a queued one from its car: the queue's own rules
  REPLACE,  // takes the place, and the turn, of its car's queued older position
  REFUSE,   // its car's queued position is newer, or the same fix with as many hops left
};

// Which positions are in the TX queue, and whose.
//
// The queue holds packets already encrypted, so it cannot read a portnum or a
// fix. The module notes each position with an identity as it goes to the
// router decoded (its own, and every one it may relay), keyed by origin and
// packet id, which the queue can read in the clear.
//
// Fairness comes from REPLACE keeping the older one's place: each car has at
// most one position queued and it keeps its turn however often the car sends,
// so the queue goes round the cars and a busy car next to us cannot push a
// relayed position from the rear car further back.
class TxPositions {
 public:
  // The TX queue's 16 places (MAX_TX_QUEUE) and the position on its way in.
  static const size_t TX_QUEUE_LEN = 16;
  static const size_t SLOTS = TX_QUEUE_LEN + 1;

  // Whether a packet is still in the TX queue.
  typedef bool (*InQueue)(uint32_t from, uint32_t id, void* ctx);

  void clear();

  // A position on its way to the TX queue at [nowMs]. One with no identity (a
  // stock sender's) is not noted, and the queue treats it the stock way. A full
  // table reuses the place of a packet no longer queued; with every place
  // queued the note is dropped, which leaves that position to the stock rules.
  void note(uint32_t from, uint32_t id, const FixId& fix, uint32_t nowMs, InQueue inQueue, void* ctx);

  // The identity noted for a packet, or null.
  const FixId* find(uint32_t from, uint32_t id) const;
  // How long since a packet was noted, which for one leaving the queue is its
  // wait there (5d). False if it was not noted.
  bool waited(uint32_t from, uint32_t id, uint32_t nowMs, uint32_t& waitedMs) const;

  // Where [incoming] goes in a queue of [count] packets, [at](i) giving the
  // i-th. On REPLACE, [index] is the one it replaces.
  template <typename At>
  TxPlace place(const QueuedPacket& incoming, size_t count, At at, size_t& index) const {
    const FixId* fix = find(incoming.from, incoming.id);
    if (fix == nullptr) return TxPlace::QUEUE;
    for (size_t i = 0; i < count; i++) {
      const QueuedPacket queued = at(i);
      if (queued.from != incoming.from || queued.id == incoming.id) continue;
      const FixId* held = find(queued.from, queued.id);
      if (held == nullptr) continue;
      index = i;
      return placeAgainst(*fix, incoming.hopLimit, *held, queued.hopLimit);
    }
    return TxPlace::QUEUE;
  }

 private:
  // One origin has at most one position queued, so the first found decides.
  static TxPlace placeAgainst(const FixId& incoming, uint8_t incomingHops, const FixId& held, uint8_t heldHops);

  struct Tag {
    uint32_t from = 0;
    uint32_t id = 0;
    FixId fix;  // session 0: an empty place
    uint32_t notedMs = 0;
  };
  Tag tags_[SLOTS];
};

}  // namespace touge
