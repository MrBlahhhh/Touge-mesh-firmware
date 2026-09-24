#pragma once
//
// Positions to the phone in batches, newest per car (SCALE-PLAN steps 1 and 3).
//
// One MeshPacket per heard position is one BLE read per car per second, and at
// 25 cars that fills Meshtastic's 32-packet phone queue, which then drops the
// newest packet and keeps the stale ones. Instead the module keeps the newest
// unsent position per car here and hands the phone several cars in one
// private-port payload, at most a couple of batches in flight at a time.
//
// Private port payloads are told apart by their first byte:
//   '{'  0x7B  JSON (fl, fs, fq, gf, profiles, invites)
//   'T'  0x54  voice frame (VoicePacket.MAGIC, planned)
//   0xC1       position batch, radio to phone (this file)
//   0xC2       phone hello, phone to its own radio (this file)
//   0xC3       reach summary, radio to radio over LoRa (reach.h)
// None of the three can start UTF-8 text, so no JSON document collides.
//
// Platform-free like mesh.h: time comes in as an argument.

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "mesh.h"
#include "ram.h"

namespace touge {

static const uint8_t BATCH_MAGIC = 0xC1;
// 2 from build 38: records carry the fix identity in place of the frame id.
static const uint8_t BATCH_VERSION = 2;

// Header, big-endian like frame.cpp:
//   0 magic 0xC1 | 1 version | 2 header length | 3 record length | 4 count
//   5 flags | 6-7 batch sequence | 8-11 radio millis when encoded, or from
//   build 35 when handed to the phone (flag BATCH_AGES_AT_DELIVERY)
// A decoder skips header and record bytes it does not know, so a later
// version can append fields without breaking this one.
static const size_t BATCH_HEADER = 12;

// Record, version 2:
//   0-3 node | 4-7 lat e7 | 8-11 lon e7
//   12-23 fix identity: 12-13 session | 14-17 sequence | 18-21 fix seconds | 22-23 fix ms
//   24-25 heading, centidegrees | 26-27 speed, 0.1 km/h
//   28-29 ms since the radio heard it, at most RECORD_EXPIRE_MS | 30 rssi dBm, 0 unknown
//   31 flags: bit 0 phone fix, bits 1-2 lane (0 = 2.4 GHz), bit 3 expired,
//      bits 4-7 hops away
static const size_t BATCH_RECORD = 32;
static const size_t RECORD_AGE_AT = 28;
static const size_t RECORD_FLAGS_AT = 31;

// Header flag (byte 5): the header time is when the phone
// got the batch and every age runs to then, so the phone takes "arrival minus
// age" as the heard time with no clock to estimate. Set by deliverBatch.
static const uint8_t BATCH_AGES_AT_DELIVERY = 0x01;

// Record flag: held too long to be a position; the phone must not use
// it. Set where a record cannot be taken out: a pre-encoded batch.
static const uint8_t RECORD_EXPIRED = 0x08;

// Older than this, a record is dropped rather than delivered. Under the age
// field's 65.5 s range, so an age the phone does get is always exact; well
// past the 1 Hz beacon, so only a stall or a disconnect gets a record here.
static const uint32_t RECORD_EXPIRE_MS = 60000;

// Meshtastic's Data payload ceiling (meshtastic_Constants_DATA_PAYLOAD_LEN).
// The batch rides in one, so this is the most a batch can be whatever the MTU.
static const size_t BATCH_MAX_PAYLOAD = 233;
static const size_t BATCH_MAX_RECORDS = (BATCH_MAX_PAYLOAD - BATCH_HEADER) / BATCH_RECORD;  // 6

static const uint8_t LANE_FAST = 0;
static const uint8_t LANE_LORA = 1;

struct PhoneRecord {
  uint32_t node = 0;
  int32_t lat = 0;
  int32_t lon = 0;
  // Which fix this is, as its sender named it: orders two copies of one car
  // here and on the phone, and tells the phone this fix from its LoRa copy.
  FixId fix;
  uint16_t headingCdeg = 0;
  uint16_t speedDkmh = 0;
  int8_t rssi = 0;
  bool external = false;
  uint8_t lane = LANE_FAST;
  uint8_t hopsAway = 0;
  // Radio millis when this position was heard. Encoded as an age.
  uint32_t heardMs = 0;
  bool expired = false;
};

struct BatchHeader {
  uint8_t version = 0;
  uint8_t count = 0;
  uint16_t seq = 0;
  uint32_t radioMs = 0;
  uint8_t flags = 0;
};

// The largest payload a batch should be, for the MTU the phone negotiated.
//
// A read bigger than MTU - 1 costs Android extra round trips (read blob), so a
// batch is sized to arrive in one ATT response. 48 bytes covers the FromRadio
// and MeshPacket envelope around the payload. Never above BATCH_MAX_PAYLOAD.
// 0 means the phone did not learn its MTU; long reads still work, so full size.
size_t batchBudgetForMtu(uint16_t mtu);

// Records that fit in [capBytes].
size_t recordsThatFit(size_t capBytes);

// Encodes [n] records. Returns bytes written, 0 if they do not all fit.
size_t encodeBatch(const PhoneRecord* records, size_t n, uint16_t seq, uint32_t nowMs, uint8_t* out,
                   size_t cap);

// Decodes a batch. False, with nothing written to [out], on anything malformed
// or truncated: a short batch is never half-published.
bool decodeBatch(const uint8_t* in, size_t len, BatchHeader& header, PhoneRecord* out, size_t outCap);

// Checks a batch's header and length and reads the header only. The radio uses
// it to see which batch the phone just read.
bool decodeBatchHeader(const uint8_t* in, size_t len, BatchHeader& header);

// ---- Newest unsent position per car ----------------------------------------

// One pending record per car heard on 2.4 GHz, so the roster's size is enough;
// a car past it evicts the longest-waiting record. 48 bytes a slot.
#if TOUGE_LEAN_RAM
static const size_t PHONE_STORE_SLOTS = MAX_RIDERS;
#else
static const size_t PHONE_STORE_SLOTS = 32;
#endif

// How long the first record in may wait for others to join it. SCALE-PLAN asks
// for 50-100 ms; the top of that range buys the most cars per read.
static const uint32_t BATCH_FLUSH_MS = 100;

class PhoneStore {
 public:
  enum Offer : uint8_t {
    ADDED,
    REPLACED,  // a pending record for this car was superseded, or heard again
    STALE,     // an older fix than the one pending for this car; kept the pending one
    EVICTED,   // store full: the longest-waiting record was dropped for this one
  };

  void clear();

  Offer offer(const PhoneRecord& r, uint32_t nowMs);

  size_t pending() const;

  // How long the longest-waiting record has been here. 0 when empty.
  uint32_t oldestWaitMs(uint32_t nowMs) const;

  // Time to send: a full batch is waiting, or the oldest has waited [flushMs].
  bool due(uint32_t nowMs, size_t capBytes, uint32_t flushMs) const;

  // Encodes the longest-waiting records that fit and removes them. Returns
  // bytes written, 0 when nothing is pending. [taken] gets the record count.
  // A record older than RECORD_EXPIRE_MS (a car gone quiet during a stall) is
  // dropped instead and counted in expired().
  size_t takeBatch(uint8_t* out, size_t cap, uint16_t seq, uint32_t nowMs, size_t& taken);
  uint32_t expired() const { return expired_; }

 private:
  struct Slot {
    PhoneRecord record;
    // When this car first went pending. A replacement keeps it, so a car
    // updating four times a second cannot hold its own flush off forever.
    uint32_t queuedMs = 0;
    bool used = false;
  };
  Slot slots_[PHONE_STORE_SLOTS];
  uint32_t expired_ = 0;
};

// ---- Batches handed over and not yet read ----------------------------------

// Two in flight: one being read, one ready behind it. More would only queue
// history in front of the phone, which is what the store exists to prevent.
static const size_t MAX_BATCHES_IN_FLIGHT = 2;

// A batch holds its place until it is read or is known to have left the
// queues some other way. There is no timeout: a timed-out batch still sitting
// in Meshtastic's phone queue would let a third in behind it, and a long stall
// would stack them up (review 2026-09-24).
class BatchesInFlight {
 public:
  struct Entry {
    uint16_t seq = 0;
    uint8_t records = 0;
    uint32_t atMs = 0;
    // The MeshPacket id it went out under, to find it in the phone queue.
    uint32_t packetId = 0;
    // In NimBLE's read queue rather than Meshtastic's (core-patches/0007).
    bool preloaded = false;
  };

  void clear();
  bool full() const { return count_ >= MAX_BATCHES_IN_FLIGHT; }
  size_t count() const { return count_; }
  const Entry& at(size_t i) const { return entries_[i]; }
  // False, adding nothing, when full: the caller must check full() first.
  bool add(uint16_t seq, uint8_t records, uint32_t nowMs, uint32_t packetId, bool preloaded);

  // The phone read [seq]. Returns its record count, 0 if it was not in flight.
  uint8_t delivered(uint16_t seq);

  // [seq] left the queues without being read. Returns its record count.
  uint8_t dropped(uint16_t seq);

  // The pre-encoded batch was read, or was lost with the link. Record counts.
  uint8_t preloadRead();
  uint8_t preloadLost();
  bool hasPreloaded() const;

  // Drops every queued (not pre-encoded) batch whose packet is no longer in
  // the phone queue: it left without the delivered hook firing, so it was
  // discarded. [stillQueued] answers for one packet id. Returns records lost.
  uint32_t reconcile(bool (*stillQueued)(uint32_t packetId, void* ctx), void* ctx);

  uint32_t pendingRecords() const;
  uint32_t oldestAgeMs(uint32_t nowMs) const;

 private:
  Entry entries_[MAX_BATCHES_IN_FLIGHT];
  size_t count_ = 0;
  uint8_t removeAt(size_t i);
};

// Readies a batch at the moment the phone gets it. Every age grows by the time
// the batch waited, so one read five seconds late says its positions are five
// seconds older; the header time moves to [nowMs] and BATCH_AGES_AT_DELIVERY
// is set. Records past RECORD_EXPIRE_MS are taken out when [canShrink] (a
// MeshPacket about to be encoded), or flagged RECORD_EXPIRED when not (bytes
// already encoded in NimBLE's read queue). Returns the new length, 0 if
// [payload] is not a batch; [expired] gets how many records expired.
size_t deliverBatch(uint8_t* payload, size_t len, uint32_t nowMs, bool canShrink, uint32_t& expired);

// Where a batch's payload sits inside an encoded FromRadio, so a pre-encoded
// batch can be readied in place when it is read. -1 if it is not there.
int findPayload(const uint8_t* fromRadio, size_t len, const uint8_t* payload, size_t payloadLen);

// ---- Writes the radio dropped ------------------------------------------------

// The packet id inside a ToRadio { packet } write: field 1, then the
// MeshPacket's fixed32 field 6. False for anything else (want_config...).
bool toRadioPacketId(const uint8_t* toRadio, size_t len, uint32_t& id);

// Packet ids of phone writes the radio dropped, from NimBLE's task to the
// main one (core-patches/0007). One producer, one consumer; a full ring drops
// the newest id and counts it, since the aggregate counter still has it.
class DroppedWriteIds {
 public:
  static const size_t SLOTS = 16;
  void push(uint32_t id);
  bool pop(uint32_t& id);
  uint32_t overflowed() const { return overflowed_.load(); }

 private:
  uint32_t ids_[SLOTS] = {0};
  std::atomic<uint32_t> head_{0};  // advanced by the consumer only
  std::atomic<uint32_t> tail_{0};  // advanced by the producer only
  std::atomic<uint32_t> overflowed_{0};
};

// {"wd":[id,...]}, the ids above for the phone. 0 if nothing fits.
size_t formatDroppedWrites(const uint32_t* ids, size_t n, char* out, size_t cap);

// ---- Phone hello ------------------------------------------------------------
//
// The phone tells its radio it is there to read batches, and how. Positions go
// to the phone only in batches, and only after a hello; before one the
// 2.4 GHz lane sends the phone no positions at all (LoRa ones still arrive as
// ordinary Meshtastic packets).
//   0 magic 0xC2 | 1 version | 2 flags | 3-4 ATT MTU, big-endian
// Flag bit 0 is retired (it asked for batches, which are now unconditional).
static const uint8_t HELLO_MAGIC = 0xC2;
static const uint8_t HELLO_VERSION = 1;
static const size_t HELLO_LEN = 5;
// Let the radio answer reads from a pre-encoded batch (core-patches/0007).
static const uint8_t HELLO_PRELOAD = 0x02;

struct PhoneHello {
  uint8_t flags = 0;
  uint16_t mtu = 0;
};

bool decodeHello(const uint8_t* in, size_t len, PhoneHello& out);
size_t encodeHello(const PhoneHello& h, uint8_t* out, size_t cap);

// ---- Step 1 counters ----------------------------------------------------------

struct LaneCounts {
  uint32_t tx = 0;         // frames this board sent (2.4 only)
  uint32_t rx = 0;         // distinct positions heard
  uint32_t suppressed = 0; // forwards not sent because neighbours had it covered
  uint32_t queued = 0;     // positions handed toward the phone
  uint32_t delivered = 0;  // positions the phone read
};

struct LinkStats {
  LaneCounts fast;
  LaneCounts lora;
  uint32_t fastTxFail = 0;
  uint32_t batches = 0;
  uint32_t batchesRead = 0;
  uint32_t replaced = 0;
  // Drops by reason.
  uint32_t dropStoreFull = 0;
  uint32_t dropAlloc = 0;       // packet pool empty
  uint32_t dropLost = 0;        // records in batches that left the queues unread
  uint32_t dropDisconnect = 0;  // records pending or in flight at a disconnect
  uint32_t dropStale = 0;       // older than what was already pending
  uint32_t coreReplaced = 0;    // MeshService queue, newest-wins (patch 0005)
  uint32_t coreDropped = 0;     // MeshService queue full, packet lost (patch 0005)
  uint32_t coreEvicted = 0;     // MeshService queue full, oldest position dropped for room (patch 0005)
  uint32_t dropExpired = 0;     // records too old to deliver, at packing or at delivery
  uint32_t writeDropped = 0;    // phone writes lost after a good BLE write (patch 0007)
  uint32_t writeDuplicate = 0;  // identical consecutive writes discarded (patch 0007)
  uint32_t preloadOffered = 0;
  uint32_t preloadRead = 0;
  uint32_t preloadRefused = 0;
  uint16_t queueDepth = 0;      // MeshService to-phone queue now
  uint16_t queueDepthMax = 0;   // and its high-water mark since the previous report
  uint16_t storePending = 0;
  uint32_t oldestQueuedMs = 0;  // oldest position waiting in our store or in flight
  uint32_t minFreeHeap = 0;
};

// The counters go to the phone every five seconds as three JSON objects with
// named keys, split so each fits one Meshtastic payload with every counter at
// 2^32 - 1:
//   {"fs":{"tx","rx","sp","q","d","lr","ld","b","br","rp","tf"}}
//     2.4 GHz sent, heard, suppressed, queued, delivered; LoRa heard,
//     delivered; batches, batches read, replaced, send failures
//   {"fq":{"qd","qm","pe","ol","hp","po","pr","pf"}}
//     phone queue depth and max, store pending, oldest waiting ms, min free
//     heap; pre-encoded batches offered, read, refused
//   {"fd":{"sf","al","lo","dc","st","cr","cd","ce","ex","wl","wr"}}
//     drops: store full, alloc, lost, disconnect, stale, core replaced, core
//     dropped, core evicted, expired; phone writes lost, repeated
size_t formatLaneStats(const LinkStats& s, char* out, size_t cap);
size_t formatQueueStats(const LinkStats& s, char* out, size_t cap);
size_t formatDropStats(const LinkStats& s, char* out, size_t cap);

// Why the 2.4 GHz lane is not running, for the report the radio sends every
// five seconds whether the lane runs or not. A Touge radio always says its
// build; a stock Meshtastic radio says nothing, which is how the phone tells
// the two apart.
enum class LaneDown : uint8_t {
  STARTING,  // waiting for Bluetooth to settle after boot
  NO_KEY,    // the primary channel has no ride key
  RADIO,     // ESP-NOW would not start (usually no heap left beside BLE)
};

// {"fl":{"fw":build,"up":0,"why":"boot"|"key"|"radio"}}
size_t formatLaneDown(uint32_t build, LaneDown why, char* out, size_t cap);

// The line both ends print every few seconds, as rates over [windowMs] from
// the difference between two snapshots. Grep for "BASELINE".
size_t formatBaseline(const LinkStats& now, const LinkStats& before, uint32_t windowMs, char* out,
                      size_t cap);

}  // namespace touge
