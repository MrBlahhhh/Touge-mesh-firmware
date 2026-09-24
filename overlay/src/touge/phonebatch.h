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
// 0xC1 and 0xC2 can never start UTF-8 text, so no JSON document collides.
//
// Platform-free like mesh.h: time comes in as an argument.

#include <stddef.h>
#include <stdint.h>

namespace touge {

static const uint8_t BATCH_MAGIC = 0xC1;
static const uint8_t BATCH_VERSION = 1;

// Header, version 1, big-endian like frame.cpp:
//   0 magic 0xC1 | 1 version | 2 header length | 3 record length | 4 count
//   5 reserved 0 | 6-7 batch sequence | 8-11 radio millis when encoded
// A decoder skips header and record bytes it does not know, so a later
// version can append fields without breaking this one.
static const size_t BATCH_HEADER = 12;

// Record, version 1:
//   0-3 node | 4-7 lat e7 | 8-11 lon e7 | 12-15 sender frame id
//   16-17 heading, centidegrees | 18-19 speed, 0.1 km/h
//   20-21 ms since the radio heard it (saturates) | 22 rssi dBm, 0 unknown
//   23 flags: bit 0 phone fix, bits 1-2 lane (0 = 2.4 GHz), bits 4-7 hops away
static const size_t BATCH_RECORD = 24;

// The largest age a record carries. 0xFFFF itself is left alone: build 30 radios
// wrote it for a slightly negative age, and the app reads it as "just heard".
static const uint16_t AGE_MAX = 0xFFFE;

// Meshtastic's Data payload ceiling (meshtastic_Constants_DATA_PAYLOAD_LEN).
// The batch rides in one, so this is the most a batch can be whatever the MTU.
static const size_t BATCH_MAX_PAYLOAD = 233;
static const size_t BATCH_MAX_RECORDS = (BATCH_MAX_PAYLOAD - BATCH_HEADER) / BATCH_RECORD;  // 9

static const uint8_t LANE_FAST = 0;
static const uint8_t LANE_LORA = 1;

struct PhoneRecord {
  uint32_t node = 0;
  int32_t lat = 0;
  int32_t lon = 0;
  // The sender's frame id. It counts up per sender, so it orders two copies of
  // one car and shows gaps in what reached the phone.
  uint32_t frameId = 0;
  uint16_t headingCdeg = 0;
  uint16_t speedDkmh = 0;
  int8_t rssi = 0;
  bool external = false;
  uint8_t lane = LANE_FAST;
  uint8_t hopsAway = 0;
  // Radio millis when this position was heard. Encoded as an age.
  uint32_t heardMs = 0;
};

struct BatchHeader {
  uint8_t version = 0;
  uint8_t count = 0;
  uint16_t seq = 0;
  uint32_t radioMs = 0;
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

static const size_t PHONE_STORE_SLOTS = 32;

// How long the first record in may wait for others to join it. SCALE-PLAN asks
// for 50-100 ms; the top of that range buys the most cars per read.
static const uint32_t BATCH_FLUSH_MS = 100;

class PhoneStore {
 public:
  enum Offer : uint8_t {
    ADDED,
    REPLACED,  // a pending record for this car was superseded
    STALE,     // older than what is pending for this car; kept the pending one
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
  size_t takeBatch(uint8_t* out, size_t cap, uint16_t seq, uint32_t nowMs, size_t& taken);

 private:
  struct Slot {
    PhoneRecord record;
    // When this car first went pending. A replacement keeps it, so a car
    // updating four times a second cannot hold its own flush off forever.
    uint32_t queuedMs = 0;
    bool used = false;
  };
  Slot slots_[PHONE_STORE_SLOTS];
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

// Moves a batch's clock to [nowMs], the moment it is handed to the phone:
// every record's age grows by the time the batch waited in the queue, so a
// batch read five seconds late says its positions are five seconds older.
// False, changing nothing, if [payload] is not a batch.
bool restampBatch(uint8_t* payload, size_t len, uint32_t nowMs);

// ---- Phone hello ------------------------------------------------------------
//
// The phone tells its radio it reads batches. Until it does, positions go out
// one packet each as in build 29, so an older app keeps working on this build.
//   0 magic 0xC2 | 1 version | 2 flags | 3-4 ATT MTU, big-endian
static const uint8_t HELLO_MAGIC = 0xC2;
static const uint8_t HELLO_VERSION = 1;
static const size_t HELLO_LEN = 5;
static const uint8_t HELLO_BATCHES = 0x01;
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
  uint32_t dropLost = 0;        // records in batches never read (expired)
  uint32_t dropDisconnect = 0;  // records pending or in flight at a disconnect
  uint32_t dropStale = 0;       // older than what was already pending
  uint32_t coreReplaced = 0;    // MeshService queue, newest-wins (patch 0005)
  uint32_t coreDropped = 0;     // MeshService queue full, packet lost (patch 0005)
  uint32_t coreEvicted = 0;     // MeshService queue full, oldest position dropped for room (patch 0005)
  uint32_t writeDropped = 0;    // phone writes lost after a good BLE write (patch 0007)
  uint32_t writeDuplicate = 0;  // identical consecutive writes discarded (patch 0007)
  uint32_t preloadOffered = 0;
  uint32_t preloadRead = 0;
  uint32_t preloadRefused = 0;
  uint16_t queueDepth = 0;      // MeshService to-phone queue now
  uint16_t queueDepthMax = 0;   // and its high-water mark
  uint16_t storePending = 0;
  uint32_t oldestQueuedMs = 0;  // oldest position waiting in our store or in flight
  uint32_t minFreeHeap = 0;
};

// {"fs":[1, fast tx, rx, suppressed, queued, delivered, lora rx, lora delivered,
//        batches, batches read, replaced, fast tx fail]}
// Positional, so the worst case (every counter at 2^32 - 1) still fits one
// Meshtastic payload. New fields are appended; the first element is the version.
size_t formatLaneStats(const LinkStats& s, char* out, size_t cap);

// {"fq":[1, queue depth, depth max, store pending, oldest queued ms,
//        drop store full, alloc, lost, disconnect, stale, core replaced,
//        core dropped, write dropped, write duplicate, preload offered,
//        preload read, preload refused, min free heap, core evicted]}
// Depth max is the high-water mark since the previous report.
size_t formatQueueStats(const LinkStats& s, char* out, size_t cap);

// The line both ends print every few seconds, as rates over [windowMs] from
// the difference between two snapshots. Grep for "BASELINE".
size_t formatBaseline(const LinkStats& now, const LinkStats& before, uint32_t windowMs, char* out,
                      size_t cap);

}  // namespace touge
