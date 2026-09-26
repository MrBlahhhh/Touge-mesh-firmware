#pragma once
//
// Which cars' LoRa positions reach this one, and the summaries that tell the
// rest of the convoy (SCALE-PLAN 5e).
//
// Hearing a neighbour relay a packet proves only that it got one hop further,
// which is all Meshtastic's implicit acknowledgement says. So each car keeps,
// per origin, the newest position it heard over LoRa: the fix sequence, how old
// the fix was when it arrived, how many hops it had come and whose relayed copy
// got here first. Every few positions it broadcasts that list, and every car,
// and every phone (Meshtastic hands each one every packet its radio hears),
// learns which origins reach which cars and how old they are when they do.
//
// From build 43 an entry also says whether the car hears that origin steadily
// direct. Every car keeps what the others' summaries claim about that, which is
// the evidence relaypref.h skips relays on, and a car that stops hearing an
// origin it claimed says so at once in an early summary instead of waiting for
// its next one.
//
// A summary on the private port, big-endian:
//   0 magic 0xC3 | 1 version | 2 entries | 3 flags: 0x01 the list goes on in
//   the next summary; 0x02 early, listing only the origins it is about; 0x04
//   the entries carry the steady flag (build 43)
//   then 10 bytes an entry:
//   0-3 origin node | 4-5 fix sequence, low 16 bits | 6 fix age on arrival, in
//   250 ms steps (0xFE: 63.5 s or more, 0xFF: unknown) | 7 seconds since heard
//   (0xFF: 255 or more) | 8 hops it had come in the low nibble (0x0F: unknown),
//   0x10 heard steadily direct | 9 the relay whose copy arrived first,
//   Meshtastic's one-byte relay_node (for 0 hops, the origin's own)
// 0xC3 starts no JSON document and no other private-port payload (phonebatch.h).
//
// Platform-free: the module reads the packets.

#include <stddef.h>
#include <stdint.h>
#include "frame.h"
#include "mesh.h"

namespace touge {

static const uint8_t REACH_MAGIC = 0xC3;
static const uint8_t REACH_VERSION = 1;
static const size_t REACH_HEADER = 4;
static const size_t REACH_ENTRY = 10;
static const uint8_t REACH_MORE = 0x01;
static const uint8_t REACH_EARLY = 0x02;
static const uint8_t REACH_STEADY = 0x04;
static const uint8_t REACH_ENTRY_STEADY = 0x10;

static const uint8_t REACH_AGE_OVER = 0xFE;
static const uint8_t REACH_AGE_UNKNOWN = 0xFF;
static const uint32_t REACH_AGE_STEP_MS = 250;
static const uint8_t REACH_HOPS_UNKNOWN = 0x0F;

// An origin not heard for this long is left out of the summary and forgotten.
// Twelve positions at the longest interval (loraload.h).
static const uint32_t REACH_KEEP_MS = 240000;

// Entries in one summary. Sixteen is 164 bytes of payload; summaries stay
// signed, so that is 254 bytes on the air, 204 ms on SHORT_FAST. A longer list
// goes on in the next summary.
static const size_t REACH_PER_SUMMARY = 16;

// A summary goes with every twelfth of this car's own positions: once a minute
// at 5 s, every four minutes at 20 s. Signed, 99 ms (two origins) to 204 ms (a
// full one) against twelve unsigned positions' 790 ms: 13-26 % on top of the
// car's own LoRa airtime, and the same to its relays. Private port, not a
// Position field: Meshtastic re-encodes a relayed packet from the fields it
// knows, so there is no spare room in a position that relays carry, and a stock
// app shows the fields there are.
static const uint32_t REACH_EVERY_POSITIONS = 12;

// Heard steadily direct: the last four positions of an origin reached us first
// directly, none missed in between (more than one and a half intervals apart),
// the latest within one and a half intervals. Four is 20 s at 5 s; a link that
// drops one position in ten gets there two times in three.
static const uint8_t REACH_STEADY_FIXES = 4;

// Early summaries (build 43), in our own intervals. An origin our last summary
// called steady goes in one when it has been quiet two and a half intervals
// (two positions missed, the second not merely late) or its last two positions
// came first through a relay. It goes a random part of a quarter interval
// later, so cars that lost an origin together do not all send at once, and
// never within an interval of our last early summary; one repeat follows two
// intervals on if the origin stays quiet, in case the first was lost. One to
// three entries, signed, is 91-107 ms on SHORT_FAST.

// One origin as a summary reports it.
struct ReachEntry {
  uint32_t origin = 0;
  uint16_t seq = 0;
  uint8_t ageQ = REACH_AGE_UNKNOWN;
  uint8_t sinceS = 0;
  uint8_t hops = REACH_HOPS_UNKNOWN;
  uint8_t relay = 0;
  bool steady = false;
};

// Milliseconds to the age byte and back. 0xFF when [ageMs] is UINT32_MAX.
uint8_t reachAgeQ(uint32_t ageMs);
// UINT32_MAX for unknown; REACH_AGE_OVER reads as 63.5 s.
uint32_t reachAgeMs(uint8_t ageQ);

// A summary's entry count and flags. False if [in] is not a summary this
// version reads.
bool decodeReachHeader(const uint8_t* in, size_t len, size_t& entries, uint8_t& flags);
// Entry [i] of a summary; false past the end. Entries are read one at a time,
// with no table to hold a whole summary. The steady flag counts only in a
// summary whose header says the entries carry it.
bool decodeReachEntry(const uint8_t* in, size_t len, size_t i, ReachEntry& out);
size_t encodeReach(const ReachEntry* entries, size_t n, uint8_t flags, uint8_t* out, size_t cap);

// For the serial log, one entry: "a1b2#1234 3.4s 1h/5e -12s", the origin's
// last four hex digits (as the app shows cars), the sequence, the age on
// arrival, hops and first relay (relay only past 0 hops; "0h steady" when
// heard steadily direct), seconds since heard.
size_t formatReachEntry(const ReachEntry& e, char* out, size_t cap);

// What one car's summaries say about hearing an origin directly.
enum class DirectClaim : uint8_t {
  STEADY,      // its latest summary listing the origin says it hears it steadily direct
  NOT_STEADY,  // it says it does not, or has never listed the origin
  STALE,       // its last full summary is older than the window
  UNPROVEN,    // nothing from it that says: a stock node, an older build, a car just heard
};

// One slot per origin: the ride's roster. A full table forgets the origin heard
// longest ago. 16 bytes a slot, and 6 more for the claims (build 43). 32, not
// MAX_RIDERS (28): a 30-car ride has 29 other origins, and each arrival evicted
// the car due to send next, so nobody's claims survived and no relay was
// skipped. The extra four slots cost 88 bytes.
static const size_t REACH_SLOTS = 32;
static_assert(REACH_SLOTS >= MAX_RIDERS, "at least every car the roster holds");
static_assert(REACH_SLOTS <= 32, "the claims keep one bit per car in a 32-bit word");

class Reach {
 public:
  void clear();

  // A position from [origin] heard over LoRa (only LoRa: this is the lane under
  // test). [ageMs] from the fix's measured time to now, UINT32_MAX unknown;
  // [hops] REACH_HOPS_UNKNOWN when the sender's firmware does not say. A late
  // copy of an older fix is ignored; the same fix again (a parked phone's
  // repeat) still counts as heard. [intervalMs] is ours, taken as the origin's:
  // it says when a position was missed.
  void heard(uint32_t origin, const FixId& fix, uint32_t ageMs, uint8_t hops, uint8_t relay, uint32_t intervalMs,
             uint32_t nowMs);

  // Heard from [origin], any hops, within [withinMs].
  bool heardWithin(uint32_t origin, uint32_t withinMs, uint32_t nowMs) const;
  // Origins heard within REACH_KEEP_MS.
  size_t count(uint32_t nowMs) const;

  // The next summary into [out]: up to REACH_PER_SUMMARY origins, carrying on
  // where the last one stopped, each with whether we hear it steadily direct.
  // Forgets origins past REACH_KEEP_MS on the way. 0 when there is nobody to
  // report.
  size_t takeSummary(uint32_t nowMs, uint32_t intervalMs, uint8_t* out, size_t cap);

  // Whether an early summary is due now; asked every pass while we send
  // positions. [random] places it in its quarter interval.
  bool earlyDue(uint32_t nowMs, uint32_t intervalMs, uint32_t random);
  // The early summary into [out]: the origins it is about. 0 if none.
  size_t takeEarlySummary(uint32_t nowMs, uint32_t intervalMs, uint8_t* out, size_t cap);

  // A summary from [reporter], heard over LoRa: keeps what it claims about
  // hearing each origin steadily direct. Kept only for a reporter in the table,
  // whose slot holds its claims. An early summary changes the claims it lists
  // and leaves the rest to age from the last full one. False if [payload] is
  // not a summary.
  bool noteSummary(uint32_t reporter, const uint8_t* payload, size_t len, uint32_t nowMs);
  // What [car] claims about hearing [origin] directly, in a summary fresh
  // within [freshMs].
  DirectClaim claim(uint32_t car, uint32_t origin, uint32_t freshMs, uint32_t nowMs) const;

  uint32_t summariesSent() const { return sent_; }  // early ones included
  uint32_t summariesHeard() const { return heard_; }
  uint32_t earlySent() const { return earlySent_; }

 private:
  struct Slot {
    uint32_t origin = 0;  // 0: empty
    uint32_t heardMs = 0;
    uint16_t session = 0;
    uint16_t seq = 0;
    uint8_t ageQ = REACH_AGE_UNKNOWN;
    uint8_t hops = REACH_HOPS_UNKNOWN;
    uint8_t relay = 0;
    uint8_t state = 0;  // the bits below, in what was padding
  };
  // Slot::state.
  static const uint8_t STREAK = 0x07;          // positions in a row first heard directly, none missed; up to 7
  static const uint8_t PREV_RELAYED = 0x08;    // the last one came first through a relay
  static const uint8_t CLAIMED_STEADY = 0x10;  // what our last summary listing it said
  static const uint8_t EARLY_PENDING = 0x20;   // goes in the next early summary
  static const uint8_t EARLY_SENT = 0x40;      // an early summary said it lost it: one repeat if it stays quiet

  bool fresh(const Slot& s, uint32_t nowMs) const {
    return s.origin != 0 && (uint32_t)(nowMs - s.heardMs) < REACH_KEEP_MS;
  }
  // The slot of [origin] heard within REACH_KEEP_MS; REACH_SLOTS if none.
  size_t freshIndex(uint32_t origin, uint32_t nowMs) const;
  bool steady(const Slot& s, uint32_t intervalMs, uint32_t nowMs) const;
  ReachEntry entryOf(const Slot& s, uint32_t intervalMs, uint32_t nowMs) const;
  // Empties slot [i], with every claim about its origin and every claim it made.
  void forget(size_t i);

  Slot slots_[REACH_SLOTS];
  // The other cars' claims (build 43): bit c of claimedBy_[o] is set when the
  // car in slot c said, in its latest summary listing slot o's origin, that it
  // hears it steadily direct. summaryAt_[c] is when that car's latest full
  // summary came in, in 256 ms ticks of the radio's clock, low 16 bits (4.7
  // hours round; 0 none). 192 bytes.
  uint32_t claimedBy_[REACH_SLOTS] = {0};
  uint16_t summaryAt_[REACH_SLOTS] = {0};
  uint32_t lastEarlyMs_ = 0;  // our last early summary, when hasLastEarly_
  uint32_t earlyAtMs_ = 0;    // when the pending one goes, when earlyScheduled_
  bool hasLastEarly_ = false;
  bool earlyScheduled_ = false;
  uint8_t cursor_ = 0;
  uint32_t sent_ = 0;
  uint32_t heard_ = 0;
  uint32_t earlySent_ = 0;
};

}  // namespace touge
