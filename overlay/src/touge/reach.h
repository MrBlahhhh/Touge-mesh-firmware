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
// A summary on the private port, big-endian:
//   0 magic 0xC3 | 1 version | 2 entries | 3 flags (0x01: the list goes on in
//   the next summary)
//   then 10 bytes an entry:
//   0-3 origin node | 4-5 fix sequence, low 16 bits | 6 fix age on arrival, in
//   250 ms steps (0xFE: 63.5 s or more, 0xFF: unknown) | 7 seconds since heard
//   (0xFF: 255 or more) | 8 hops it had come (0x0F: unknown) | 9 the relay
//   whose copy arrived first, Meshtastic's one-byte relay_node (for 0 hops,
//   the origin's own)
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

static const uint8_t REACH_AGE_OVER = 0xFE;
static const uint8_t REACH_AGE_UNKNOWN = 0xFF;
static const uint32_t REACH_AGE_STEP_MS = 250;
static const uint8_t REACH_HOPS_UNKNOWN = 0x0F;

// An origin not heard for this long is left out of the summary and forgotten.
// Twelve positions at the longest interval (loraload.h).
static const uint32_t REACH_KEEP_MS = 240000;

// Entries in one summary. Sixteen is 164 bytes of payload, about 185 on the
// air against a position's 56: 153 ms on SHORT_FAST to a position's 58. A
// longer list goes on in the next summary.
static const size_t REACH_PER_SUMMARY = 16;

// A summary goes with every twelfth of this car's own positions: once a minute
// at 5 s, every four minutes at 20 s. That adds 16 % (ten origins, 1.9
// positions' airtime) to 22 % (a full one) to the car's own LoRa airtime, and
// the same to its relays. Private port, not a Position field: Meshtastic
// re-encodes a relayed packet from the fields it knows, so there is no spare
// room in a position that relays carry, and a stock app shows the fields there
// are.
static const uint32_t REACH_EVERY_POSITIONS = 12;

// One origin as a summary reports it.
struct ReachEntry {
  uint32_t origin = 0;
  uint16_t seq = 0;
  uint8_t ageQ = REACH_AGE_UNKNOWN;
  uint8_t sinceS = 0;
  uint8_t hops = REACH_HOPS_UNKNOWN;
  uint8_t relay = 0;
};

// Milliseconds to the age byte and back. 0xFF when [ageMs] is UINT32_MAX.
uint8_t reachAgeQ(uint32_t ageMs);
// UINT32_MAX for unknown; REACH_AGE_OVER reads as 63.5 s.
uint32_t reachAgeMs(uint8_t ageQ);

// A summary's entry count and flags. False if [in] is not a summary this
// version reads.
bool decodeReachHeader(const uint8_t* in, size_t len, size_t& entries, uint8_t& flags);
// Entry [i] of a summary; false past the end. Entries are read one at a time,
// with no table to hold a whole summary.
bool decodeReachEntry(const uint8_t* in, size_t len, size_t i, ReachEntry& out);
size_t encodeReach(const ReachEntry* entries, size_t n, uint8_t flags, uint8_t* out, size_t cap);

// For the serial log, one entry: "a1b2#1234 3.4s 1h/5e -12s", the origin's
// last four hex digits (as the app shows cars), the sequence, the age on
// arrival, hops and first relay (relay only past 0 hops), seconds since heard.
size_t formatReachEntry(const ReachEntry& e, char* out, size_t cap);

// One slot per origin: the ride's roster. A full table forgets the origin heard
// longest ago. 16 bytes a slot.
static const size_t REACH_SLOTS = MAX_RIDERS;

class Reach {
 public:
  void clear();

  // A position from [origin] heard over LoRa (only LoRa: this is the lane under
  // test). [ageMs] from the fix's measured time to now, UINT32_MAX unknown;
  // [hops] REACH_HOPS_UNKNOWN when the sender's firmware does not say. A late
  // copy of an older fix is ignored; the same fix again (a parked phone's
  // repeat) still counts as heard.
  void heard(uint32_t origin, const FixId& fix, uint32_t ageMs, uint8_t hops, uint8_t relay, uint32_t nowMs);

  // Heard from [origin], any hops, within [withinMs].
  bool heardWithin(uint32_t origin, uint32_t withinMs, uint32_t nowMs) const;
  // Origins heard within REACH_KEEP_MS.
  size_t count(uint32_t nowMs) const;

  // The next summary into [out]: up to REACH_PER_SUMMARY origins, carrying on
  // where the last one stopped. Forgets origins past REACH_KEEP_MS on the way.
  // 0 when there is nobody to report.
  size_t takeSummary(uint32_t nowMs, uint8_t* out, size_t cap);

  uint32_t summariesSent() const { return sent_; }
  uint32_t summariesHeard() const { return heard_; }
  void noteSummaryHeard() { heard_++; }

 private:
  struct Slot {
    uint32_t origin = 0;  // 0: empty
    uint32_t heardMs = 0;
    uint16_t session = 0;
    uint16_t seq = 0;
    uint8_t ageQ = REACH_AGE_UNKNOWN;
    uint8_t hops = REACH_HOPS_UNKNOWN;
    uint8_t relay = 0;
  };
  bool fresh(const Slot& s, uint32_t nowMs) const {
    return s.origin != 0 && (uint32_t)(nowMs - s.heardMs) < REACH_KEEP_MS;
  }
  Slot slots_[REACH_SLOTS];
  uint8_t cursor_ = 0;
  uint32_t sent_ = 0;
  uint32_t heard_ = 0;
};

}  // namespace touge
