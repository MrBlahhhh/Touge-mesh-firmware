#pragma once
//
// LoRa relays chosen on evidence, with every other car kept as a backup
// (SCALE-PLAN 5f).
//
// Meshtastic floods broadcasts: every car that hears one relays it after a delay
// weighted by signal, weakest first, and drops its own copy on hearing another
// car's (managed flooding). A car the evidence shows useful for an origin now
// relays that origin's positions early instead, in the window Meshtastic gives
// a ROUTER (core-patches/0012), so its copy is the one the others hear and drop
// theirs for. Nobody is switched off: a car that is not preferred keeps its
// ordinary delay and relays whenever the preferred copy has not been heard by
// then. That is the backup, and it is what carries the rear car on when the
// preferred relay drops out.
//
// The evidence is a reach summary (reach.h) from a car further along: it had
// the origin's newest position from a hop or more away, first through our copy,
// fresh. Only a summary grants or renews; hearing another car relay proves
// nothing about delivery. A 2.4 GHz link is never an input, so a good fast lane
// cannot, on its own, turn a LoRa relay off.
//
// Without renewal a preference lapses and the car is back to ordinary
// forwarding. A summary showing worse delivery through us ends it at once.
//
// From build 43 a car also skips a relay nobody needs (relayVerdict): when
// fresh summaries show every other car it knows already hears the origin
// steadily direct. A preference only matters for a relay that goes at all.
//
// Platform-free.

#include <stddef.h>
#include <stdint.h>
#include "lorapos.h"
#include "reach.h"

namespace touge {

// ---- Relays nobody needs (build 43) ------------------------------------------

enum class RelayVerdict : uint8_t {
  STOCK,        // not a Touge position: Meshtastic relays it as ever
  SKIP,         // every other car we know hears the origin steadily direct
  NO_EVIDENCE,  // relayed: a car we know has claimed nothing (a stock node, an older build, a car just heard)
  STALE,        // relayed: a car's last full summary is older than the window
  NEEDED,       // relayed: a car's fresh summary says it does not hear the origin steadily direct
};

// The cars we know other than us and [origin], into [cars] (room for [cap]);
// false when there are more, and nobody can then be shown not to need a relay.
typedef bool (*KnownCars)(uint32_t origin, uint32_t* cars, size_t cap, size_t& n, void* ctx);

// The most cars a relay decision reads: a full ride and a few strangers. On
// the stack of the one relay being judged, 128 bytes.
static const size_t KNOWN_CARS_MAX = MAX_RIDERS + 4;

// Whether a position from [origin] is worth relaying for the [n] cars in
// [known]. SKIP only when every one of them claims to hear the origin steadily
// direct in a summary fresh within [freshMs]; otherwise the strongest reason to
// relay, NEEDED over STALE over NO_EVIDENCE. Nobody else known is nobody to
// relay for: a car that turns up is known from its first packet.
RelayVerdict judgeRelay(const Reach& reach, uint32_t origin, const uint32_t* known, size_t n, uint32_t freshMs,
                        uint32_t nowMs);

// The same for packet [from, id] as Meshtastic is about to relay it. Only a
// position noted with a fix identity (5c) is judged; text, NodeInfo, control,
// summaries and stock positions are STOCK, and [known] is not asked.
RelayVerdict relayVerdict(const TxPositions& noted, uint32_t from, uint32_t id, const Reach& reach, KnownCars known,
                          void* ctx, uint32_t freshMs, uint32_t nowMs);

// Since boot, in "le": how many relays were skipped, and why the rest went.
struct RelaySkips {
  uint32_t skipped = 0;     // sk
  uint32_t noEvidence = 0;  // rn
  uint32_t stale = 0;       // rs
  uint32_t needed = 0;      // rd
  void note(RelayVerdict v);
};

// Origins one car can be a preferred relay for. A car in a convoy carries the
// cars behind it one way and the cars ahead the other; eight covers a long
// ride's worth of either side before the soonest to lapse gives way.
static const size_t PREFER_SLOTS = 8;

// Delivery counts as good while the far car gets the fix no older than this.
// A position leaves within a second or two of its fix (5b); ten seconds is a
// relay path that queues, or that has started to miss.
static const uint32_t PREFER_MAX_AGE_MS = 10000;

class RelayPrefs {
 public:
  enum class Verdict : uint8_t {
    NONE,       // nothing about us
    GRANTED,    // we relay this origin early from now
    RENEWED,
    WITHDRAWN,  // delivery through us got worse: ordinary forwarding
  };

  void clear();

  // One entry of the summary [reporter] sent. [self] and [selfByte] are our
  // node and Meshtastic relay byte. [weHearIt]: we have heard the origin over
  // LoRa lately. [maxSinceS]: how long the reporter may have gone without the
  // origin before its delivery counts as worse (a few of our intervals).
  // [holdMs]: how long a grant lasts without renewal.
  Verdict consider(const ReachEntry& e, uint32_t reporter, uint32_t self, uint8_t selfByte, bool weHearIt,
                   uint32_t maxSinceS, uint32_t holdMs, uint32_t nowMs);

  // Relay [origin]'s positions early.
  bool preferred(uint32_t origin, uint32_t nowMs) const;
  // Origins we are preferred for now.
  size_t count(uint32_t nowMs) const;

  uint32_t granted() const { return granted_; }
  uint32_t withdrawn() const { return withdrawn_; }

 private:
  struct Pref {
    uint32_t origin = 0;  // 0: empty
    uint32_t untilMs = 0;
  };
  bool live(const Pref& p, uint32_t nowMs) const { return p.origin != 0 && (int32_t)(p.untilMs - nowMs) > 0; }
  Pref prefs_[PREFER_SLOTS];
  uint32_t granted_ = 0;
  uint32_t withdrawn_ = 0;
};

// "le", since boot: summaries sent and heard, preferences granted and
// withdrawn, then (build 43) how many of our summaries went early, relays
// skipped and why the rest went. Serial line when [json] is false, the phone's
// JSON when true.
size_t formatLoraReach(const Reach& reach, const RelayPrefs& prefs, const RelaySkips& skips, bool json, char* out,
                       size_t cap);

}  // namespace touge
