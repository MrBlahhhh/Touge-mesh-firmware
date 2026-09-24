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
// Platform-free.

#include <stddef.h>
#include <stdint.h>
#include "reach.h"

namespace touge {

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

// "le": summaries sent and heard, preferences granted and withdrawn, since
// boot. Serial line when [json] is false, the phone's JSON when true.
size_t formatLoraReach(const Reach& reach, const RelayPrefs& prefs, bool json, char* out, size_t cap);

}  // namespace touge
