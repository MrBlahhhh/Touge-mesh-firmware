#include "reach.h"

#include <stdio.h>

namespace touge {
namespace {

void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)v;
}
void put32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}
uint16_t get16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }
uint32_t get32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// The claims' clock: 256 ms ticks of the radio's millis, low 16 bits. 0 is
// kept for "none".
uint16_t tickOf(uint32_t nowMs) {
  const uint16_t t = (uint16_t)(nowMs >> 8);
  return t == 0 ? 1 : t;
}
uint32_t msSinceTick(uint16_t tick, uint32_t nowMs) { return (uint32_t)(uint16_t)(tickOf(nowMs) - tick) << 8; }

// A car still sending positions but no summaries (a radio back in stock mode)
// loses its stamp long before 4.7 hours can wrap it round to look fresh. The
// longest window a claim is read in is ten minutes (20 s positions).
const uint32_t SUMMARY_STAMP_MAX_MS = 900000;

}  // namespace

uint8_t reachAgeQ(uint32_t ageMs) {
  if (ageMs == UINT32_MAX) return REACH_AGE_UNKNOWN;
  const uint32_t steps = ageMs / REACH_AGE_STEP_MS;
  return steps >= REACH_AGE_OVER ? REACH_AGE_OVER : (uint8_t)steps;
}

uint32_t reachAgeMs(uint8_t ageQ) {
  if (ageQ == REACH_AGE_UNKNOWN) return UINT32_MAX;
  return (uint32_t)ageQ * REACH_AGE_STEP_MS;
}

bool decodeReachHeader(const uint8_t* in, size_t len, size_t& entries, uint8_t& flags) {
  if (in == nullptr || len < REACH_HEADER || in[0] != REACH_MAGIC || in[1] != REACH_VERSION) return false;
  entries = in[2];
  flags = in[3];
  return len >= REACH_HEADER + entries * REACH_ENTRY;
}

bool decodeReachEntry(const uint8_t* in, size_t len, size_t i, ReachEntry& out) {
  size_t entries = 0;
  uint8_t flags = 0;
  if (!decodeReachHeader(in, len, entries, flags) || i >= entries) return false;
  const uint8_t* e = in + REACH_HEADER + i * REACH_ENTRY;
  out.origin = get32(e);
  out.seq = get16(e + 4);
  out.ageQ = e[6];
  out.sinceS = e[7];
  out.hops = (uint8_t)(e[8] & 0x0F);
  out.steady = (flags & REACH_STEADY) != 0 && (e[8] & REACH_ENTRY_STEADY) != 0;
  out.relay = e[9];
  return true;
}

size_t encodeReach(const ReachEntry* entries, size_t n, uint8_t flags, uint8_t* out, size_t cap) {
  if (out == nullptr || n > 255) return 0;
  const size_t len = REACH_HEADER + n * REACH_ENTRY;
  if (cap < len) return 0;
  out[0] = REACH_MAGIC;
  out[1] = REACH_VERSION;
  out[2] = (uint8_t)n;
  out[3] = flags;
  for (size_t i = 0; i < n; i++) {
    uint8_t* e = out + REACH_HEADER + i * REACH_ENTRY;
    put32(e, entries[i].origin);
    put16(e + 4, entries[i].seq);
    e[6] = entries[i].ageQ;
    e[7] = entries[i].sinceS;
    e[8] = (uint8_t)((entries[i].hops & 0x0F) | (entries[i].steady ? REACH_ENTRY_STEADY : 0));
    e[9] = entries[i].relay;
  }
  return len;
}

size_t formatReachEntry(const ReachEntry& e, char* out, size_t cap) {
  if (out == nullptr || cap == 0) return 0;
  char age[24];
  if (e.ageQ == REACH_AGE_UNKNOWN) {
    snprintf(age, sizeof(age), "?");
  } else {
    const uint32_t tenths = reachAgeMs(e.ageQ) / 100;
    snprintf(age, sizeof(age), "%lu.%lus", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
  }
  char path[16];
  if (e.hops == REACH_HOPS_UNKNOWN) {
    snprintf(path, sizeof(path), "?h/%02x", (unsigned)e.relay);
  } else if (e.hops == 0) {
    snprintf(path, sizeof(path), e.steady ? "0h steady" : "0h");
  } else {
    snprintf(path, sizeof(path), "%uh/%02x", (unsigned)e.hops, (unsigned)e.relay);
  }
  const int n = snprintf(out, cap, "%04lx#%u %s %s -%us", (unsigned long)(e.origin & 0xFFFF), (unsigned)e.seq, age,
                         path, (unsigned)e.sinceS);
  if (n < 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

void Reach::clear() { *this = Reach(); }

size_t Reach::freshIndex(uint32_t origin, uint32_t nowMs) const {
  if (origin == 0) return REACH_SLOTS;
  for (size_t i = 0; i < REACH_SLOTS; i++) {
    if (slots_[i].origin == origin) return fresh(slots_[i], nowMs) ? i : REACH_SLOTS;
  }
  return REACH_SLOTS;
}

void Reach::forget(size_t i) {
  const uint32_t bit = 1u << i;
  for (size_t k = 0; k < REACH_SLOTS; k++) claimedBy_[k] &= ~bit;
  claimedBy_[i] = 0;
  summaryAt_[i] = 0;
  slots_[i] = Slot();
}

void Reach::heard(uint32_t origin, const FixId& fix, uint32_t ageMs, uint8_t hops, uint8_t relay, uint32_t intervalMs,
                  uint32_t nowMs) {
  if (origin == 0) return;
  size_t at = REACH_SLOTS;
  for (size_t i = 0; i < REACH_SLOTS && at == REACH_SLOTS; i++) {
    if (slots_[i].origin == origin) at = i;
  }
  if (at < REACH_SLOTS && fresh(slots_[at], nowMs)) {
    const Slot& held = slots_[at];
    if (held.session != 0 && held.session == fix.session) {
      // Sixteen bits of sequence, compared across the wrap: a fix a second takes
      // nine hours to reach the half-range.
      if ((int16_t)((uint16_t)fix.seq - held.seq) < 0) return;
    }
    if (summaryAt_[at] != 0 && msSinceTick(summaryAt_[at], nowMs) > SUMMARY_STAMP_MAX_MS) summaryAt_[at] = 0;
  } else {
    if (at == REACH_SLOTS) {
      for (size_t i = 0; i < REACH_SLOTS && at == REACH_SLOTS; i++) {
        if (!fresh(slots_[i], nowMs)) at = i;
      }
    }
    if (at == REACH_SLOTS) {
      at = 0;
      for (size_t i = 1; i < REACH_SLOTS; i++) {
        if ((uint32_t)(nowMs - slots_[i].heardMs) > (uint32_t)(nowMs - slots_[at].heardMs)) at = i;
      }
    }
    // A new origin, or one back after REACH_KEEP_MS: nothing said about the old
    // one carries over.
    forget(at);
  }

  Slot& s = slots_[at];
  const bool direct = hops == 0;
  const bool missed = s.origin != 0 && (uint32_t)(nowMs - s.heardMs) > intervalMs * 3 / 2;
  uint8_t streak = s.state & STREAK;
  if (!direct) {
    streak = 0;
  } else if (s.origin == 0 || missed) {
    streak = 1;
  } else if (streak < STREAK) {
    streak++;
  }
  // Heard at all: an early summary about losing it needs no repeat.
  uint8_t state = (uint8_t)((s.state & ~(STREAK | EARLY_SENT)) | streak);
  if (direct) {
    state &= (uint8_t)~(PREV_RELAYED | EARLY_PENDING);
  } else {
    // Arriving through relays now, though we claimed it steady.
    if ((state & CLAIMED_STEADY) && (state & PREV_RELAYED)) state |= EARLY_PENDING;
    state |= PREV_RELAYED;
  }

  s.origin = origin;
  s.heardMs = nowMs;
  s.session = fix.session;
  s.seq = (uint16_t)fix.seq;
  s.ageQ = reachAgeQ(ageMs);
  s.hops = hops > REACH_HOPS_UNKNOWN ? REACH_HOPS_UNKNOWN : hops;
  s.relay = relay;
  s.state = state;
}

bool Reach::heardWithin(uint32_t origin, uint32_t withinMs, uint32_t nowMs) const {
  for (size_t i = 0; i < REACH_SLOTS; i++) {
    if (slots_[i].origin == origin && fresh(slots_[i], nowMs)) return (uint32_t)(nowMs - slots_[i].heardMs) < withinMs;
  }
  return false;
}

size_t Reach::count(uint32_t nowMs) const {
  size_t n = 0;
  for (size_t i = 0; i < REACH_SLOTS; i++) n += fresh(slots_[i], nowMs) ? 1 : 0;
  return n;
}

bool Reach::steady(const Slot& s, uint32_t intervalMs, uint32_t nowMs) const {
  // Not the fix's age: that is how stale the origin's GPS is, which a relay
  // cannot improve (build 44; indoors every fix arrived 20-30 s old and no
  // relay was ever skipped).
  return (s.state & STREAK) >= REACH_STEADY_FIXES && s.hops == 0 &&
         (uint32_t)(nowMs - s.heardMs) <= intervalMs * 3 / 2;
}

ReachEntry Reach::entryOf(const Slot& s, uint32_t intervalMs, uint32_t nowMs) const {
  const uint32_t sinceS = (nowMs - s.heardMs) / 1000;
  ReachEntry e;
  e.origin = s.origin;
  e.seq = s.seq;
  e.ageQ = s.ageQ;
  e.sinceS = sinceS >= 255 ? 255 : (uint8_t)sinceS;
  e.hops = s.hops;
  e.relay = s.relay;
  e.steady = steady(s, intervalMs, nowMs);
  return e;
}

size_t Reach::takeSummary(uint32_t nowMs, uint32_t intervalMs, uint8_t* out, size_t cap) {
  for (size_t k = 0; k < REACH_SLOTS; k++) {
    if (slots_[k].origin != 0 && !fresh(slots_[k], nowMs)) forget(k);
  }
  // One pass over the table from where the last summary stopped to the end;
  // the one after a pass starts again from the top.
  ReachEntry entries[REACH_PER_SUMMARY];
  size_t listed[REACH_PER_SUMMARY];
  size_t n = 0;
  size_t i = cursor_ < REACH_SLOTS ? cursor_ : 0;
  for (; i < REACH_SLOTS && n < REACH_PER_SUMMARY; i++) {
    if (slots_[i].origin == 0) continue;
    listed[n] = i;
    entries[n++] = entryOf(slots_[i], intervalMs, nowMs);
  }
  if (n == 0) {
    // The rest of the pass went quiet; start again from the top.
    if (cursor_ == 0) return 0;
    cursor_ = 0;
    return takeSummary(nowMs, intervalMs, out, cap);
  }
  bool more = false;
  for (size_t k = i; k < REACH_SLOTS && !more; k++) more = slots_[k].origin != 0;
  const size_t len = encodeReach(entries, n, (uint8_t)(REACH_STEADY | (more ? REACH_MORE : 0)), out, cap);
  if (len == 0) return 0;
  cursor_ = more ? (uint8_t)i : 0;
  // What this says stands for these origins until the next: nothing early is
  // left to say about them.
  for (size_t k = 0; k < n; k++) {
    Slot& s = slots_[listed[k]];
    s.state = (uint8_t)((s.state & ~(CLAIMED_STEADY | EARLY_PENDING | EARLY_SENT)) |
                        (entries[k].steady ? CLAIMED_STEADY : 0));
  }
  sent_++;
  return len;
}

bool Reach::earlyDue(uint32_t nowMs, uint32_t intervalMs, uint32_t random) {
  const uint32_t quietMs = intervalMs * 5 / 2;
  bool pending = false;
  for (size_t i = 0; i < REACH_SLOTS; i++) {
    Slot& s = slots_[i];
    if (!fresh(s, nowMs)) continue;
    if ((s.state & CLAIMED_STEADY) && (uint32_t)(nowMs - s.heardMs) > quietMs) s.state |= EARLY_PENDING;
    // Not heard since our early summary said we lost it: once more, in case
    // that one was lost.
    if ((s.state & EARLY_SENT) && (uint32_t)(nowMs - lastEarlyMs_) >= 2 * intervalMs) s.state |= EARLY_PENDING;
    if (s.state & EARLY_PENDING) pending = true;
  }
  if (!pending) {
    earlyScheduled_ = false;
    return false;
  }
  if (!earlyScheduled_) {
    uint32_t at = nowMs + random % (intervalMs / 4 + 1);
    if (hasLastEarly_ && (int32_t)(lastEarlyMs_ + intervalMs - at) > 0) at = lastEarlyMs_ + intervalMs;
    earlyAtMs_ = at;
    earlyScheduled_ = true;
  }
  return (int32_t)(nowMs - earlyAtMs_) >= 0;
}

size_t Reach::takeEarlySummary(uint32_t nowMs, uint32_t intervalMs, uint8_t* out, size_t cap) {
  earlyScheduled_ = false;
  ReachEntry entries[REACH_PER_SUMMARY];
  size_t listed[REACH_PER_SUMMARY];
  size_t n = 0;
  for (size_t i = 0; i < REACH_SLOTS && n < REACH_PER_SUMMARY; i++) {
    if (!fresh(slots_[i], nowMs) || !(slots_[i].state & EARLY_PENDING)) continue;
    listed[n] = i;
    entries[n++] = entryOf(slots_[i], intervalMs, nowMs);
  }
  if (n == 0) return 0;
  const size_t len = encodeReach(entries, n, REACH_EARLY | REACH_STEADY, out, cap);
  if (len == 0) return 0;
  for (size_t k = 0; k < n; k++) {
    Slot& s = slots_[listed[k]];
    // The first about a loss is followed by one repeat; the repeat by none.
    const uint8_t sent = (s.state & EARLY_SENT) ? 0 : EARLY_SENT;
    s.state = (uint8_t)((s.state & ~(CLAIMED_STEADY | EARLY_PENDING | EARLY_SENT)) |
                        (entries[k].steady ? CLAIMED_STEADY : 0) | sent);
  }
  lastEarlyMs_ = nowMs;
  hasLastEarly_ = true;
  sent_++;
  earlySent_++;
  return len;
}

bool Reach::noteSummary(uint32_t reporter, const uint8_t* payload, size_t len, uint32_t nowMs) {
  size_t entries = 0;
  uint8_t flags = 0;
  if (!decodeReachHeader(payload, len, entries, flags)) return false;
  heard_++;
  // A build 41 summary says nothing about hearing directly.
  if ((flags & REACH_STEADY) == 0) return true;
  const size_t r = freshIndex(reporter, nowMs);
  if (r >= REACH_SLOTS) return true;
  if ((flags & REACH_EARLY) == 0) summaryAt_[r] = tickOf(nowMs);
  const uint32_t bit = 1u << r;
  ReachEntry e;
  for (size_t i = 0; decodeReachEntry(payload, len, i, e); i++) {
    const size_t o = freshIndex(e.origin, nowMs);
    if (o >= REACH_SLOTS || o == r) continue;
    if (e.steady && e.hops == 0) {
      claimedBy_[o] |= bit;
    } else {
      claimedBy_[o] &= ~bit;
    }
  }
  return true;
}

DirectClaim Reach::claim(uint32_t car, uint32_t origin, uint32_t freshMs, uint32_t nowMs) const {
  const size_t c = freshIndex(car, nowMs);
  if (c >= REACH_SLOTS || summaryAt_[c] == 0) return DirectClaim::UNPROVEN;
  if (msSinceTick(summaryAt_[c], nowMs) > freshMs) return DirectClaim::STALE;
  const size_t o = freshIndex(origin, nowMs);
  if (o >= REACH_SLOTS) return DirectClaim::NOT_STEADY;
  return (claimedBy_[o] & (1u << c)) != 0 ? DirectClaim::STEADY : DirectClaim::NOT_STEADY;
}

}  // namespace touge
