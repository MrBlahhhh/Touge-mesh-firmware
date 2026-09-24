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
    e[8] = (uint8_t)(entries[i].hops & 0x0F);
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
    snprintf(path, sizeof(path), "0h");
  } else {
    snprintf(path, sizeof(path), "%uh/%02x", (unsigned)e.hops, (unsigned)e.relay);
  }
  const int n = snprintf(out, cap, "%04lx#%u %s %s -%us", (unsigned long)(e.origin & 0xFFFF), (unsigned)e.seq, age,
                         path, (unsigned)e.sinceS);
  if (n < 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

void Reach::clear() { *this = Reach(); }

void Reach::heard(uint32_t origin, const FixId& fix, uint32_t ageMs, uint8_t hops, uint8_t relay, uint32_t nowMs) {
  if (origin == 0) return;
  Slot* slot = nullptr;
  for (size_t i = 0; i < REACH_SLOTS && slot == nullptr; i++) {
    if (slots_[i].origin == origin) slot = &slots_[i];
  }
  if (slot != nullptr && slot->session != 0 && slot->session == fix.session) {
    // Sixteen bits of sequence, compared across the wrap: a fix a second takes
    // nine hours to reach the half-range.
    if ((int16_t)((uint16_t)fix.seq - slot->seq) < 0) return;
  }
  if (slot == nullptr) {
    for (size_t i = 0; i < REACH_SLOTS && slot == nullptr; i++) {
      if (!fresh(slots_[i], nowMs)) slot = &slots_[i];
    }
  }
  if (slot == nullptr) {
    slot = &slots_[0];
    for (size_t i = 1; i < REACH_SLOTS; i++) {
      if ((uint32_t)(nowMs - slots_[i].heardMs) > (uint32_t)(nowMs - slot->heardMs)) slot = &slots_[i];
    }
  }
  slot->origin = origin;
  slot->heardMs = nowMs;
  slot->session = fix.session;
  slot->seq = (uint16_t)fix.seq;
  slot->ageQ = reachAgeQ(ageMs);
  slot->hops = hops > REACH_HOPS_UNKNOWN ? REACH_HOPS_UNKNOWN : hops;
  slot->relay = relay;
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

size_t Reach::takeSummary(uint32_t nowMs, uint8_t* out, size_t cap) {
  for (size_t k = 0; k < REACH_SLOTS; k++) {
    if (slots_[k].origin != 0 && !fresh(slots_[k], nowMs)) slots_[k] = Slot();
  }
  // One pass over the table from where the last summary stopped to the end;
  // the one after a pass starts again from the top.
  ReachEntry entries[REACH_PER_SUMMARY];
  size_t n = 0;
  size_t i = cursor_ < REACH_SLOTS ? cursor_ : 0;
  for (; i < REACH_SLOTS && n < REACH_PER_SUMMARY; i++) {
    const Slot& s = slots_[i];
    if (s.origin == 0) continue;
    const uint32_t sinceS = (nowMs - s.heardMs) / 1000;
    ReachEntry& e = entries[n++];
    e.origin = s.origin;
    e.seq = s.seq;
    e.ageQ = s.ageQ;
    e.sinceS = sinceS >= 255 ? 255 : (uint8_t)sinceS;
    e.hops = s.hops;
    e.relay = s.relay;
  }
  if (n == 0) {
    // The rest of the pass went quiet; start again from the top.
    if (cursor_ == 0) return 0;
    cursor_ = 0;
    return takeSummary(nowMs, out, cap);
  }
  bool more = false;
  for (size_t k = i; k < REACH_SLOTS && !more; k++) more = slots_[k].origin != 0;
  cursor_ = more ? (uint8_t)i : 0;
  const size_t len = encodeReach(entries, n, more ? REACH_MORE : 0, out, cap);
  if (len > 0) sent_++;
  return len;
}

}  // namespace touge
