#include "mesh.h"
#include <string.h>

namespace touge {

void Mesh::reset() {
  memset(seen_, 0, sizeof(seen_));
  memset(riders_, 0, sizeof(riders_));
  lastId_ = 0;
}

bool Mesh::firstSight(uint32_t src, uint32_t id, uint32_t nowMs) {
  int free = -1;
  int oldest = 0;
  uint32_t oldestAt = 0xFFFFFFFF;

  for (size_t i = 0; i < SEEN_SLOTS; i++) {
    Seen& s = seen_[i];
    // An expired slot is treated as empty rather than as a match, so a rider
    // who left and came back is not silenced by a stale entry.
    if (s.used && (uint32_t)(nowMs - s.atMs) > SEEN_TTL_MS) s.used = false;
    if (!s.used) {
      if (free < 0) free = (int)i;
      continue;
    }
    if (s.src == src && s.id == id) return false;
    if (s.atMs < oldestAt) {
      oldestAt = s.atMs;
      oldest = (int)i;
    }
  }

  // Full and nothing expired: evict the oldest. Worst case this re-forwards a
  // packet once, which costs airtime. The alternative is refusing to record
  // it, which loses dedupe entirely and costs far more.
  int slot = free >= 0 ? free : oldest;
  seen_[slot].src = src;
  seen_[slot].id = id;
  seen_[slot].atMs = nowMs;
  seen_[slot].used = true;
  return true;
}

Rider* Mesh::note(uint32_t src, const Position& p, uint8_t via, int16_t rssi, uint8_t hopsAway,
                  uint32_t nowMs) {
  Rider* slot = nullptr;
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    if (riders_[i].used && riders_[i].id == src) {
      slot = &riders_[i];
      break;
    }
  }
  if (slot == nullptr) {
    for (size_t i = 0; i < MAX_RIDERS; i++) {
      if (!riders_[i].used) {
        slot = &riders_[i];
        break;
      }
    }
  }
  if (slot == nullptr) {
    // Full. Give the seat to the newcomer only if someone on it has gone
    // properly quiet, so a ninth car cannot bump a car you are driving behind.
    uint32_t worstAt = nowMs;
    Rider* worst = nullptr;
    for (size_t i = 0; i < MAX_RIDERS; i++) {
      if (riders_[i].used && (uint32_t)(nowMs - riders_[i].atMs) > RIDER_STALE_MS &&
          riders_[i].atMs <= worstAt) {
        worstAt = riders_[i].atMs;
        worst = &riders_[i];
      }
    }
    if (worst == nullptr) return nullptr;
    slot = worst;
    memset(slot, 0, sizeof(*slot));
  }

  // A name arrives only now and then, so an empty one means "unchanged", not
  // "forget who this is". Without this the roster blanks between name pings.
  char keep[sizeof(slot->pos.name)];
  memcpy(keep, slot->pos.name, sizeof(keep));

  slot->id = src;
  slot->pos = p;
  if (p.name[0] == 0) memcpy(slot->pos.name, keep, sizeof(keep));
  slot->atMs = nowMs;
  slot->via = via;
  slot->rssi = rssi;
  slot->hopsAway = hopsAway;
  slot->used = true;
  return slot;
}

void Mesh::age(uint32_t nowMs) {
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    if (riders_[i].used && (uint32_t)(nowMs - riders_[i].atMs) > RIDER_DROP_MS) {
      memset(&riders_[i], 0, sizeof(riders_[i]));
    }
  }
}

size_t Mesh::count() const {
  size_t n = 0;
  for (size_t i = 0; i < MAX_RIDERS; i++)
    if (riders_[i].used) n++;
  return n;
}

const Rider* Mesh::find(uint32_t id) const {
  for (size_t i = 0; i < MAX_RIDERS; i++)
    if (riders_[i].used && riders_[i].id == id) return &riders_[i];
  return nullptr;
}

} // namespace touge
