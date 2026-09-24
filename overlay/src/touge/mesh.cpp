#include "mesh.h"
#include <string.h>

namespace touge {

uint32_t forwardSpreadMs(size_t neighbours) {
  // One neighbour needs no room at all to avoid anybody, but the floor is the
  // old flat window: two cars still have to miss each other.
  uint32_t want = (uint32_t)(FORWARD_JITTER_MS * (neighbours > 0 ? neighbours : 1)) /
                  (SUPPRESS_AFTER > 0 ? SUPPRESS_AFTER : 1);
  if (want < FORWARD_JITTER_MS) want = FORWARD_JITTER_MS;
  if (want > FORWARD_JITTER_MAX_MS) want = FORWARD_JITTER_MAX_MS;
  return want;
}

uint32_t forwardDelayMs(int16_t rssi, uint32_t spreadMs, uint32_t tieBreak) {
  const uint32_t tie = FORWARD_TIE_MS > 0 ? (tieBreak % FORWARD_TIE_MS) : 0;
  if (spreadMs == 0) return tie;

  int32_t r = rssi;
  if (r < FORWARD_FAR_DBM) r = FORWARD_FAR_DBM;
  if (r > FORWARD_NEAR_DBM) r = FORWARD_NEAR_DBM;

  // Zero at the far end, the full spread at the near end. A board reporting no
  // RSSI at all reads as 0 dBm, which clamps to the near end and holds
  // longest - the right way to be wrong, since a car that cannot measure
  // signal should not be the one elected to relay.
  const int32_t span = (int32_t)FORWARD_NEAR_DBM - (int32_t)FORWARD_FAR_DBM;
  const uint32_t d = (uint32_t)(((int64_t)(r - FORWARD_FAR_DBM) * (int64_t)spreadMs) / span);
  return d + tie;
}

uint32_t nextOnGrid(uint32_t deadlineMs, uint32_t periodMs, uint32_t nowMs) {
  if ((int32_t)(nowMs - deadlineMs) < 0) return deadlineMs;
  if (periodMs == 0) return nowMs;
  // Skips every grid point already missed, so a long stall resumes on the grid
  // rather than firing once per missed period to catch up.
  const uint32_t missed = (nowMs - deadlineMs) / periodMs + 1;
  return deadlineMs + missed * periodMs;
}

void Mesh::reset() {
  memset(seen_, 0, sizeof(seen_));
  memset(riders_, 0, sizeof(riders_));
  memset(held_, 0, sizeof(held_));
  lastId_ = 0;
  suppressed_ = 0;
}

Mesh::Seen* Mesh::lookup(uint32_t src, uint32_t id, uint32_t nowMs) {
  for (size_t i = 0; i < SEEN_SLOTS; i++) {
    Seen& s = seen_[i];
    if (!s.used) continue;
    // An expired slot is treated as empty rather than as a match, so a rider
    // who left and came back is not silenced by a stale entry.
    if ((uint32_t)(nowMs - s.atMs) > SEEN_TTL_MS) {
      s.used = false;
      continue;
    }
    if (s.src == src && s.id == id) return &s;
  }
  return nullptr;
}

const Mesh::Seen* Mesh::lookup(uint32_t src, uint32_t id, uint32_t nowMs) const {
  for (size_t i = 0; i < SEEN_SLOTS; i++) {
    const Seen& s = seen_[i];
    if (!s.used) continue;
    if ((uint32_t)(nowMs - s.atMs) > SEEN_TTL_MS) continue;
    if (s.src == src && s.id == id) return &s;
  }
  return nullptr;
}

bool Mesh::firstSight(uint32_t src, uint32_t id, uint32_t nowMs) {
  Seen* hit = lookup(src, id, nowMs);
  if (hit != nullptr) {
    if (hit->count < 255) hit->count++;
    return false;
  }

  int free = -1;
  int oldest = 0;
  uint32_t oldestAt = 0xFFFFFFFF;
  for (size_t i = 0; i < SEEN_SLOTS; i++) {
    Seen& s = seen_[i];
    if (!s.used) {
      if (free < 0) free = (int)i;
      continue;
    }
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
  seen_[slot].count = 1;
  seen_[slot].used = true;
  return true;
}

uint8_t Mesh::copies(uint32_t src, uint32_t id, uint32_t nowMs) const {
  const Seen* hit = lookup(src, id, nowMs);
  return hit ? hit->count : 0;
}

size_t Mesh::slotCapacity(size_t i) {
  return i < FORWARD_FULL_SLOTS ? FRAME_MAX : FORWARD_SMALL_BYTES;
}

uint8_t* Mesh::slotBytes(size_t i) {
  if (i < FORWARD_FULL_SLOTS) return forwardBytes_ + i * FRAME_MAX;
  return forwardBytes_ + FORWARD_FULL_SLOTS * FRAME_MAX + (i - FORWARD_FULL_SLOTS) * FORWARD_SMALL_BYTES;
}

bool Mesh::holdIn(size_t i, const uint8_t* wire, size_t len, uint32_t src, uint32_t id,
                  uint32_t dueMs) {
  if (held_[i].used || len > slotCapacity(i)) return false;
  memcpy(slotBytes(i), wire, len);
  held_[i].len = (uint16_t)len;
  held_[i].src = src;
  held_[i].id = id;
  held_[i].dueMs = dueMs;
  held_[i].used = true;
  return true;
}

bool Mesh::defer(const uint8_t* wire, size_t len, uint32_t src, uint32_t id, uint32_t dueMs) {
  if (wire == nullptr || len == 0 || len > FRAME_MAX) return false;

  // Small slots first, so a burst of positions does not take the full-size
  // slots a voice frame needs. They fall back to a full one when those run out.
  for (size_t i = FORWARD_FULL_SLOTS; i < FORWARD_SLOTS; i++) {
    if (holdIn(i, wire, len, src, id, dueMs)) return true;
  }
  for (size_t i = 0; i < FORWARD_FULL_SLOTS; i++) {
    if (holdIn(i, wire, len, src, id, dueMs)) return true;
  }
  // No room. The frame is dropped rather than pushing an already-waiting one
  // out: a forward that arrives late is worth less than one that arrives, and
  // in a convoy this busy somebody else is almost certainly forwarding anyway.
  return false;
}

bool Mesh::nextDue(uint32_t nowMs, Forward& out) {
  // Oldest debt first, rather than whichever slot happens to be lowest.
  //
  // Taking these in array order meant a frame that came due twenty
  // milliseconds ago could go out behind one that came due just now, purely
  // because it landed in a higher slot. The jitter that spaces forwards apart
  // is chosen per frame, so under load the order the table happens to be in
  // has nothing to do with the order the air wanted them in.
  for (;;) {
    int best = -1;
    for (size_t i = 0; i < FORWARD_SLOTS; i++) {
      const Held& h = held_[i];
      if (!h.used) continue;
      // Unsigned, so a frame scheduled before a millis() wrap still comes due
      // rather than waiting out the next forty-nine days. The same signed
      // difference orders two due frames against each other.
      if ((int32_t)(nowMs - h.dueMs) < 0) continue;
      if (best < 0 || (int32_t)(h.dueMs - held_[best].dueMs) < 0) best = (int)i;
    }
    if (best < 0) return false;

    Held& due = held_[best];
    due.used = false;
    // Neighbours may have rebroadcast it while this one waited. If enough of
    // them did, everyone in earshot has it and this transmission would be
    // pure interference. Suppressing one does not excuse the rest, so this
    // goes round again rather than giving up for this tick; the slot has been
    // released either way, so the loop always shrinks.
    if (copies(due.src, due.id, nowMs) >= SUPPRESS_AFTER) {
      suppressed_++;
      continue;
    }
    memcpy(out.wire, slotBytes((size_t)best), due.len);
    out.len = due.len;
    out.src = due.src;
    out.id = due.id;
    out.dueMs = due.dueMs;
    return true;
  }
}

Rider* Mesh::note(uint32_t src, const Position& p, uint8_t via, int16_t rssi, uint8_t hopsAway,
                  uint32_t nowMs, uint8_t chan) {
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
  slot->chan = chan;
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

size_t Mesh::countOn(uint8_t chan, uint32_t windowMs, uint32_t nowMs) const {
  size_t n = 0;
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    if (!riders_[i].used) continue;
    if (riders_[i].chan != chan) continue;
    if ((uint32_t)(nowMs - riders_[i].atMs) > windowMs) continue;
    n++;
  }
  return n;
}

const Rider* Mesh::find(uint32_t id) const {
  for (size_t i = 0; i < MAX_RIDERS; i++)
    if (riders_[i].used && riders_[i].id == id) return &riders_[i];
  return nullptr;
}

} // namespace touge
