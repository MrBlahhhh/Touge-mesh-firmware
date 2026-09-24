#include "lorapos.h"

#include "ownfix.h"

namespace touge {

FixId fixIdOf(uint32_t sensorId, uint32_t seqNumber, uint32_t timestamp, int32_t millisAdjust, uint32_t time) {
  Fix measured;
  setMeasured(measured, timestamp, millisAdjust, time);
  FixId fix;
  // Our sessions are 16 bits. A wider sensor_id is somebody else's use of the
  // field, not a session.
  fix.session = sensorId <= 0xFFFF ? (uint16_t)sensorId : 0;
  fix.seq = seqNumber;
  fix.fixSec = measured.fixSec;
  fix.fixMs = measured.fixMs;
  return fix;
}

FixRank rankQueued(const FixId& incoming, const FixId& queued) {
  if (incoming.session == 0 || queued.session == 0) return FixRank::NEWER;
  return rankFix(incoming, queued);
}

void TxPositions::clear() {
  for (size_t i = 0; i < SLOTS; i++) tags_[i] = Tag();
}

void TxPositions::note(uint32_t from, uint32_t id, const FixId& fix, uint32_t nowMs, InQueue inQueue, void* ctx) {
  if (fix.session == 0) return;
  Tag* tag = nullptr;
  for (size_t i = 0; i < SLOTS && tag == nullptr; i++) {
    if (tags_[i].fix.session != 0 && tags_[i].from == from && tags_[i].id == id) tag = &tags_[i];
  }
  for (size_t i = 0; i < SLOTS && tag == nullptr; i++) {
    if (tags_[i].fix.session == 0) tag = &tags_[i];
  }
  // Sent, replaced, or never queued: its place is free.
  for (size_t i = 0; i < SLOTS && tag == nullptr; i++) {
    if (!inQueue(tags_[i].from, tags_[i].id, ctx)) tag = &tags_[i];
  }
  if (tag == nullptr) return;
  tag->from = from;
  tag->id = id;
  tag->fix = fix;
  tag->notedMs = nowMs;
}

const FixId* TxPositions::find(uint32_t from, uint32_t id) const {
  for (size_t i = 0; i < SLOTS; i++) {
    if (tags_[i].fix.session != 0 && tags_[i].from == from && tags_[i].id == id) return &tags_[i].fix;
  }
  return nullptr;
}

bool TxPositions::waited(uint32_t from, uint32_t id, uint32_t nowMs, uint32_t& waitedMs) const {
  for (size_t i = 0; i < SLOTS; i++) {
    if (tags_[i].fix.session != 0 && tags_[i].from == from && tags_[i].id == id) {
      waitedMs = nowMs - tags_[i].notedMs;
      return true;
    }
  }
  return false;
}

TxPlace TxPositions::placeAgainst(const FixId& incoming, uint8_t incomingHops, const FixId& held, uint8_t heldHops) {
  switch (rankFix(incoming, held)) {
    case FixRank::NEWER:
      return TxPlace::REPLACE;
    case FixRank::SAME:
      // The same fix sent twice (a parked phone repeats its last one): keep the
      // copy that can still travel further.
      return incomingHops > heldHops ? TxPlace::REPLACE : TxPlace::REFUSE;
    case FixRank::OLDER:
    default:
      return TxPlace::REFUSE;
  }
}

}  // namespace touge
