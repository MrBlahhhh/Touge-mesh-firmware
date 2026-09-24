#include "phonebatch.h"

#include <stdio.h>
#include <string.h>

namespace touge {
namespace {

void put16(uint8_t* b, uint16_t v) {
  b[0] = (uint8_t)(v >> 8);
  b[1] = (uint8_t)v;
}

void put32(uint8_t* b, uint32_t v) {
  b[0] = (uint8_t)(v >> 24);
  b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);
  b[3] = (uint8_t)v;
}

uint16_t get16(const uint8_t* b) { return (uint16_t)(((uint16_t)b[0] << 8) | b[1]); }

uint32_t get32(const uint8_t* b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

void encodeRecord(const PhoneRecord& r, uint32_t nowMs, uint8_t* b) {
  put32(b + 0, r.node);
  put32(b + 4, (uint32_t)r.lat);
  put32(b + 8, (uint32_t)r.lon);
  put32(b + 12, r.frameId);
  put16(b + 16, r.headingCdeg);
  put16(b + 18, r.speedDkmh);
  // Signed: nowMs is the loop's timestamp, taken before a receive callback in
  // the same pass can stamp heardMs a few ms later. Unsigned, that negative
  // age wrapped and saturated to 65.5 s, and the phone threw away every fresh
  // 2.4 GHz position as older than LoRa.
  const int32_t age = (int32_t)(nowMs - r.heardMs);
  // Saturates at 0xFFFE: build 31+ apps read 0xFFFF as a build 30 radio's wrapped
  // negative age, i.e. "just heard", so a genuinely old position must not say it.
  put16(b + 20, age <= 0 ? (uint16_t)0 : age > (int32_t)AGE_MAX ? AGE_MAX : (uint16_t)age);
  b[22] = (uint8_t)r.rssi;
  b[23] = (uint8_t)((r.external ? 0x01 : 0) | ((r.lane & 0x03) << 1) | (r.expired ? RECORD_EXPIRED : 0) |
                    ((r.hopsAway & 0x0F) << 4));
}

void decodeRecord(const uint8_t* b, uint32_t radioMs, PhoneRecord& r) {
  r.node = get32(b + 0);
  r.lat = (int32_t)get32(b + 4);
  r.lon = (int32_t)get32(b + 8);
  r.frameId = get32(b + 12);
  r.headingCdeg = get16(b + 16);
  r.speedDkmh = get16(b + 18);
  r.heardMs = radioMs - get16(b + 20);
  r.rssi = (int8_t)b[22];
  r.external = (b[23] & 0x01) != 0;
  r.lane = (uint8_t)((b[23] >> 1) & 0x03);
  r.hopsAway = (uint8_t)(b[23] >> 4);
  r.expired = (b[23] & RECORD_EXPIRED) != 0;
}

}  // namespace

size_t batchBudgetForMtu(uint16_t mtu) {
  if (mtu == 0) return BATCH_MAX_PAYLOAD;
  // One byte of ATT opcode, then the FromRadio and MeshPacket envelope.
  const size_t envelope = 1 + 48;
  if (mtu <= envelope + BATCH_HEADER + BATCH_RECORD) return BATCH_HEADER + BATCH_RECORD;
  const size_t room = (size_t)mtu - envelope;
  return room < BATCH_MAX_PAYLOAD ? room : BATCH_MAX_PAYLOAD;
}

size_t recordsThatFit(size_t capBytes) {
  if (capBytes < BATCH_HEADER) return 0;
  const size_t n = (capBytes - BATCH_HEADER) / BATCH_RECORD;
  // The count is one byte on the wire.
  return n > 255 ? 255 : n;
}

size_t encodeBatch(const PhoneRecord* records, size_t n, uint16_t seq, uint32_t nowMs, uint8_t* out,
                   size_t cap) {
  if (out == nullptr || n > 255 || (n > 0 && records == nullptr)) return 0;
  const size_t len = BATCH_HEADER + n * BATCH_RECORD;
  if (len > cap) return 0;
  out[0] = BATCH_MAGIC;
  out[1] = BATCH_VERSION;
  out[2] = (uint8_t)BATCH_HEADER;
  out[3] = (uint8_t)BATCH_RECORD;
  out[4] = (uint8_t)n;
  out[5] = 0;
  put16(out + 6, seq);
  put32(out + 8, nowMs);
  for (size_t i = 0; i < n; i++) encodeRecord(records[i], nowMs, out + BATCH_HEADER + i * BATCH_RECORD);
  return len;
}

bool decodeBatchHeader(const uint8_t* in, size_t len, BatchHeader& header) {
  if (in == nullptr || len < BATCH_HEADER) return false;
  if (in[0] != BATCH_MAGIC || in[1] == 0) return false;
  const size_t headerLen = in[2];
  const size_t recordLen = in[3];
  const size_t count = in[4];
  // Shorter than version 1 means fields we rely on are missing.
  if (headerLen < BATCH_HEADER || recordLen < BATCH_RECORD) return false;
  if (headerLen + count * recordLen != len) return false;
  header.version = in[1];
  header.count = (uint8_t)count;
  header.seq = get16(in + 6);
  header.radioMs = get32(in + 8);
  header.flags = in[5];
  return true;
}

bool decodeBatch(const uint8_t* in, size_t len, BatchHeader& header, PhoneRecord* out, size_t outCap) {
  BatchHeader h;
  if (!decodeBatchHeader(in, len, h)) return false;
  if (h.count > outCap || (h.count > 0 && out == nullptr)) return false;
  const size_t headerLen = in[2];
  const size_t recordLen = in[3];
  for (size_t i = 0; i < h.count; i++) decodeRecord(in + headerLen + i * recordLen, h.radioMs, out[i]);
  header = h;
  return true;
}

// ---- PhoneStore ----------------------------------------------------------------

void PhoneStore::clear() {
  for (size_t i = 0; i < PHONE_STORE_SLOTS; i++) slots_[i] = Slot();
}

PhoneStore::Offer PhoneStore::offer(const PhoneRecord& r, uint32_t nowMs) {
  Slot* free = nullptr;
  Slot* longestWaiting = nullptr;
  for (size_t i = 0; i < PHONE_STORE_SLOTS; i++) {
    Slot& s = slots_[i];
    if (!s.used) {
      if (free == nullptr) free = &s;
      continue;
    }
    if (s.record.node == r.node) {
      // A relayed copy of an older frame can land after the direct copy of a
      // newer one. Frame ids count up per sender, so the signed difference
      // says which is newer across the wrap.
      if ((int32_t)(r.frameId - s.record.frameId) < 0) return STALE;
      s.record = r;
      return REPLACED;
    }
    if (longestWaiting == nullptr || (int32_t)(s.queuedMs - longestWaiting->queuedMs) < 0) longestWaiting = &s;
  }
  Slot* target = free != nullptr ? free : longestWaiting;
  if (target == nullptr) return STALE;  // PHONE_STORE_SLOTS == 0; not reachable
  const Offer result = free != nullptr ? ADDED : EVICTED;
  target->record = r;
  target->queuedMs = nowMs;
  target->used = true;
  return result;
}

size_t PhoneStore::pending() const {
  size_t n = 0;
  for (size_t i = 0; i < PHONE_STORE_SLOTS; i++)
    if (slots_[i].used) n++;
  return n;
}

uint32_t PhoneStore::oldestWaitMs(uint32_t nowMs) const {
  uint32_t oldest = 0;
  for (size_t i = 0; i < PHONE_STORE_SLOTS; i++) {
    if (!slots_[i].used) continue;
    const uint32_t waited = nowMs - slots_[i].queuedMs;
    if (waited > oldest) oldest = waited;
  }
  return oldest;
}

bool PhoneStore::due(uint32_t nowMs, size_t capBytes, uint32_t flushMs) const {
  const size_t waiting = pending();
  if (waiting == 0) return false;
  if (waiting >= recordsThatFit(capBytes)) return true;
  return oldestWaitMs(nowMs) >= flushMs;
}

size_t PhoneStore::takeBatch(uint8_t* out, size_t cap, uint16_t seq, uint32_t nowMs, size_t& taken) {
  taken = 0;
  size_t room = recordsThatFit(cap);
  if (room > BATCH_MAX_RECORDS) room = BATCH_MAX_RECORDS;
  if (room == 0) return 0;

  PhoneRecord batch[BATCH_MAX_RECORDS];
  // Longest-waiting first, so a car that missed the last batch is in this one.
  while (taken < room) {
    Slot* next = nullptr;
    for (size_t i = 0; i < PHONE_STORE_SLOTS; i++) {
      Slot& s = slots_[i];
      if (!s.used) continue;
      if (next == nullptr || (int32_t)(s.queuedMs - next->queuedMs) < 0) next = &s;
    }
    if (next == nullptr) break;
    next->used = false;
    if ((int32_t)(nowMs - next->record.heardMs) > (int32_t)RECORD_EXPIRE_MS) {
      expired_++;
      continue;
    }
    batch[taken++] = next->record;
  }
  if (taken == 0) return 0;
  return encodeBatch(batch, taken, seq, nowMs, out, cap);
}

// ---- BatchesInFlight -------------------------------------------------------------

void BatchesInFlight::clear() { count_ = 0; }

uint8_t BatchesInFlight::removeAt(size_t i) {
  const uint8_t records = entries_[i].records;
  for (size_t k = i + 1; k < count_; k++) entries_[k - 1] = entries_[k];
  count_--;
  return records;
}

bool BatchesInFlight::add(uint16_t seq, uint8_t records, uint32_t nowMs, uint32_t packetId, bool preloaded) {
  if (count_ >= MAX_BATCHES_IN_FLIGHT) return false;
  Entry& e = entries_[count_++];
  e.seq = seq;
  e.records = records;
  e.atMs = nowMs;
  e.packetId = packetId;
  e.preloaded = preloaded;
  return true;
}

uint8_t BatchesInFlight::delivered(uint16_t seq) {
  for (size_t i = 0; i < count_; i++)
    if (entries_[i].seq == seq) return removeAt(i);
  return 0;
}

uint8_t BatchesInFlight::dropped(uint16_t seq) { return delivered(seq); }

uint8_t BatchesInFlight::preloadRead() {
  for (size_t i = 0; i < count_; i++)
    if (entries_[i].preloaded) return removeAt(i);
  return 0;
}

uint8_t BatchesInFlight::preloadLost() { return preloadRead(); }

bool BatchesInFlight::hasPreloaded() const {
  for (size_t i = 0; i < count_; i++)
    if (entries_[i].preloaded) return true;
  return false;
}

uint32_t BatchesInFlight::reconcile(bool (*stillQueued)(uint32_t packetId, void* ctx), void* ctx) {
  uint32_t lost = 0;
  size_t i = 0;
  while (i < count_) {
    if (!entries_[i].preloaded && !stillQueued(entries_[i].packetId, ctx)) {
      lost += removeAt(i);
    } else {
      i++;
    }
  }
  return lost;
}

uint32_t BatchesInFlight::pendingRecords() const {
  uint32_t n = 0;
  for (size_t i = 0; i < count_; i++) n += entries_[i].records;
  return n;
}

uint32_t BatchesInFlight::oldestAgeMs(uint32_t nowMs) const {
  uint32_t oldest = 0;
  for (size_t i = 0; i < count_; i++) {
    const uint32_t age = nowMs - entries_[i].atMs;
    if (age > oldest) oldest = age;
  }
  return oldest;
}

size_t deliverBatch(uint8_t* payload, size_t len, uint32_t nowMs, bool canShrink, uint32_t& expired) {
  expired = 0;
  BatchHeader h;
  if (!decodeBatchHeader(payload, len, h)) return 0;
  const int32_t waited = (int32_t)(nowMs - h.radioMs);
  const uint32_t extra = waited > 0 ? (uint32_t)waited : 0;
  const size_t headerLen = payload[2];
  const size_t recordLen = payload[3];
  size_t kept = 0;
  for (size_t i = 0; i < h.count; i++) {
    uint8_t* rec = payload + headerLen + i * recordLen;
    const uint32_t age = (uint32_t)get16(rec + 20) + extra;
    if (age > RECORD_EXPIRE_MS || (rec[23] & RECORD_EXPIRED) != 0) {
      expired++;
      if (canShrink) continue;
      rec[23] |= RECORD_EXPIRED;
      put16(rec + 20, AGE_MAX);
    } else {
      put16(rec + 20, (uint16_t)age);
    }
    uint8_t* to = payload + headerLen + kept * recordLen;
    if (to != rec) memmove(to, rec, recordLen);
    kept++;
  }
  payload[4] = (uint8_t)kept;
  payload[5] |= BATCH_AGES_AT_DELIVERY;
  put32(payload + 8, h.radioMs + extra);
  return headerLen + kept * recordLen;
}

int findPayload(const uint8_t* fromRadio, size_t len, const uint8_t* payload, size_t payloadLen) {
  if (fromRadio == nullptr || payload == nullptr || payloadLen == 0 || payloadLen > len) return -1;
  for (size_t at = 0; at + payloadLen <= len; at++)
    if (memcmp(fromRadio + at, payload, payloadLen) == 0) return (int)at;
  return -1;
}

// ---- Writes the radio dropped --------------------------------------------------------

namespace {

bool readVarint(const uint8_t* b, size_t len, size_t& at, uint64_t& v) {
  v = 0;
  for (int shift = 0; shift < 64 && at < len; shift += 7) {
    const uint8_t byte = b[at++];
    v |= (uint64_t)(byte & 0x7F) << shift;
    if ((byte & 0x80) == 0) return true;
  }
  return false;
}

// Moves [at] past one field of [wire] type. False on anything malformed.
bool skipField(const uint8_t* b, size_t len, size_t& at, uint32_t wire) {
  uint64_t n = 0;
  switch (wire) {
    case 0:
      return readVarint(b, len, at, n);
    case 1:
      at += 8;
      return at <= len;
    case 2:
      if (!readVarint(b, len, at, n) || n > len - at) return false;
      at += (size_t)n;
      return true;
    case 5:
      at += 4;
      return at <= len;
    default:
      return false;
  }
}

}  // namespace

bool toRadioPacketId(const uint8_t* toRadio, size_t len, uint32_t& id) {
  if (toRadio == nullptr) return false;
  size_t at = 0;
  while (at < len) {
    uint64_t key = 0;
    if (!readVarint(toRadio, len, at, key)) return false;
    const uint32_t field = (uint32_t)(key >> 3);
    const uint32_t wire = (uint32_t)(key & 7);
    if (field != 1 || wire != 2) {
      if (!skipField(toRadio, len, at, wire)) return false;
      continue;
    }
    uint64_t packetLen = 0;
    if (!readVarint(toRadio, len, at, packetLen) || packetLen > len - at) return false;
    const uint8_t* p = toRadio + at;
    const size_t plen = (size_t)packetLen;
    size_t pat = 0;
    while (pat < plen) {
      uint64_t k = 0;
      if (!readVarint(p, plen, pat, k)) return false;
      if ((k >> 3) == 6 && (k & 7) == 5) {
        if (pat + 4 > plen) return false;
        // fixed32 is little-endian on the wire.
        id = (uint32_t)p[pat] | ((uint32_t)p[pat + 1] << 8) | ((uint32_t)p[pat + 2] << 16) |
             ((uint32_t)p[pat + 3] << 24);
        return true;
      }
      if (!skipField(p, plen, pat, (uint32_t)(k & 7))) return false;
    }
    return false;
  }
  return false;
}

void DroppedWriteIds::push(uint32_t id) {
  const uint32_t tail = tail_.load(std::memory_order_relaxed);
  const uint32_t head = head_.load(std::memory_order_acquire);
  if (tail - head >= SLOTS) {
    overflowed_.fetch_add(1);
    return;
  }
  ids_[tail % SLOTS] = id;
  tail_.store(tail + 1, std::memory_order_release);
}

bool DroppedWriteIds::pop(uint32_t& id) {
  const uint32_t head = head_.load(std::memory_order_relaxed);
  const uint32_t tail = tail_.load(std::memory_order_acquire);
  if (head == tail) return false;
  id = ids_[head % SLOTS];
  head_.store(head + 1, std::memory_order_release);
  return true;
}

size_t formatDroppedWrites(const uint32_t* ids, size_t n, char* out, size_t cap) {
  if (out == nullptr || cap == 0 || n == 0 || ids == nullptr) return 0;
  size_t at = 0;
  int w = snprintf(out, cap, "{\"wd\":[");
  if (w <= 0 || (size_t)w >= cap) return 0;
  at = (size_t)w;
  for (size_t i = 0; i < n; i++) {
    w = snprintf(out + at, cap - at, i == 0 ? "%lu" : ",%lu", (unsigned long)ids[i]);
    if (w <= 0 || (size_t)w >= cap - at) return 0;
    at += (size_t)w;
  }
  w = snprintf(out + at, cap - at, "]}");
  if (w <= 0 || (size_t)w >= cap - at) return 0;
  return at + (size_t)w;
}

// ---- Hello -------------------------------------------------------------------------

bool decodeHello(const uint8_t* in, size_t len, PhoneHello& out) {
  if (in == nullptr || len < HELLO_LEN) return false;
  if (in[0] != HELLO_MAGIC || in[1] == 0) return false;
  out.flags = in[2];
  out.mtu = get16(in + 3);
  return true;
}

size_t encodeHello(const PhoneHello& h, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < HELLO_LEN) return 0;
  out[0] = HELLO_MAGIC;
  out[1] = HELLO_VERSION;
  out[2] = h.flags;
  put16(out + 3, h.mtu);
  return HELLO_LEN;
}

// ---- Stats ---------------------------------------------------------------------------

size_t formatLaneStats(const LinkStats& s, char* out, size_t cap) {
  if (out == nullptr || cap == 0) return 0;
  int n = snprintf(out, cap, "{\"fs\":[1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu]}",
                   (unsigned long)s.fast.tx, (unsigned long)s.fast.rx, (unsigned long)s.fast.suppressed,
                   (unsigned long)s.fast.queued, (unsigned long)s.fast.delivered, (unsigned long)s.lora.rx,
                   (unsigned long)s.lora.delivered, (unsigned long)s.batches, (unsigned long)s.batchesRead,
                   (unsigned long)s.replaced, (unsigned long)s.fastTxFail);
  if (n <= 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

size_t formatQueueStats(const LinkStats& s, char* out, size_t cap) {
  if (out == nullptr || cap == 0) return 0;
  int n = snprintf(out, cap,
                   "{\"fq\":[1,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu]}",
                   (unsigned)s.queueDepth, (unsigned)s.queueDepthMax, (unsigned)s.storePending,
                   (unsigned long)s.oldestQueuedMs, (unsigned long)s.dropStoreFull, (unsigned long)s.dropAlloc,
                   (unsigned long)s.dropLost, (unsigned long)s.dropDisconnect, (unsigned long)s.dropStale,
                   (unsigned long)s.coreReplaced, (unsigned long)s.coreDropped, (unsigned long)s.writeDropped,
                   (unsigned long)s.writeDuplicate, (unsigned long)s.preloadOffered,
                   (unsigned long)s.preloadRead, (unsigned long)s.preloadRefused, (unsigned long)s.minFreeHeap,
                   (unsigned long)s.coreEvicted, (unsigned long)s.dropExpired);
  if (n <= 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

namespace {
// Tenths per second, so the line needs no floating point.
unsigned long perSecX10(uint32_t now, uint32_t before, uint32_t windowMs) {
  if (windowMs == 0) return 0;
  return (unsigned long)(((uint64_t)(now - before) * 10000u) / windowMs);
}
}  // namespace

size_t formatBaseline(const LinkStats& now, const LinkStats& before, uint32_t windowMs, char* out,
                      size_t cap) {
  if (out == nullptr || cap == 0) return 0;
  const unsigned long ftx = perSecX10(now.fast.tx, before.fast.tx, windowMs);
  const unsigned long frx = perSecX10(now.fast.rx, before.fast.rx, windowMs);
  const unsigned long fq = perSecX10(now.fast.queued, before.fast.queued, windowMs);
  const unsigned long fd = perSecX10(now.fast.delivered, before.fast.delivered, windowMs);
  const unsigned long lrx = perSecX10(now.lora.rx, before.lora.rx, windowMs);
  const unsigned long ld = perSecX10(now.lora.delivered, before.lora.delivered, windowMs);
  const unsigned long bq = perSecX10(now.batches, before.batches, windowMs);
  const uint32_t drops = (now.dropStoreFull - before.dropStoreFull) + (now.dropAlloc - before.dropAlloc) +
                         (now.dropLost - before.dropLost) + (now.dropDisconnect - before.dropDisconnect) +
                         (now.coreDropped - before.coreDropped) + (now.coreEvicted - before.coreEvicted) +
                         (now.dropExpired - before.dropExpired);
  int n = snprintf(out, cap,
                   "BASELINE radio fast tx=%lu.%lu rx=%lu.%lu queued=%lu.%lu delivered=%lu.%lu/s "
                   "lora rx=%lu.%lu delivered=%lu.%lu/s batches=%lu.%lu/s depth=%u max=%u pending=%u "
                   "oldest=%lums drops=%lu wlost=%lu",
                   ftx / 10, ftx % 10, frx / 10, frx % 10, fq / 10, fq % 10, fd / 10, fd % 10, lrx / 10,
                   lrx % 10, ld / 10, ld % 10, bq / 10, bq % 10, (unsigned)now.queueDepth,
                   (unsigned)now.queueDepthMax, (unsigned)now.storePending, (unsigned long)now.oldestQueuedMs,
                   (unsigned long)drops, (unsigned long)(now.writeDropped - before.writeDropped));
  if (n <= 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

}  // namespace touge
