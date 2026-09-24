// Host tests for the phone-bound position batches (SCALE-PLAN steps 1 and 3).
//
// Its own suite rather than more of test_frame, so the schedule work landing in
// that file at the same time does not have to merge with this.

#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "phonebatch.h"

using namespace touge;

void setUp() {}
void tearDown() {}

static PhoneRecord car(uint32_t node, uint32_t frameId, uint32_t heardMs) {
  PhoneRecord r;
  r.node = node;
  r.lat = 355000000 + (int32_t)node;
  r.lon = -825000000 - (int32_t)node;
  r.frameId = frameId;
  r.headingCdeg = 27150;
  r.speedDkmh = 885;
  r.rssi = -71;
  r.external = true;
  r.lane = LANE_FAST;
  r.hopsAway = 1;
  r.heardMs = heardMs;
  return r;
}

void test_a_batch_round_trips_every_field() {
  PhoneRecord in[3] = {car(0xb03436ae, 7, 900), car(0x11, 0xFFFFFFF0, 950), car(0x22, 1, 1000)};
  in[1].lat = -1;
  in[1].external = false;
  in[1].rssi = 0;
  in[2].hopsAway = 15;
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t n = encodeBatch(in, 3, 0xBEEF, 1000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(BATCH_HEADER + 3 * BATCH_RECORD, n);
  TEST_ASSERT_EQUAL_HEX8(BATCH_MAGIC, buf[0]);

  BatchHeader h;
  PhoneRecord out[4];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 4));
  TEST_ASSERT_EQUAL(3, h.count);
  TEST_ASSERT_EQUAL_HEX16(0xBEEF, h.seq);
  TEST_ASSERT_EQUAL(1000, h.radioMs);
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_EQUAL_HEX32(in[i].node, out[i].node);
    TEST_ASSERT_EQUAL(in[i].lat, out[i].lat);
    TEST_ASSERT_EQUAL(in[i].lon, out[i].lon);
    TEST_ASSERT_EQUAL_HEX32(in[i].frameId, out[i].frameId);
    TEST_ASSERT_EQUAL(in[i].headingCdeg, out[i].headingCdeg);
    TEST_ASSERT_EQUAL(in[i].speedDkmh, out[i].speedDkmh);
    TEST_ASSERT_EQUAL(in[i].rssi, out[i].rssi);
    TEST_ASSERT_EQUAL(in[i].external, out[i].external);
    TEST_ASSERT_EQUAL(in[i].lane, out[i].lane);
    TEST_ASSERT_EQUAL(in[i].hopsAway, out[i].hopsAway);
    // Carried as an age against the header's clock, so it comes back exact.
    TEST_ASSERT_EQUAL(in[i].heardMs, out[i].heardMs);
  }
}

// The same bytes are pinned in the app's PositionBatchTest, so the two decoders agree.
void test_a_known_batch_encodes_to_the_pinned_bytes() {
  PhoneRecord r;
  r.node = 0xb03436ae;
  r.lat = 355123456;
  r.lon = -825654321;
  r.frameId = 7;
  r.headingCdeg = 27150;
  r.speedDkmh = 885;
  r.heardMs = 900;
  r.rssi = -71;
  r.external = true;
  r.lane = LANE_FAST;
  r.hopsAway = 1;
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t n = encodeBatch(&r, 1, 0xBEEF, 1000, buf, sizeof(buf));
  char hexOut[2 * BATCH_MAX_PAYLOAD + 1];
  for (size_t i = 0; i < n; i++) sprintf(hexOut + 2 * i, "%02X", buf[i]);
  hexOut[2 * n] = 0;
  TEST_ASSERT_EQUAL_STRING(
      "C1010C180100BEEF000003E8B03436AE152AC100CEC983CF000000076A0E03750064B911", hexOut);
}

// The first byte is what tells a batch from JSON and from a voice frame on the
// same port. Pinned, because the app switches on it.
void test_the_first_byte_cannot_be_json_or_voice() {
  TEST_ASSERT_NOT_EQUAL('{', BATCH_MAGIC);
  TEST_ASSERT_NOT_EQUAL(0x54, BATCH_MAGIC);
  TEST_ASSERT_NOT_EQUAL('{', HELLO_MAGIC);
  TEST_ASSERT_NOT_EQUAL(0x54, HELLO_MAGIC);
  TEST_ASSERT_NOT_EQUAL(BATCH_MAGIC, HELLO_MAGIC);
  // 0xC0 and 0xC1 never appear in UTF-8; 0xC2 is only ever a lead byte, and
  // JSON text cannot start with a non-ASCII character.
  TEST_ASSERT_TRUE(BATCH_MAGIC >= 0x80);
  TEST_ASSERT_TRUE(HELLO_MAGIC >= 0x80);
}

void test_a_truncated_batch_is_rejected_whole() {
  PhoneRecord in[2] = {car(1, 1, 0), car(2, 1, 0)};
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t n = encodeBatch(in, 2, 1, 10, buf, sizeof(buf));
  BatchHeader h;
  PhoneRecord out[2];
  out[0].node = 0xDEAD;
  for (size_t cut = 0; cut < n; cut++) {
    TEST_ASSERT_FALSE(decodeBatch(buf, cut, h, out, 2));
  }
  // Nothing half-written on the way out.
  TEST_ASSERT_EQUAL_HEX32(0xDEAD, out[0].node);
  // A count that claims more than arrived.
  buf[4] = 3;
  TEST_ASSERT_FALSE(decodeBatch(buf, n, h, out, 2));
}

void test_the_header_alone_names_the_batch() {
  PhoneRecord in[2] = {car(1, 1, 0), car(2, 1, 0)};
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t n = encodeBatch(in, 2, 0x1234, 10, buf, sizeof(buf));
  BatchHeader h;
  TEST_ASSERT_TRUE(decodeBatchHeader(buf, n, h));
  TEST_ASSERT_EQUAL_HEX16(0x1234, h.seq);
  TEST_ASSERT_EQUAL(2, h.count);
  TEST_ASSERT_FALSE(decodeBatchHeader(buf, n - 1, h));
}

void test_junk_is_not_a_batch() {
  const uint8_t json[] = "{\"fl\":{}}";
  BatchHeader h;
  PhoneRecord out[1];
  TEST_ASSERT_FALSE(decodeBatch(json, sizeof(json) - 1, h, out, 1));
  uint8_t zeros[BATCH_HEADER] = {0};
  TEST_ASSERT_FALSE(decodeBatch(zeros, sizeof(zeros), h, out, 1));
  TEST_ASSERT_FALSE(decodeBatch(nullptr, 0, h, out, 1));
}

void test_more_records_than_the_caller_holds_is_rejected() {
  PhoneRecord in[3] = {car(1, 1, 0), car(2, 1, 0), car(3, 1, 0)};
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t n = encodeBatch(in, 3, 1, 10, buf, sizeof(buf));
  BatchHeader h;
  PhoneRecord out[2];
  TEST_ASSERT_FALSE(decodeBatch(buf, n, h, out, 2));
}

// A later version may append to the header and to each record. This build
// reads the fields it knows and steps over the rest.
void test_a_longer_future_record_still_decodes() {
  const size_t header = BATCH_HEADER + 2;
  const size_t record = BATCH_RECORD + 4;
  uint8_t v1[BATCH_MAX_PAYLOAD];
  PhoneRecord in[2] = {car(5, 9, 100), car(6, 10, 100)};
  encodeBatch(in, 2, 7, 100, v1, sizeof(v1));

  uint8_t v2[BATCH_MAX_PAYLOAD] = {0};
  memcpy(v2, v1, BATCH_HEADER);
  v2[1] = 2;
  v2[2] = (uint8_t)header;
  v2[3] = (uint8_t)record;
  for (int i = 0; i < 2; i++) memcpy(v2 + header + i * record, v1 + BATCH_HEADER + i * BATCH_RECORD, BATCH_RECORD);
  BatchHeader h;
  PhoneRecord out[2];
  TEST_ASSERT_TRUE(decodeBatch(v2, header + 2 * record, h, out, 2));
  TEST_ASSERT_EQUAL(2, h.version);
  TEST_ASSERT_EQUAL_HEX32(6, out[1].node);
  TEST_ASSERT_EQUAL_HEX32(10, out[1].frameId);
}

void test_budget_follows_the_mtu_and_stops_at_the_payload_limit() {
  // The app negotiates 517; Meshtastic's payload ceiling wins.
  TEST_ASSERT_EQUAL(BATCH_MAX_PAYLOAD, batchBudgetForMtu(517));
  // 247, which one test radio settled on: one ATT response, no read blob.
  TEST_ASSERT_TRUE(batchBudgetForMtu(247) + 49 <= 247);
  TEST_ASSERT_EQUAL(7, recordsThatFit(batchBudgetForMtu(247)));
  // A default 23-byte MTU still gets one record rather than nothing.
  TEST_ASSERT_EQUAL(1, recordsThatFit(batchBudgetForMtu(23)));
  TEST_ASSERT_EQUAL(9, recordsThatFit(BATCH_MAX_PAYLOAD));
  // A phone that never learned its MTU still gets full batches; long reads cope.
  TEST_ASSERT_EQUAL(BATCH_MAX_PAYLOAD, batchBudgetForMtu(0));
}

void test_a_second_record_for_a_car_replaces_the_first() {
  PhoneStore s;
  s.clear();
  TEST_ASSERT_EQUAL(PhoneStore::ADDED, s.offer(car(1, 10, 0), 0));
  TEST_ASSERT_EQUAL(PhoneStore::REPLACED, s.offer(car(1, 11, 40), 40));
  TEST_ASSERT_EQUAL(1, s.pending());

  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 0;
  size_t n = s.takeBatch(buf, sizeof(buf), 1, 50, taken);
  TEST_ASSERT_EQUAL(1, taken);
  BatchHeader h;
  PhoneRecord out[1];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 1));
  TEST_ASSERT_EQUAL(11, out[0].frameId);
  TEST_ASSERT_EQUAL(0, s.pending());
}

void test_an_older_frame_does_not_replace_a_newer_one() {
  PhoneStore s;
  s.clear();
  s.offer(car(1, 20, 0), 0);
  TEST_ASSERT_EQUAL(PhoneStore::STALE, s.offer(car(1, 19, 5), 5));
  // Across the id wrap too.
  s.offer(car(2, 3, 0), 0);
  TEST_ASSERT_EQUAL(PhoneStore::STALE, s.offer(car(2, 0xFFFFFFFE, 5), 5));
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 0;
  size_t n = s.takeBatch(buf, sizeof(buf), 1, 10, taken);
  BatchHeader h;
  PhoneRecord out[2];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 2));
  TEST_ASSERT_EQUAL(20, out[0].frameId);
  TEST_ASSERT_EQUAL(3, out[1].frameId);
}

void test_a_replacement_keeps_its_place_in_the_flush() {
  PhoneStore s;
  s.clear();
  s.offer(car(1, 1, 0), 0);
  // The same car again and again must not hold its own flush off.
  for (uint32_t t = 20; t < 100; t += 20) s.offer(car(1, 1 + t, t), t);
  TEST_ASSERT_FALSE(s.due(99, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS));
  TEST_ASSERT_TRUE(s.due(100, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS));
}

void test_a_full_batch_goes_without_waiting() {
  PhoneStore s;
  s.clear();
  for (uint32_t i = 0; i < 8; i++) s.offer(car(i + 1, 1, 0), 0);
  TEST_ASSERT_FALSE(s.due(1, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS));
  s.offer(car(9, 1, 1), 1);
  TEST_ASSERT_TRUE(s.due(1, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS));
}

void test_an_empty_store_is_never_due_and_takes_nothing() {
  PhoneStore s;
  s.clear();
  TEST_ASSERT_FALSE(s.due(100000, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS));
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 99;
  TEST_ASSERT_EQUAL(0, s.takeBatch(buf, sizeof(buf), 1, 0, taken));
  TEST_ASSERT_EQUAL(0, taken);
}

void test_the_longest_waiting_cars_go_first() {
  PhoneStore s;
  s.clear();
  for (uint32_t i = 0; i < 12; i++) s.offer(car(100 + i, 1, i), i);
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 0;
  size_t n = s.takeBatch(buf, sizeof(buf), 1, 20, taken);
  TEST_ASSERT_EQUAL(9, taken);
  BatchHeader h;
  PhoneRecord out[9];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 9));
  for (uint32_t i = 0; i < 9; i++) TEST_ASSERT_EQUAL_HEX32(100 + i, out[i].node);
  TEST_ASSERT_EQUAL(3, s.pending());
}

void test_a_full_store_evicts_the_longest_waiting() {
  PhoneStore s;
  s.clear();
  for (uint32_t i = 0; i < PHONE_STORE_SLOTS; i++) s.offer(car(i + 1, 1, i), i);
  TEST_ASSERT_EQUAL(PhoneStore::EVICTED, s.offer(car(999, 1, 100), 100));
  TEST_ASSERT_EQUAL(PHONE_STORE_SLOTS, s.pending());
  // Car 1, queued first, is the one that went.
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 0;
  size_t n = s.takeBatch(buf, sizeof(buf), 1, 200, taken);
  BatchHeader h;
  PhoneRecord out[9];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 9));
  TEST_ASSERT_EQUAL_HEX32(2, out[0].node);
}

void test_in_flight_batches_are_bounded_and_expire() {
  BatchesInFlight f;
  f.clear();
  f.add(1, 5, 0);
  TEST_ASSERT_FALSE(f.full());
  f.add(2, 4, 100);
  TEST_ASSERT_TRUE(f.full());
  TEST_ASSERT_EQUAL(1000, f.oldestAgeMs(1000));
  TEST_ASSERT_EQUAL(5, f.delivered(1));
  TEST_ASSERT_EQUAL(0, f.delivered(1));
  TEST_ASSERT_EQUAL(900, f.oldestAgeMs(1000));
  TEST_ASSERT_EQUAL(4, f.expire(100 + BATCH_LOST_MS, BATCH_LOST_MS));
  TEST_ASSERT_EQUAL(0, f.count());
  f.add(3, 2, 0);
  f.add(4, 3, 0);
  TEST_ASSERT_EQUAL(5, f.pendingRecords());
}

void test_hello_round_trips_and_refuses_junk() {
  PhoneHello h;
  h.flags = HELLO_BATCHES | HELLO_PRELOAD;
  h.mtu = 517;
  uint8_t buf[8];
  TEST_ASSERT_EQUAL(HELLO_LEN, encodeHello(h, buf, sizeof(buf)));
  PhoneHello back;
  TEST_ASSERT_TRUE(decodeHello(buf, HELLO_LEN, back));
  TEST_ASSERT_EQUAL(h.flags, back.flags);
  TEST_ASSERT_EQUAL(517, back.mtu);
  TEST_ASSERT_FALSE(decodeHello(buf, HELLO_LEN - 1, back));
  const uint8_t voice[] = {0x54, 1, 0, 0, 0, 1};
  TEST_ASSERT_FALSE(decodeHello(voice, sizeof(voice), back));
}

static LinkStats maxed() {
  LinkStats s;
  const uint32_t m = 0xFFFFFFFFu;
  s.fast.tx = s.fast.rx = s.fast.suppressed = s.fast.queued = s.fast.delivered = m;
  s.lora.rx = s.lora.delivered = m;
  s.fastTxFail = s.batches = s.batchesRead = s.replaced = m;
  s.dropStoreFull = s.dropAlloc = s.dropLost = s.dropDisconnect = s.dropStale = m;
  s.coreReplaced = s.coreDropped = s.coreEvicted = s.writeDropped = s.writeDuplicate = m;
  s.preloadOffered = s.preloadRead = s.preloadRefused = s.minFreeHeap = s.oldestQueuedMs = m;
  s.queueDepth = s.queueDepthMax = s.storePending = 0xFFFF;
  return s;
}

// Every counter at its limit still fits one Meshtastic payload.
void test_stats_fit_a_payload_at_their_worst() {
  char buf[BATCH_MAX_PAYLOAD];
  LinkStats s = maxed();
  TEST_ASSERT_TRUE(formatLaneStats(s, buf, sizeof(buf)) > 0);
  TEST_ASSERT_TRUE(formatQueueStats(s, buf, sizeof(buf)) > 0);
}

void test_stats_format_as_the_app_reads_them() {
  LinkStats s;
  s.fast.tx = 1;
  s.fast.rx = 2;
  s.fast.suppressed = 3;
  s.fast.queued = 4;
  s.fast.delivered = 5;
  s.lora.rx = 6;
  s.lora.delivered = 7;
  s.batches = 8;
  s.batchesRead = 9;
  s.replaced = 10;
  s.fastTxFail = 11;
  char buf[BATCH_MAX_PAYLOAD];
  formatLaneStats(s, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"fs\":[1,1,2,3,4,5,6,7,8,9,10,11]}", buf);

  LinkStats q;
  q.queueDepth = 3;
  q.queueDepthMax = 31;
  q.storePending = 2;
  q.oldestQueuedMs = 140;
  q.dropStoreFull = 1;
  q.dropAlloc = 2;
  q.dropLost = 3;
  q.dropDisconnect = 4;
  q.dropStale = 5;
  q.coreReplaced = 6;
  q.coreDropped = 7;
  q.writeDropped = 8;
  q.writeDuplicate = 9;
  q.preloadOffered = 10;
  q.preloadRead = 11;
  q.preloadRefused = 12;
  q.minFreeHeap = 40000;
  q.coreEvicted = 13;
  formatQueueStats(q, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"fq\":[1,3,31,2,140,1,2,3,4,5,6,7,8,9,10,11,12,40000,13]}", buf);
}

void test_baseline_reports_rates_over_the_window() {
  LinkStats before, now;
  now.fast.rx = 40;      // 8.0/s over 5 s
  now.fast.delivered = 39;
  now.batches = 12;      // 2.4/s
  now.queueDepth = 1;
  now.coreDropped = 2;
  char buf[256];
  TEST_ASSERT_TRUE(formatBaseline(now, before, 5000, buf, sizeof(buf)) > 0);
  TEST_ASSERT_NOT_NULL(strstr(buf, "BASELINE radio"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "rx=8.0"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "delivered=7.8/s"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "batches=2.4/s"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "drops=2"));
}

struct SimResult {
  uint32_t heard = 0;
  uint32_t delivered = 0;
  uint32_t superseded = 0;
  uint32_t leftInPipe = 0;
  uint32_t reads = 0;
  uint32_t worstWaitFirstMinute = 0;
  uint32_t worstWaitLastMinute = 0;
};

// [peers] cars at 1 Hz, spread across the second like slots in a schedule. The
// phone reads whenever a batch is waiting and each read takes [readMs].
static SimResult simulate(uint32_t peers, uint32_t readMs, uint32_t runMs) {
  PhoneStore s;
  s.clear();
  BatchesInFlight f;
  f.clear();
  uint32_t frameId[32] = {0};
  uint32_t nextBeacon[32];
  for (uint32_t i = 0; i < peers; i++) nextBeacon[i] = i * 1000 / peers;

  SimResult r;
  uint16_t seq = 0;
  uint32_t readBusyUntil = 0;
  uint8_t buf[BATCH_MAX_PAYLOAD];
  for (uint32_t now = 0; now < runMs; now += 5) {
    for (uint32_t i = 0; i < peers; i++) {
      if ((int32_t)(now - nextBeacon[i]) < 0) continue;
      nextBeacon[i] += 1000;
      r.heard++;
      PhoneStore::Offer o = s.offer(car(i + 1, ++frameId[i], now), now);
      TEST_ASSERT_NOT_EQUAL(PhoneStore::EVICTED, o);
      if (o == PhoneStore::REPLACED) r.superseded++;
    }
    if (f.count() > 0 && (int32_t)(now - readBusyUntil) >= 0) {
      r.delivered += f.delivered((uint16_t)(seq - f.count()));
      r.reads++;
      readBusyUntil = now + readMs;
    }
    if (!f.full() && s.due(now, BATCH_MAX_PAYLOAD, BATCH_FLUSH_MS)) {
      size_t taken = 0;
      if (s.takeBatch(buf, sizeof(buf), seq, now, taken) > 0) f.add(seq++, (uint8_t)taken, now);
    }
    uint32_t wait = s.oldestWaitMs(now);
    if (f.oldestAgeMs(now) > wait) wait = f.oldestAgeMs(now);
    if (now < 60000 && wait > r.worstWaitFirstMinute) r.worstWaitFirstMinute = wait;
    if (now >= runMs - 60000 && wait > r.worstWaitLastMinute) r.worstWaitLastMinute = wait;
  }
  r.leftInPipe = s.pending() + f.pendingRecords();
  return r;
}

// SCALE-PLAN step 3's test point: 8 simulated peers at 1 Hz for ten minutes.
// Nothing lost, nothing superseded, and the wait flat from the first minute to
// the last. Arrivals 125 ms apart rarely share a 100 ms window, so at this size
// reads stay near one per car; batching earns its keep at 24 (next test).
void test_eight_peers_at_1hz_for_ten_minutes() {
  SimResult r = simulate(8, 15, 10 * 60 * 1000);
  TEST_ASSERT_EQUAL(r.heard, r.delivered + r.leftInPipe);
  TEST_ASSERT_EQUAL(0, r.superseded);
  TEST_ASSERT_TRUE(r.worstWaitLastMinute <= BATCH_FLUSH_MS + 15);
  TEST_ASSERT_TRUE(r.worstWaitLastMinute <= r.worstWaitFirstMinute);
  TEST_ASSERT_TRUE(r.reads <= r.heard);
}

// 24 peers, the 25-car target. Several cars per read, and every one delivered.
void test_twenty_four_peers_share_reads() {
  SimResult r = simulate(24, 15, 10 * 60 * 1000);
  TEST_ASSERT_EQUAL(r.heard, r.delivered + r.leftInPipe);
  const uint32_t readsPerSec = r.reads / 600;
  char msg[80];
  snprintf(msg, sizeof(msg), "%u reads/s for 24 positions/s", (unsigned)readsPerSec);
  TEST_ASSERT_TRUE_MESSAGE(readsPerSec <= 10, msg);
  TEST_ASSERT_TRUE(r.worstWaitLastMinute <= BATCH_FLUSH_MS + 2 * 15);
}

// A reader ten times slower than it should be. The pipe self-clocks: batches
// fill up instead of queueing, older positions are replaced by newer ones, and
// the wait stays flat rather than growing for ten minutes.
void test_a_slow_reader_gets_fuller_batches_not_a_backlog() {
  SimResult r = simulate(24, 150, 10 * 60 * 1000);
  TEST_ASSERT_EQUAL(r.heard, r.delivered + r.superseded + r.leftInPipe);
  TEST_ASSERT_TRUE(r.reads / 600 <= 7);
  TEST_ASSERT_TRUE(r.worstWaitLastMinute <= r.worstWaitFirstMinute + 5);
  TEST_ASSERT_TRUE(r.worstWaitLastMinute < 1000);
}

// The same eight peers with a phone that stops reading for 5 s. The store holds
// one position per car, so after the stall the phone gets each car's newest,
// not five seconds of history.
void test_a_stalled_reader_gets_the_newest_not_the_backlog() {
  PhoneStore s;
  s.clear();
  for (uint32_t t = 0; t < 5000; t += 125) {
    const uint32_t peer = (t / 125) % 8;
    s.offer(car(peer + 1, t, t), t);
  }
  TEST_ASSERT_EQUAL(8, s.pending());
  uint8_t buf[BATCH_MAX_PAYLOAD];
  size_t taken = 0;
  size_t n = s.takeBatch(buf, sizeof(buf), 1, 5000, taken);
  BatchHeader h;
  PhoneRecord out[9];
  TEST_ASSERT_TRUE(decodeBatch(buf, n, h, out, 9));
  for (size_t i = 0; i < taken; i++) TEST_ASSERT_TRUE(out[i].frameId >= 5000 - 1000);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_batch_round_trips_every_field);
  RUN_TEST(test_a_known_batch_encodes_to_the_pinned_bytes);
  RUN_TEST(test_the_first_byte_cannot_be_json_or_voice);
  RUN_TEST(test_a_truncated_batch_is_rejected_whole);
  RUN_TEST(test_the_header_alone_names_the_batch);
  RUN_TEST(test_junk_is_not_a_batch);
  RUN_TEST(test_more_records_than_the_caller_holds_is_rejected);
  RUN_TEST(test_a_longer_future_record_still_decodes);
  RUN_TEST(test_budget_follows_the_mtu_and_stops_at_the_payload_limit);
  RUN_TEST(test_a_second_record_for_a_car_replaces_the_first);
  RUN_TEST(test_an_older_frame_does_not_replace_a_newer_one);
  RUN_TEST(test_a_replacement_keeps_its_place_in_the_flush);
  RUN_TEST(test_a_full_batch_goes_without_waiting);
  RUN_TEST(test_an_empty_store_is_never_due_and_takes_nothing);
  RUN_TEST(test_the_longest_waiting_cars_go_first);
  RUN_TEST(test_a_full_store_evicts_the_longest_waiting);
  RUN_TEST(test_in_flight_batches_are_bounded_and_expire);
  RUN_TEST(test_hello_round_trips_and_refuses_junk);
  RUN_TEST(test_stats_fit_a_payload_at_their_worst);
  RUN_TEST(test_stats_format_as_the_app_reads_them);
  RUN_TEST(test_baseline_reports_rates_over_the_window);
  RUN_TEST(test_eight_peers_at_1hz_for_ten_minutes);
  RUN_TEST(test_twenty_four_peers_share_reads);
  RUN_TEST(test_a_slow_reader_gets_fuller_batches_not_a_backlog);
  RUN_TEST(test_a_stalled_reader_gets_the_newest_not_the_backlog);
  return UNITY_END();
}
