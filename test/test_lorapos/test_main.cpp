// Host tests for LoRa positions (SCALE-PLAN 5b and 5c): this car's interval,
// and one position per car in the queues.
//
// The TX queue here is a plain FIFO driven by the same rules the core patch
// applies (core-patches/0010): place() first, then the stock capacity check.

#include <unity.h>
#include <string.h>
#include <vector>
#include "lorapos.h"

using namespace touge;

void setUp() {}
void tearDown() {}

// ---- The interval (5b) --------------------------------------------------------
//
// The app's PingPacingTest pins the same numbers for Convoy.loraIntervalMs.

void test_the_interval_is_5_s_until_the_air_says_otherwise() {
  // SHORT_FAST, about 58 ms a position.
  TEST_ASSERT_EQUAL_UINT32(5000, loraIntervalMs(1, 58));
  TEST_ASSERT_EQUAL_UINT32(5000, loraIntervalMs(3, 58));
  // 8 x 8 x 58 ms is 3.7 s of positions a round, 30 % of 12.4 s.
  TEST_ASSERT_EQUAL_UINT32(12373, loraIntervalMs(8, 58));
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, loraIntervalMs(30, 58));
  // LONG_FAST: three cars already want more than the cap.
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, loraIntervalMs(3, 760));
  // Nobody heard yet, or no radio to ask for an airtime: the target.
  TEST_ASSERT_EQUAL_UINT32(5000, loraIntervalMs(0, 58));
  TEST_ASSERT_EQUAL_UINT32(5000, loraIntervalMs(25, 0));
}

void test_the_interval_never_shrinks_as_cars_join() {
  uint32_t last = 0;
  for (uint32_t cars = 1; cars <= 40; cars++) {
    const uint32_t ms = loraIntervalMs(cars, 58);
    TEST_ASSERT_TRUE(ms >= last);
    TEST_ASSERT_TRUE(ms >= LORA_TARGET_MS && ms <= LORA_MAX_MS);
    last = ms;
  }
}

// ---- Fix identity out of Meshtastic's Position ---------------------------------

void test_the_identity_is_read_from_meshtastics_fields() {
  const FixId named = fixIdOf(0xA5C3, 7, 1790000000, 250, 1790000009);
  TEST_ASSERT_EQUAL_UINT16(0xA5C3, named.session);
  TEST_ASSERT_EQUAL_UINT32(7, named.seq);
  TEST_ASSERT_EQUAL_UINT32(1790000000, named.fixSec);
  TEST_ASSERT_EQUAL_UINT16(250, named.fixMs);
  // A stock sender: no session, and the time is all there is.
  const FixId stock = fixIdOf(0, 12, 0, 0, 1790000009);
  TEST_ASSERT_EQUAL_UINT16(0, stock.session);
  TEST_ASSERT_EQUAL_UINT32(1790000009, stock.fixSec);
  // A sensor_id wider than a session is not one of ours.
  TEST_ASSERT_EQUAL_UINT16(0, fixIdOf(0x10001, 1, 1790000000, 0, 0).session);
}

static FixId fixId(uint16_t session, uint32_t seq, uint32_t fixSec, uint16_t fixMs = 0) {
  FixId f;
  f.session = session;
  f.seq = seq;
  f.fixSec = fixSec;
  f.fixMs = fixMs;
  return f;
}

void test_without_an_identity_the_later_arrival_wins() {
  const FixId stock = fixId(0, 0, 1790000100);
  const FixId touge = fixId(7, 5, 1790000100);
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankQueued(stock, touge));
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankQueued(touge, stock));
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankQueued(stock, stock));
  // Both named: rankFix decides, so a late older copy stays behind.
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankQueued(fixId(7, 4, 1790000099), touge));
  TEST_ASSERT_EQUAL(FixRank::SAME, rankQueued(touge, touge));
}

// ---- One position per car in the TX queue (5c) ---------------------------------

struct SimQueue {
  TxPositions tags;
  std::vector<QueuedPacket> q;
  uint32_t replaced = 0;
  uint32_t refused = 0;
  uint32_t full = 0;

  SimQueue() { tags.clear(); }

  static bool inQueue(uint32_t from, uint32_t id, void* ctx) {
    const SimQueue* self = (const SimQueue*)ctx;
    for (size_t i = 0; i < self->q.size(); i++) {
      if (self->q[i].from == from && self->q[i].id == id) return true;
    }
    return false;
  }

  // A packet on its way to the queue; a position is noted first, as the module
  // does before the router queues it.
  TxPlace offer(uint32_t from, uint32_t id, uint8_t hops, const FixId* fix) {
    if (fix != nullptr) tags.note(from, id, *fix, &SimQueue::inQueue, this);
    QueuedPacket in;
    in.from = from;
    in.id = id;
    in.hopLimit = hops;
    size_t at = 0;
    const TxPlace place = tags.place(in, q.size(), [this](size_t i) { return q[i]; }, at);
    if (place == TxPlace::REFUSE) {
      refused++;
      return place;
    }
    if (place == TxPlace::REPLACE) {
      q[at] = in;
      replaced++;
      return place;
    }
    if (q.size() >= TxPositions::TX_QUEUE_LEN) {
      full++;
      return place;
    }
    q.push_back(in);
    return place;
  }

  bool send(QueuedPacket& out) {
    if (q.empty()) return false;
    out = q.front();
    q.erase(q.begin());
    return true;
  }

  size_t queuedFrom(uint32_t from) const {
    size_t n = 0;
    for (size_t i = 0; i < q.size(); i++) n += q[i].from == from ? 1 : 0;
    return n;
  }
};

static const uint32_t CAR_A = 0xA1;
static const uint32_t CAR_B = 0xB2;
static const uint32_t CAR_C = 0xC3;

void test_a_newer_position_takes_the_older_ones_place_and_turn() {
  SimQueue sim;
  const FixId a1 = fixId(1, 1, 1790000000), b1 = fixId(2, 1, 1790000000), c1 = fixId(3, 1, 1790000000);
  sim.offer(CAR_A, 10, 3, &a1);
  sim.offer(CAR_B, 20, 3, &b1);
  sim.offer(CAR_C, 30, 3, &c1);
  const FixId b2 = fixId(2, 2, 1790000005);
  TEST_ASSERT_EQUAL(TxPlace::REPLACE, sim.offer(CAR_B, 21, 2, &b2));
  TEST_ASSERT_EQUAL_UINT32(3, sim.q.size());
  // B keeps its turn, second, with the newer packet and its own hop budget.
  TEST_ASSERT_EQUAL_UINT32(CAR_B, sim.q[1].from);
  TEST_ASSERT_EQUAL_UINT32(21, sim.q[1].id);
  TEST_ASSERT_EQUAL_UINT8(2, sim.q[1].hopLimit);
}

void test_a_late_older_copy_is_refused() {
  SimQueue sim;
  const FixId b2 = fixId(2, 2, 1790000005), b1 = fixId(2, 1, 1790000000);
  sim.offer(CAR_B, 21, 3, &b2);
  TEST_ASSERT_EQUAL(TxPlace::REFUSE, sim.offer(CAR_B, 20, 3, &b1));
  TEST_ASSERT_EQUAL_UINT32(1, sim.q.size());
  TEST_ASSERT_EQUAL_UINT32(21, sim.q[0].id);
}

void test_the_same_fix_twice_keeps_the_copy_that_travels_further() {
  SimQueue sim;
  const FixId b = fixId(2, 5, 1790000005);
  sim.offer(CAR_B, 40, 1, &b);
  TEST_ASSERT_EQUAL(TxPlace::REPLACE, sim.offer(CAR_B, 41, 2, &b));
  TEST_ASSERT_EQUAL(TxPlace::REFUSE, sim.offer(CAR_B, 42, 2, &b));
  TEST_ASSERT_EQUAL_UINT32(1, sim.q.size());
  TEST_ASSERT_EQUAL_UINT32(41, sim.q[0].id);
}

void test_a_rebooted_car_replaces_its_old_session() {
  SimQueue sim;
  const FixId before = fixId(7, 5000, 1790000100, 250), after = fixId(9, 1, 1790000130);
  sim.offer(CAR_B, 50, 3, &before);
  TEST_ASSERT_EQUAL(TxPlace::REPLACE, sim.offer(CAR_B, 51, 3, &after));
  TEST_ASSERT_EQUAL_UINT32(51, sim.q[0].id);
}

void test_text_and_stock_positions_queue_the_stock_way() {
  SimQueue sim;
  const FixId b1 = fixId(2, 1, 1790000000);
  sim.offer(CAR_B, 20, 3, &b1);
  // A text from the same car: never noted, so never a rival.
  TEST_ASSERT_EQUAL(TxPlace::QUEUE, sim.offer(CAR_B, 60, 3, nullptr));
  // A stock sender's position has no identity and is not noted either.
  const FixId stock = fixId(0, 3, 1790000010);
  TEST_ASSERT_EQUAL(TxPlace::QUEUE, sim.offer(CAR_C, 70, 3, &stock));
  TEST_ASSERT_EQUAL(TxPlace::QUEUE, sim.offer(CAR_C, 71, 3, &stock));
  TEST_ASSERT_EQUAL_UINT32(4, sim.q.size());
}

void test_the_same_packet_again_is_not_its_own_rival() {
  // Meshtastic takes a packet out and queues it again (the late rebroadcast
  // window); its own tag must not refuse it.
  SimQueue sim;
  const FixId b1 = fixId(2, 1, 1790000000);
  sim.offer(CAR_B, 20, 3, &b1);
  QueuedPacket out;
  TEST_ASSERT_TRUE(sim.send(out));
  TEST_ASSERT_EQUAL(TxPlace::QUEUE, sim.offer(CAR_B, 20, 3, &b1));
  // And with a copy of itself still queued, it is not refused either.
  TEST_ASSERT_EQUAL(TxPlace::QUEUE, sim.offer(CAR_B, 20, 3, &b1));
}

void test_tags_reuse_places_no_longer_queued() {
  SimQueue sim;
  // Far more positions than places, sent as they come: every one is still found
  // while queued.
  for (uint32_t i = 1; i <= 200; i++) {
    const FixId fix = fixId(1, i, 1790000000 + i);
    sim.offer(0x1000 + (i % 25), i, 3, &fix);
    if (i % 3 == 0) {
      QueuedPacket out;
      sim.send(out);
    }
    for (size_t k = 0; k < sim.q.size(); k++) {
      TEST_ASSERT_NOT_NULL(sim.tags.find(sim.q[k].from, sim.q[k].id));
    }
  }
}

static bool alwaysQueued(uint32_t, uint32_t, void*) { return true; }

void test_a_note_with_every_place_queued_is_dropped() {
  TxPositions tags;
  tags.clear();
  for (uint32_t i = 1; i <= TxPositions::SLOTS; i++) tags.note(i, i, fixId(1, i, 1790000000), &alwaysQueued, nullptr);
  for (uint32_t i = 1; i <= TxPositions::SLOTS; i++) TEST_ASSERT_NOT_NULL(tags.find(i, i));
  tags.note(99, 99, fixId(1, 99, 1790000000), &alwaysQueued, nullptr);
  TEST_ASSERT_NULL(tags.find(99, 99));
  // Noting a packet already held updates it in place.
  tags.note(3, 3, fixId(1, 300, 1790000300), &alwaysQueued, nullptr);
  TEST_ASSERT_EQUAL_UINT32(300, tags.find(3, 3)->seq);
}

// A busy car next to us sends every second, five more every 5 s, and a car at
// the rear of the convoy is relayed through us every 5 s. The channel sends one
// packet every 700 ms, so the queue backs up. Each car has at most one position
// queued, always its newest, and the rear car gets every one of its positions
// out: the queue goes round the cars, not round the packets.
// The newest sequence offered for each of the cars below.
struct Newest {
  uint32_t origin[8] = {0};
  uint32_t seq[8] = {0};
  uint32_t& operator[](uint32_t from) {
    size_t i = 0;
    while (i < 7 && origin[i] != from && origin[i] != 0) i++;
    origin[i] = from;
    return seq[i];
  }
};

void test_under_a_backlog_the_rear_car_keeps_its_turn() {
  SimQueue sim;
  const uint32_t BUSY = 0xB05;
  const uint32_t REAR = 0x4EA4;
  Newest newest;
  uint32_t nextId = 1;
  uint32_t rearOffered = 0, rearSent = 0, busySent = 0;

  for (uint32_t ms = 0; ms < 300000; ms += 100) {
    const uint32_t sec = 1790000000 + ms / 1000;
    if (ms % 1000 == 0) {
      const FixId fix = fixId(0xB5, ++newest[BUSY], sec);
      sim.offer(BUSY, nextId++, 3, &fix);
    }
    if (ms % 5000 == 0) {
      for (uint32_t car = 0xC0; car < 0xC5; car++) {
        const FixId fix = fixId((uint16_t)car, ++newest[car], sec);
        sim.offer(car, nextId++, 3, &fix);
      }
    }
    if (ms % 5000 == 2500) {
      const FixId fix = fixId(0x4E, ++newest[REAR], sec);
      sim.offer(REAR, nextId++, 1, &fix);
      rearOffered++;
    }

    for (size_t i = 0; i < sim.q.size(); i++) {
      TEST_ASSERT_EQUAL_UINT32(1, sim.queuedFrom(sim.q[i].from));
      TEST_ASSERT_EQUAL_UINT32(newest[sim.q[i].from], sim.tags.find(sim.q[i].from, sim.q[i].id)->seq);
    }

    if (ms % 700 == 0) {
      QueuedPacket out;
      if (sim.send(out)) {
        if (out.from == REAR) rearSent++;
        if (out.from == BUSY) busySent++;
      }
    }
  }
  TEST_ASSERT_EQUAL_UINT32(0, sim.full);
  TEST_ASSERT_EQUAL_UINT32(0, sim.refused);
  // At most the last one still waiting when the run ends.
  TEST_ASSERT_TRUE(rearSent + 1 >= rearOffered);
  // The busy car gets a turn a round, not one a second.
  TEST_ASSERT_TRUE(busySent < 300 / 3);
  TEST_ASSERT_TRUE(busySent > 300 / 7);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_interval_is_5_s_until_the_air_says_otherwise);
  RUN_TEST(test_the_interval_never_shrinks_as_cars_join);
  RUN_TEST(test_the_identity_is_read_from_meshtastics_fields);
  RUN_TEST(test_without_an_identity_the_later_arrival_wins);
  RUN_TEST(test_a_newer_position_takes_the_older_ones_place_and_turn);
  RUN_TEST(test_a_late_older_copy_is_refused);
  RUN_TEST(test_the_same_fix_twice_keeps_the_copy_that_travels_further);
  RUN_TEST(test_a_rebooted_car_replaces_its_old_session);
  RUN_TEST(test_text_and_stock_positions_queue_the_stock_way);
  RUN_TEST(test_the_same_packet_again_is_not_its_own_rival);
  RUN_TEST(test_tags_reuse_places_no_longer_queued);
  RUN_TEST(test_a_note_with_every_place_queued_is_dropped);
  RUN_TEST(test_under_a_backlog_the_rear_car_keeps_its_turn);
  return UNITY_END();
}
