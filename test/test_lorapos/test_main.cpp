// Host tests for LoRa positions (SCALE-PLAN 5b and 5c): the identity they
// carry, and one position per car in the queues. The interval they go out at is
// measured load now (5d, test_loraload). From build 43, how ours goes out:
// ahead of relays, and unsigned.
//
// The TX queue in the 5c tests is a plain FIFO driven by the same rules the
// core patch applies (core-patches/0010): place() first, then the stock
// capacity check.

#include <unity.h>
#include <algorithm>
#include <string.h>
#include <vector>
#include "lorapos.h"

using namespace touge;

void setUp() {}
void tearDown() {}

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
  uint32_t nowMs = 0;

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
    if (fix != nullptr) tags.note(from, id, *fix, nowMs, &SimQueue::inQueue, this);
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
  for (uint32_t i = 1; i <= TxPositions::SLOTS; i++) tags.note(i, i, fixId(1, i, 1790000000), 0, &alwaysQueued, nullptr);
  for (uint32_t i = 1; i <= TxPositions::SLOTS; i++) TEST_ASSERT_NOT_NULL(tags.find(i, i));
  tags.note(99, 99, fixId(1, 99, 1790000000), 0, &alwaysQueued, nullptr);
  TEST_ASSERT_NULL(tags.find(99, 99));
  // Noting a packet already held updates it in place.
  tags.note(3, 3, fixId(1, 300, 1790000300), 0, &alwaysQueued, nullptr);
  TEST_ASSERT_EQUAL_UINT32(300, tags.find(3, 3)->seq);
}

// 5d reads a position's TX queue wait off its note as it leaves the queue.
void test_a_noted_position_knows_how_long_it_has_waited() {
  TxPositions tags;
  tags.clear();
  tags.note(CAR_B, 20, fixId(2, 1, 1790000000), 1000, &alwaysQueued, nullptr);
  uint32_t waited = 0;
  TEST_ASSERT_TRUE(tags.waited(CAR_B, 20, 1750, waited));
  TEST_ASSERT_EQUAL_UINT32(750, waited);
  // Across millis() wrapping.
  tags.note(CAR_C, 30, fixId(3, 1, 1790000000), 0xFFFFFF00u, &alwaysQueued, nullptr);
  TEST_ASSERT_TRUE(tags.waited(CAR_C, 30, 0x100, waited));
  TEST_ASSERT_EQUAL_UINT32(0x200, waited);
  // A packet never noted (text, a stock position) has no wait to give.
  TEST_ASSERT_FALSE(tags.waited(CAR_B, 21, 1750, waited));
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

// ---- Our position ahead of relays (build 43) -----------------------------------
//
// One radio's LoRa TX path as Meshtastic runs it (RadioLibInterface.cpp): the
// queue in CompareMeshPacketFunc's order; one notification slot for the
// transmit timer, which notifyLater does not overwrite; a relay at the head
// drawing its SNR-weighted delay (getTxDelayMsecWeighted, 128 ms plus up to
// 2 s at the strong signals of a bench) and a packet of ours the contention
// delay (getTxDelayMsec, 0-56 ms); nothing sent while receiving or sending; and
// the timer re-armed for the head after every reception and transmission, as
// the interrupt takes the slot. Build 43 is OWN_POSITION_PRIORITY and, for our
// position arriving at the head, a pending timer pulled in to its own
// contention delay (core-patches/0014, notifySooner).

namespace txpath {

const uint32_t AIR_MS = 66;  // an unsigned position on SHORT_FAST
const uint32_t SLOT_MS = 8;  // RadioInterface::computeSlotTimeMsec on SHORT_FAST
const uint8_t CW_MIN = 3;
const uint8_t CW_MAX = 8;
const uint32_t ROUND_MS = 5000;

struct Packet {
  uint32_t id = 0;
  bool ours = false;
  uint8_t priority = 0;
  uint32_t queuedMs = 0;
  uint8_t cw = 0;  // a relay's contention window, from the SNR it was heard at
};

// CompareMeshPacketFunc (MeshPacketQueue.cpp) without the late window: the
// higher priority first, and at equal priority another car's before ours.
bool goesBefore(const Packet& a, const Packet& b) {
  if (a.priority != b.priority) return a.priority > b.priority;
  return !a.ours && b.ours;
}

struct Radio {
  bool build43 = false;
  uint32_t rng = 1;
  std::vector<Packet> queue;
  bool timerArmed = false;
  uint32_t timerAtMs = 0;
  uint32_t receivingUntilMs = 0;
  bool sending = false;
  uint32_t sentAtMs = 0;
  std::vector<uint32_t> ownWaits;
  uint32_t relaysQueued = 0;
  uint32_t relaysSent = 0;
  uint32_t ownLate = 0;
  uint32_t lastOwnId = 0;
  uint32_t nextId = 1;

  uint32_t random() {
    rng = rng * 1103515245u + 12345u;
    return rng >> 8;
  }
  uint32_t contentionMs() { return (random() % (1u << CW_MIN)) * SLOT_MS; }
  uint32_t headDelayMs() {
    const Packet& head = queue.front();
    if (head.ours) return contentionMs();
    return 2 * CW_MAX * SLOT_MS + (random() % (1u << head.cw)) * SLOT_MS;
  }
  // setTransmitDelay: arms the timer for the head, unless one is pending.
  void armForHead(uint32_t nowMs) {
    if (queue.empty() || timerArmed) return;
    timerArmed = true;
    timerAtMs = nowMs + headDelayMs();
  }
  // An RX or TX interrupt takes the one slot; onNotify then re-arms for the head.
  void interrupt(uint32_t nowMs) {
    timerArmed = false;
    armForHead(nowMs);
  }
  // RadioLibInterface::send.
  void send(const Packet& p, uint32_t nowMs) {
    size_t at = 0;
    while (at < queue.size() && !goesBefore(p, queue[at])) at++;
    queue.insert(queue.begin() + at, p);
    if (build43 && p.ours && at == 0 && timerArmed) {
      const uint32_t soonMs = contentionMs();
      if ((int32_t)(timerAtMs - nowMs) > (int32_t)soonMs) {
        timerAtMs = nowMs + soonMs;
        return;
      }
    }
    armForHead(nowMs);
  }
  void ownPosition(uint32_t nowMs) {
    for (size_t i = 0; i < queue.size(); i++) {
      if (queue[i].id == lastOwnId) ownLate++;
    }
    Packet p;
    p.id = nextId++;
    p.ours = true;
    // Build 41's own position went at BACKGROUND, as PositionModule's.
    p.priority = build43 ? OWN_POSITION_PRIORITY : PRIORITY_BACKGROUND;
    p.queuedMs = nowMs;
    lastOwnId = p.id;
    send(p, nowMs);
  }
  void relay(uint8_t cw, uint32_t nowMs) {
    Packet p;
    p.id = nextId++;
    p.priority = PRIORITY_DEFAULT;
    p.queuedMs = nowMs;
    p.cw = cw;
    relaysQueued++;
    send(p, nowMs);
  }
  void tick(uint32_t nowMs) {
    if (sending && nowMs >= sentAtMs + AIR_MS) {
      sending = false;
      interrupt(nowMs);
    }
    if (!timerArmed || nowMs < timerAtMs) return;
    timerArmed = false;
    if (queue.empty()) return;
    if (nowMs < receivingUntilMs || sending) {
      armForHead(nowMs);
      return;
    }
    const Packet p = queue.front();
    queue.erase(queue.begin());
    if (p.ours) {
      ownWaits.push_back(nowMs - p.queuedMs);
    } else {
      relaysSent++;
    }
    sending = true;
    sentAtMs = nowMs;
  }
};

struct Heard {
  uint32_t atMs = 0;
  bool toRelay = false;  // a car's position we relay; else another car's relay of one
  uint8_t cw = 0;
};

// The bench (build 41): two other cars in direct range, each sending a position
// every round that we queue a relay for, and each relaying the other's once.
// Ours goes every round at 1 s into it. Then ten seconds for the queue to drain.
Radio ride(bool build43, uint32_t seed, uint32_t rounds) {
  Radio radio;
  radio.build43 = build43;
  radio.rng = seed;
  uint32_t traffic = seed * 31 + 1;
  auto next = [&traffic]() {
    traffic = traffic * 1103515245u + 12345u;
    return traffic >> 8;
  };
  std::vector<Heard> starts;
  for (uint32_t r = 0; r < rounds; r++) {
    for (int car = 0; car < 2; car++) {
      Heard position;
      position.atMs = r * ROUND_MS + next() % ROUND_MS;
      position.toRelay = true;
      position.cw = CW_MAX;
      starts.push_back(position);
      Heard relayed;
      relayed.atMs = position.atMs + AIR_MS + 2 * CW_MAX * SLOT_MS + (next() % 256) * SLOT_MS;
      starts.push_back(relayed);
    }
  }
  std::stable_sort(starts.begin(), starts.end(), [](const Heard& a, const Heard& b) { return a.atMs < b.atMs; });

  std::vector<Heard> ends;
  size_t nextStart = 0;
  const uint32_t endMs = rounds * ROUND_MS + 10000;
  for (uint32_t nowMs = 0; nowMs < endMs; nowMs++) {
    for (size_t i = 0; i < ends.size();) {
      if (ends[i].atMs != nowMs) {
        i++;
        continue;
      }
      const Heard done = ends[i];
      ends.erase(ends.begin() + i);
      radio.interrupt(nowMs);
      if (done.toRelay) radio.relay(done.cw, nowMs);
    }
    for (; nextStart < starts.size() && starts[nextStart].atMs <= nowMs; nextStart++) {
      // Half duplex: a packet that starts while we transmit is lost to us.
      if (radio.sending) continue;
      Heard done = starts[nextStart];
      done.atMs = nowMs + AIR_MS;
      if (done.atMs > radio.receivingUntilMs) radio.receivingUntilMs = done.atMs;
      ends.push_back(done);
    }
    if (nowMs % ROUND_MS == 1000 && nowMs < rounds * ROUND_MS) radio.ownPosition(nowMs);
    radio.tick(nowMs);
  }
  return radio;
}

uint32_t countOver(const std::vector<uint32_t>& waits, uint32_t ms) {
  uint32_t n = 0;
  for (size_t i = 0; i < waits.size(); i++) n += waits[i] > ms ? 1 : 0;
  return n;
}

}  // namespace txpath

// Build 41 on the model: our BACKGROUND position waits behind relays and their
// SNR delays, as it did on the bench (median 0.6 s, p90 4.5 s, 6.2 s at worst).
void test_behind_relays_our_position_waited_seconds() {
  const uint32_t seeds[] = {7, 11, 12345};
  for (size_t s = 0; s < 3; s++) {
    const txpath::Radio radio = txpath::ride(false, seeds[s], 120);
    TEST_ASSERT_EQUAL_UINT32(120, radio.ownWaits.size());
    // A fifth or more waited over a second, and some were still queued when the
    // next was due.
    TEST_ASSERT_TRUE(txpath::countOver(radio.ownWaits, 1000) >= 24);
    TEST_ASSERT_TRUE(radio.ownLate >= 5);
  }
}

// Build 43: ahead of every relay, delayed ones included, our position goes
// within its contention delay and whatever reception or transmission is under
// way; none is late. Every relay still goes.
void test_our_position_leaves_ahead_of_queued_relays_and_relays_still_go() {
  const uint32_t seeds[] = {7, 11, 12345};
  for (size_t s = 0; s < 3; s++) {
    const txpath::Radio radio = txpath::ride(true, seeds[s], 120);
    TEST_ASSERT_EQUAL_UINT32(120, radio.ownWaits.size());
    TEST_ASSERT_EQUAL_UINT32(0, txpath::countOver(radio.ownWaits, 400));
    std::vector<uint32_t> sorted = radio.ownWaits;
    std::sort(sorted.begin(), sorted.end());
    TEST_ASSERT_TRUE(sorted[sorted.size() / 2] <= 100);
    TEST_ASSERT_EQUAL_UINT32(0, radio.ownLate);
    TEST_ASSERT_TRUE(radio.relaysQueued >= 200);
    TEST_ASSERT_EQUAL_UINT32(radio.relaysQueued, radio.relaysSent);
  }
}

// One packet at a time: ours goes ahead of a relay still waiting out its SNR
// delay, and the timer that relay started is pulled in to ours. Build 41 left
// ours behind the relay with the relay's timer running.
void test_our_position_goes_ahead_of_a_relay_waiting_out_its_delay() {
  for (int build43 = 0; build43 <= 1; build43++) {
    txpath::Radio radio;
    radio.build43 = build43 != 0;
    radio.rng = 5;
    // A relay of a strong signal: 128 ms and up before it may go.
    radio.relay(txpath::CW_MAX, 0);
    TEST_ASSERT_TRUE(radio.timerArmed);
    const uint32_t relayDueMs = radio.timerAtMs;
    TEST_ASSERT_TRUE(relayDueMs >= 2 * txpath::CW_MAX * txpath::SLOT_MS);
    radio.ownPosition(10);
    if (build43 == 0) {
      TEST_ASSERT_FALSE(radio.queue.front().ours);
      TEST_ASSERT_EQUAL_UINT32(relayDueMs, radio.timerAtMs);
      continue;
    }
    TEST_ASSERT_TRUE(radio.queue.front().ours);
    TEST_ASSERT_TRUE(radio.timerAtMs <= 10 + 7 * txpath::SLOT_MS);
    // A relay queued after ours stays behind it and leaves the timer alone.
    const uint32_t ownDueMs = radio.timerAtMs;
    radio.relay(txpath::CW_MAX, 12);
    TEST_ASSERT_TRUE(radio.queue.front().ours);
    TEST_ASSERT_EQUAL_UINT32(ownDueMs, radio.timerAtMs);
  }
  // DEFAULT would not have done: at equal priority Meshtastic sends another
  // car's packet before ours.
  txpath::Packet relayed;
  relayed.priority = PRIORITY_DEFAULT;
  txpath::Packet ours;
  ours.ours = true;
  ours.priority = PRIORITY_DEFAULT;
  TEST_ASSERT_TRUE(txpath::goesBefore(relayed, ours));
  ours.priority = OWN_POSITION_PRIORITY;
  TEST_ASSERT_TRUE(txpath::goesBefore(ours, relayed));
}

// ---- Our position unsigned (build 43) ------------------------------------------

// Meshtastic's port numbers for the traffic that stays signed.
static const uint32_t PORT_TEXT = 1;
static const uint32_t PORT_ROUTING = 5;
static const uint32_t PORT_ADMIN = 6;
static const uint32_t PORT_NODEINFO = 4;
static const uint32_t PORT_PRIVATE = 256;  // our reach summaries

void test_only_our_position_goes_unsigned() {
  // Our position, broadcast on the ride's keyed channel while the fast lane sends it.
  TEST_ASSERT_TRUE(sendsUnsigned(PORT_POSITION, true, true, true, false));
  // Everything else stays signed.
  const uint32_t signedPorts[] = {PORT_TEXT, PORT_ROUTING, PORT_ADMIN, PORT_NODEINFO, PORT_PRIVATE};
  for (size_t i = 0; i < sizeof(signedPorts) / sizeof(signedPorts[0]); i++) {
    TEST_ASSERT_FALSE(sendsUnsigned(signedPorts[i], true, true, true, false));
  }
  // A radio in stock mode (no Touge hello) is stock: PositionModule's own
  // position is signed.
  TEST_ASSERT_FALSE(sendsUnsigned(PORT_POSITION, true, false, true, false));
  // A channel anyone can read, a licensed radio, or a position to one node.
  TEST_ASSERT_FALSE(sendsUnsigned(PORT_POSITION, true, true, false, false));
  TEST_ASSERT_FALSE(sendsUnsigned(PORT_POSITION, true, true, true, true));
  TEST_ASSERT_FALSE(sendsUnsigned(PORT_POSITION, false, true, true, false));
}

void test_only_a_position_on_a_keyed_channel_passes_unsigned_from_a_signer() {
  TEST_ASSERT_TRUE(passesUnsigned(PORT_POSITION, true, false));
  // A downgraded text, NodeInfo, summary or admin from a node that signs still drops.
  const uint32_t signedPorts[] = {PORT_TEXT, PORT_ROUTING, PORT_ADMIN, PORT_NODEINFO, PORT_PRIVATE};
  for (size_t i = 0; i < sizeof(signedPorts) / sizeof(signedPorts[0]); i++) {
    TEST_ASSERT_FALSE(passesUnsigned(signedPorts[i], true, false));
  }
  // On a public channel anyone can forge a position, so Balanced keeps its rule.
  TEST_ASSERT_FALSE(passesUnsigned(PORT_POSITION, false, false));
  // A licensed radio keeps it all.
  TEST_ASSERT_FALSE(passesUnsigned(PORT_POSITION, true, true));
}

int main(int, char**) {
  UNITY_BEGIN();
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
  RUN_TEST(test_a_noted_position_knows_how_long_it_has_waited);
  RUN_TEST(test_under_a_backlog_the_rear_car_keeps_its_turn);
  RUN_TEST(test_behind_relays_our_position_waited_seconds);
  RUN_TEST(test_our_position_leaves_ahead_of_queued_relays_and_relays_still_go);
  RUN_TEST(test_our_position_goes_ahead_of_a_relay_waiting_out_its_delay);
  RUN_TEST(test_only_our_position_goes_unsigned);
  RUN_TEST(test_only_a_position_on_a_keyed_channel_passes_unsigned_from_a_signer);
  return UNITY_END();
}
