// Host tests for the forward decision drainRadio and sendDeferred make
// (forward.h): whether fastrelay's verdict drops a forward on a ride this size,
// when a held forward is due, its copy count, and the relayer byte.
//
// The rides are a real Mesh as the roster and a real FastRelay fed each car's
// slot map once a second, every car hearing every other unless told otherwise.

#include <unity.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "forward.h"
#include "schedule.h"

using namespace touge;

void setUp() {}
void tearDown() {}

static const uint32_t SELF = 0x5E1F0001;
// Payload lengths on the wire, tag included: a position without its name, and
// today's 107 B test voice body.
static const uint16_t POSITION_PAYLOAD = POSITION_MIN + TAG_LEN;
static const uint16_t VOICE_PAYLOAD = 107 + TAG_LEN;

// `cars` other cars, car a on slot a.
struct Ride {
  Mesh mesh;
  FastRelay relay;
  std::vector<uint32_t> ids;
  std::vector<std::vector<bool>> deaf;  // deaf[a][b]: car a's maps never show car b
  uint32_t nowMs = 100000;

  explicit Ride(size_t cars) {
    mesh.reset();
    for (size_t a = 0; a < cars; a++) ids.push_back(0xC0000001 + (uint32_t)a);
    deaf.assign(cars, std::vector<bool>(cars, false));
  }

  // Every car beacons in its slot each second, and we hear every beacon.
  void seconds(int n) {
    for (int i = 0; i < n; i++) {
      const uint32_t base = nowMs - nowMs % SCHEDULE_MS + SCHEDULE_MS;
      for (size_t a = 0; a < ids.size(); a++) {
        Position p;
        p.slot = (uint8_t)a;
        for (size_t b = 0; b < ids.size(); b++) {
          if (b != a && !deaf[a][b]) p.slotMap[b] = slotTag(ids[b]);
        }
        const uint32_t atMs = base + slotStartMs((uint8_t)a) + 2;
        mesh.note(ids[a], p, HEARD_FAST, -50, 0, atMs, 1);
        relay.heardMap(ids[a], p, atMs);
      }
      nowMs = base + SCHEDULE_MS - 1;
    }
  }

  // Long enough for every link to read steady.
  void settle() { seconds(FAST_RELAY_MAPS + 1); }

  ForwardPlan plan(const Frame& f, uint32_t rxMs, uint32_t nowMs, uint32_t tie = 0) const {
    return planForward(f, relay, SELF, mesh.riders(), MAX_RIDERS, rxMs, nowMs, -70, ids.size(), tie);
  }
};

static Frame heardFrom(uint32_t src, uint8_t type, uint8_t hops, uint16_t len, uint8_t relayer = 0) {
  Frame f;
  f.type = type;
  f.src = src;
  f.id = 7;
  f.hops = hops;
  f.relayer = relayer;
  f.len = len;
  return f;
}

static uint16_t payloadOf(uint8_t type) { return type == FRAME_VOICE ? VOICE_PAYLOAD : POSITION_PAYLOAD; }

// ---- Which forwards a SKIP drops ---------------------------------------------

void test_a_skip_drops_every_forward_with_the_origin_alone_and_positions_from_eight_other_cars() {
  for (size_t cars = 1; cars <= FAST_RELAY_CARS; cars++) {
    Ride ride(cars);
    ride.settle();
    for (uint8_t type : {(uint8_t)FRAME_POSITION, (uint8_t)FRAME_VOICE}) {
      const Frame f = heardFrom(ride.ids[0], type, startHopsFor(type), payloadOf(type));
      const ForwardPlan plan = ride.plan(f, ride.nowMs, ride.nowMs);
      char at[48];
      snprintf(at, sizeof(at), "%u other cars, type %u", (unsigned)cars, (unsigned)type);
      TEST_ASSERT_TRUE_MESSAGE(plan.verdict == FastRelayVerdict::SKIP, at);
      const bool drops = cars == 1 || (type != FRAME_VOICE && cars >= FAST_RELAY_MIN_CARS);
      TEST_ASSERT_EQUAL_MESSAGE(drops, plan.skip, at);
    }
  }
}

void test_a_forward_someone_needs_is_held_on_any_ride() {
  Ride ride(FAST_RELAY_CARS);
  ride.deaf[5][0] = true;
  ride.settle();
  const Frame f = heardFrom(ride.ids[0], FRAME_POSITION, FAST_HOPS, POSITION_PAYLOAD);
  const ForwardPlan plan = ride.plan(f, ride.nowMs, ride.nowMs);
  TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::NEEDED);
  TEST_ASSERT_FALSE(plan.skip);
}

void test_a_relayed_copy_is_judged_as_relayed_by_its_own_type_and_relayer() {
  // Car 2 never hears the origin, car 0, but hears car 1 steadily.
  Ride ride(3);
  ride.deaf[2][0] = true;
  ride.settle();
  TEST_ASSERT_NOT_EQUAL(slotTag(ride.ids[1]), slotTag(ride.ids[2]));
  const uint8_t relayerTag = slotTag(ride.ids[1]);
  for (uint8_t type : {(uint8_t)FRAME_POSITION, (uint8_t)FRAME_VOICE}) {
    const uint8_t start = startHopsFor(type);
    // Straight from the origin: car 2 needs it.
    ForwardPlan plan = ride.plan(heardFrom(ride.ids[0], type, start, payloadOf(type), 0), ride.nowMs, ride.nowMs);
    TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::NEEDED);
    TEST_ASSERT_FALSE(plan.relayerUnplaced);
    // Relayed by car 1: car 2 has it from car 1. Three cars, so the forward is still held.
    plan = ride.plan(heardFrom(ride.ids[0], type, start - 1, payloadOf(type), relayerTag), ride.nowMs, ride.nowMs);
    TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::SKIP);
    TEST_ASSERT_FALSE(plan.relayerUnplaced);
    TEST_ASSERT_FALSE(plan.skip);
    // Relayed with no relayer named: judged on the origin alone.
    plan = ride.plan(heardFrom(ride.ids[0], type, start - 1, payloadOf(type), 0), ride.nowMs, ride.nowMs);
    TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::NEEDED);
    TEST_ASSERT_TRUE(plan.relayerUnplaced);
  }
}

void test_the_verdict_and_the_ride_are_read_at_the_receive_time() {
  // A two-car ride heard a lease before the pass: on the receive time the
  // origin is on the ride and alone, so the forward is skipped.
  Ride two(1);
  two.settle();
  const Frame f = heardFrom(two.ids[0], FRAME_POSITION, FAST_HOPS, POSITION_PAYLOAD);
  ForwardPlan plan = two.plan(f, two.nowMs, two.nowMs + LEASE_MS);
  TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::SKIP);
  TEST_ASSERT_TRUE(plan.skip);
  // Three cars: at the receive time car 1's latest map is fresh.
  Ride three(2);
  three.settle();
  plan = three.plan(heardFrom(three.ids[0], FRAME_POSITION, FAST_HOPS, POSITION_PAYLOAD), three.nowMs,
                    three.nowMs + FAST_RELAY_FRESH_MS);
  TEST_ASSERT_TRUE(plan.verdict == FastRelayVerdict::SKIP);
}

// ---- When a held forward is due ------------------------------------------------

void test_a_forward_is_due_from_its_receive_time_by_its_own_length_and_copy_count() {
  Ride ride(3);
  ride.settle();
  const uint32_t rxMs = ride.nowMs;
  const uint32_t spread = forwardSpreadMs(ride.ids.size());
  TEST_ASSERT_NOT_EQUAL(forwardStepMs(FRAME_HEADER + VOICE_PAYLOAD), forwardStepMs(FRAME_HEADER + POSITION_PAYLOAD));
  for (uint32_t tie = 0; tie < FORWARD_TIE_STEPS; tie++) {
    for (uint8_t type : {(uint8_t)FRAME_POSITION, (uint8_t)FRAME_VOICE}) {
      const Frame f = heardFrom(ride.ids[0], type, startHopsFor(type), payloadOf(type));
      const uint32_t waitMs = forwardDelayMs(-70, spread, tie, FRAME_HEADER + payloadOf(type));
      // A pass a few ms after the frame landed, and one that began just before
      // it (the receive callback runs on the Wi-Fi task).
      for (uint32_t nowMs : {rxMs + 4, rxMs + FORWARD_STALL_MS, rxMs - 2}) {
        const ForwardPlan plan = ride.plan(f, rxMs, nowMs, tie);
        TEST_ASSERT_FALSE(plan.skip);
        TEST_ASSERT_EQUAL_UINT32(rxMs + waitMs, plan.dueMs);
        TEST_ASSERT_EQUAL_UINT8(suppressAfterFor(type), plan.suppressAfter);
      }
      // Drained after a stall: from the pass.
      const ForwardPlan late = ride.plan(f, rxMs, rxMs + 120, tie);
      TEST_ASSERT_EQUAL_UINT32(rxMs + 120 + waitMs, late.dueMs);
    }
  }
}

void test_the_stall_edge_is_two_passes_and_wrap_safe() {
  const uint32_t waitMs = forwardDelayMs(-70, forwardSpreadMs(4), 2, FRAME_HEADER + POSITION_PAYLOAD);
  const size_t len = FRAME_HEADER + POSITION_PAYLOAD;
  TEST_ASSERT_EQUAL_UINT32(1000 + waitMs, forwardDueMs(1000, 1000 + 2 * FORWARD_PASS_MS, -70, 4, 2, len));
  TEST_ASSERT_EQUAL_UINT32(1011 + waitMs, forwardDueMs(1000, 1000 + 2 * FORWARD_PASS_MS + 1, -70, 4, 2, len));
  // Across the millis() wrap, either side of the edge.
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFEu + waitMs, forwardDueMs(0xFFFFFFFEu, 3, -70, 4, 2, len));
  TEST_ASSERT_EQUAL_UINT32(18 + waitMs, forwardDueMs(0xFFFFFFF0u, 18, -70, 4, 2, len));
  // Stamped after the pass began: from the stamp.
  TEST_ASSERT_EQUAL_UINT32(3 + waitMs, forwardDueMs(3, 0xFFFFFFFEu, -70, 4, 2, len));
}

void test_a_frame_drained_after_a_stall_counts_the_copies_queued_behind_it() {
  // Heard at 1005 during a 120 ms stall and drained at 1125, with the other two
  // hearers' copies behind the drain budget until the next pass (review of 51).
  // Timed from 1005 it was due on the draining pass and went uncounted.
  const uint32_t rxMs = 1005;
  const uint32_t drainMs = 1125;
  const uint32_t nextPassMs = 1131;
  const size_t len = FRAME_HEADER + POSITION_PAYLOAD;
  uint8_t wire[FRAME_MAX] = {FRAME_MAGIC};
  Mesh mesh;
  mesh.reset();
  mesh.firstSight(7, 1, drainMs);
  const uint32_t dueMs = forwardDueMs(rxMs, drainMs, -70, 6, 1, len);
  TEST_ASSERT_TRUE(mesh.defer(wire, len, 7, 1, dueMs, SUPPRESS_AFTER));
  Forward out;
  TEST_ASSERT_FALSE(mesh.nextDue(drainMs, out));
  mesh.firstSight(7, 1, nextPassMs);
  mesh.firstSight(7, 1, nextPassMs);
  TEST_ASSERT_FALSE(mesh.nextDue(dueMs, out));
  TEST_ASSERT_EQUAL_UINT32(1, mesh.suppressed());
}

// ---- The relayer byte ----------------------------------------------------------

void test_a_forwarder_names_itself_in_the_relayer_byte() {
  uint8_t payload[POSITION_PAYLOAD];
  for (size_t i = 0; i < sizeof(payload); i++) payload[i] = (uint8_t)(i * 7 + 1);
  Frame f = heardFrom(0xA1, FRAME_POSITION, FAST_HOPS - 1, sizeof(payload), 0x42);
  f.chan = 9;
  f.payload = payload;
  uint8_t wire[FRAME_MAX];
  const size_t n = encodeFrame(f, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(FRAME_HEADER + sizeof(payload), n);
  markRelayer(wire, n, SELF);
  Frame back;
  TEST_ASSERT_TRUE(decodeFrame(wire, n, back));
  TEST_ASSERT_NOT_EQUAL(0, slotTag(SELF));
  TEST_ASSERT_EQUAL_UINT8(slotTag(SELF), back.relayer);
  TEST_ASSERT_EQUAL_UINT32(0xA1, back.src);
  TEST_ASSERT_EQUAL_UINT8(FAST_HOPS - 1, back.hops);
  TEST_ASSERT_EQUAL_UINT16(sizeof(payload), back.len);
  TEST_ASSERT_EQUAL_MEMORY(payload, back.payload, sizeof(payload));
  // Too short to hold the byte: left alone.
  uint8_t stub[FRAME_RELAYER_AT] = {0};
  markRelayer(stub, sizeof(stub), SELF);
  for (uint8_t b : stub) TEST_ASSERT_EQUAL_UINT8(0, b);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_skip_drops_every_forward_with_the_origin_alone_and_positions_from_eight_other_cars);
  RUN_TEST(test_a_forward_someone_needs_is_held_on_any_ride);
  RUN_TEST(test_a_relayed_copy_is_judged_as_relayed_by_its_own_type_and_relayer);
  RUN_TEST(test_the_verdict_and_the_ride_are_read_at_the_receive_time);
  RUN_TEST(test_a_forward_is_due_from_its_receive_time_by_its_own_length_and_copy_count);
  RUN_TEST(test_the_stall_edge_is_two_passes_and_wrap_safe);
  RUN_TEST(test_a_frame_drained_after_a_stall_counts_the_copies_queued_behind_it);
  RUN_TEST(test_a_forwarder_names_itself_in_the_relayer_byte);
  return UNITY_END();
}
