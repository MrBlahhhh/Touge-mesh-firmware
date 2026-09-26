// The 32-slot lease schedule: unit tests for the rules, then a simulator that
// runs 25 to 30 cars through joins, drops, reboots, a reference loss and a
// merge, with collisions modelled at every receiver.
//
// Split out of test_frame because it is its own subject and because the
// simulator is most of the file.

#include <unity.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <initializer_list>
#include "frame.h"
#include "mesh.h"
#include "rideclock.h"
#include "schedule.h"

using namespace touge;

// Room past MAX_RIDERS, for the cases with more cars than the firmware roster.
static const size_t ROSTER = 40;

static void addRider(Rider* r, size_t i, uint32_t id, uint32_t atMs = 0) {
  r[i] = Rider{};
  r[i].id = id;
  r[i].used = true;
  r[i].atMs = atMs;
  r[i].pos.slot = SLOT_NONE;
}

static void addLeased(Rider* r, size_t i, uint32_t id, uint8_t slot, uint16_t gen,
                      uint32_t atMs = 0) {
  addRider(r, i, id, atMs);
  r[i].pos.slot = slot;
  r[i].pos.leaseGen = gen;
  r[i].pos.schedGen = gen;
}

static void addBelieving(Rider* r, size_t i, uint32_t id, uint8_t slot, uint32_t refId,
                         uint8_t refHops, bool refLocked = false) {
  addLeased(r, i, id, slot, 1);
  r[i].pos.refId = refId;
  r[i].pos.refHops = refHops;
  r[i].pos.refLocked = refLocked;
}

static void addLocked(Rider* r, size_t i, uint32_t id) {
  addRider(r, i, id);
  r[i].pos.clockLocked = true;
}

static void touch(Rider* r, uint32_t atMs) {
  for (size_t i = 0; i < ROSTER; i++)
    if (r[i].used) r[i].atMs = atMs;
}

// Hear the roster, listen out the join window with everyone still beaconing,
// and rebuild again: where a car stands after joining a ride that looks like
// `r`. Returns the time of the last rebuild.
static uint32_t join(Schedule& s, uint32_t self, Rider* r, bool locked = false) {
  const uint32_t heard = 1000;
  touch(r, heard);
  s.rebuild(self, locked, r, ROSTER, heard);
  const uint32_t settled = heard + JOIN_LISTEN_MS;
  touch(r, settled);
  s.rebuild(self, locked, r, ROSTER, settled);
  return settled;
}

// ---- Layout -----------------------------------------------------------------

void test_slots_tile_the_second_and_leave_the_shared_windows_clear() {
  static bool used[SCHEDULE_MS];
  memset(used, 0, sizeof(used));
  for (uint8_t s = 0; s < MAX_SLOTS; s++) {
    const uint32_t start = slotStartMs(s);
    TEST_ASSERT_TRUE(start + SLOT_MS <= SCHEDULE_MS);
    TEST_ASSERT_TRUE(start % BLOCK_MS + SLOT_MS <= SHARED_OFFSET_MS);
    for (uint32_t ms = start; ms < start + SLOT_MS; ms++) {
      TEST_ASSERT_FALSE_MESSAGE(used[ms], "two slots overlap");
      used[ms] = true;
    }
  }
  for (uint32_t b = 0; b < BLOCKS; b++)
    for (uint32_t ms = b * BLOCK_MS + SHARED_OFFSET_MS; ms < (b + 1) * BLOCK_MS; ms++)
      TEST_ASSERT_FALSE(used[ms]);
}

void test_consecutive_slots_fall_in_different_blocks() {
  // The lowest free slots fill first, so they must not all be in one quarter.
  TEST_ASSERT_EQUAL_UINT32(0, slotStartMs(0));
  TEST_ASSERT_EQUAL_UINT32(BLOCK_MS, slotStartMs(1));
  TEST_ASSERT_EQUAL_UINT32(2 * BLOCK_MS, slotStartMs(2));
  TEST_ASSERT_EQUAL_UINT32(3 * BLOCK_MS, slotStartMs(3));
  TEST_ASSERT_EQUAL_UINT32(SLOT_MS, slotStartMs(4));
  TEST_ASSERT_EQUAL_UINT32(3 * BLOCK_MS + 7 * SLOT_MS, slotStartMs(MAX_SLOTS - 1));
}

void test_the_chain_is_short_enough_to_stay_inside_a_slot() {
  // The firmware asserts this; kept here so the arithmetic is visible. A car
  // MAX_REF_HOPS down the sync chain carries 3 ms of error per hop plus its
  // own wait, and its 8 ms frame still has to finish inside a 27 ms slot.
  const uint32_t syncBias = 3;
  TEST_ASSERT_TRUE(syncBias * (MAX_REF_HOPS + 1) + FRAME_AIRTIME_MS <= SLOT_MS);
  // One hop deeper would not fit, so the slot is as narrow as it can be.
  TEST_ASSERT_TRUE(syncBias * (MAX_REF_HOPS + 2) + FRAME_AIRTIME_MS > SLOT_MS);
}

// ---- Leases -----------------------------------------------------------------

void test_a_newcomer_listens_before_claiming() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 0, 1);
  addLeased(r, 1, 100, 1, 1);

  Schedule s;
  touch(r, 1000);
  s.rebuild(300, false, r, ROSTER, 1000);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_UINT8(3, s.known());

  touch(r, 1000 + JOIN_LISTEN_MS - 1);
  s.rebuild(300, false, r, ROSTER, 1000 + JOIN_LISTEN_MS - 1);
  TEST_ASSERT_FALSE(s.claimed());

  touch(r, 1000 + JOIN_LISTEN_MS);
  s.rebuild(300, false, r, ROSTER, 1000 + JOIN_LISTEN_MS);
  TEST_ASSERT_TRUE(s.claimed());
  // First free in lease order, block 0 row by row: 0 is held, so row 1.
  TEST_ASSERT_EQUAL_UINT8(4, s.slot());
  TEST_ASSERT_EQUAL_UINT16(2, s.leaseGeneration()); // one past the ride's
  TEST_ASSERT_EQUAL_UINT16(2, s.generation());
}

void test_an_unleased_car_speaks_only_in_the_shared_window() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1, 1000);
  Schedule s;
  s.rebuild(300, false, r, ROSTER, 1000);
  TEST_ASSERT_FALSE(s.claimed());

  // Each draw opens one block's window, at the drawn offset, with at least a
  // tick's worth of chances even at the latest offset.
  for (uint32_t draw = 0; draw < BLOCKS * SHARED_SPREAD_MS; draw++) {
    s.drawSharedTurn(draw);
    uint32_t perBlock[BLOCKS] = {};
    for (uint32_t phase = 0; phase < SCHEDULE_MS; phase++) {
      if (!s.inSlotAtPhase(phase)) continue;
      const uint32_t inBlock = phase % BLOCK_MS;
      TEST_ASSERT_TRUE(inBlock >= SHARED_OFFSET_MS);
      TEST_ASSERT_TRUE(inBlock + SLOT_GUARD_MS <= BLOCK_MS);
      perBlock[phase / BLOCK_MS]++;
    }
    for (uint32_t b = 0; b < BLOCKS; b++) {
      if (b == draw % BLOCKS) TEST_ASSERT_TRUE(perBlock[b] >= 5);
      else TEST_ASSERT_EQUAL_UINT32(0, perBlock[b]);
    }
  }
}

void test_a_held_slot_survives_a_car_joining_or_leaving() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 0, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  const uint8_t mine = s.slot();
  TEST_ASSERT_TRUE(s.claimed());

  // A lower node number arrives, unleased, then leased elsewhere.
  addRider(r, 1, 100, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());
  r[1].pos.slot = (uint8_t)(mine + 1);
  r[1].pos.leaseGen = 7;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());

  r[0].used = false;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());
}

void test_the_older_lease_keeps_a_contested_slot() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 900, 0, 4);
  Schedule s;
  uint32_t now = join(s, 300, r);
  const uint8_t mine = s.slot();
  const uint16_t gen = s.leaseGeneration();
  TEST_ASSERT_EQUAL_UINT16(5, gen);

  // A lower node number on our slot with a newer lease moves, not us.
  addLeased(r, 1, 100, mine, (uint16_t)(gen + 3), now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());

  // With an older lease it wins. We move at once, without listening again,
  // and our new lease is newer than anything we have heard of.
  r[1].pos.leaseGen = (uint16_t)(gen - 2);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(mine, s.slot());
  TEST_ASSERT_NOT_EQUAL(0, s.slot());
  TEST_ASSERT_TRUE(leaseOlder((uint16_t)(gen + 3), s.leaseGeneration()));
}

void test_equal_generations_go_to_the_lower_node_number() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 900, 0, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  const uint8_t mine = s.slot();

  addLeased(r, 1, 700, mine, s.leaseGeneration(), now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());

  r[1].id = 100;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(mine, s.slot());
}

void test_a_newcomer_never_takes_an_incumbents_slot() {
  // Node 100 is the lowest number on the ride and becomes the reference. Under
  // the old rule it also took any slot it liked; now it takes a free one, and
  // its lease is newer than every lease it heard, so it would lose any clash.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 900, 0, 7);
  addLeased(r, 1, 800, 1, 3);
  Schedule s;
  join(s, 100, r);
  TEST_ASSERT_TRUE(s.weAreReference());
  TEST_ASSERT_EQUAL_UINT8(4, s.slot());
  TEST_ASSERT_EQUAL_UINT16(8, s.leaseGeneration());
}

void test_cars_joining_together_land_on_distinct_slots_first_time() {
  // Twenty-nine cars (the firmware roster plus us), none leased, all hearing
  // each other. Each ranks itself among the cars that need a slot and takes
  // that free slot, so the same roster gives everyone a different answer
  // without a round of collisions. Three incumbents are already on 0, 5, 9.
  const size_t N = MAX_RIDERS + 1;
  uint32_t ids[N];
  for (size_t i = 0; i < N; i++) ids[i] = 0x1000u + (uint32_t)((i * 7919u) % 1009u) * 3u;

  bool taken[MAX_SLOTS] = {};
  taken[0] = taken[5] = taken[9] = true;
  for (size_t me = 0; me < N; me++) {
    Rider r[ROSTER] = {};
    size_t n = 0;
    addLeased(r, n++, 1, 0, 1);
    addLeased(r, n++, 2, 5, 1);
    addLeased(r, n++, 3, 9, 1);
    for (size_t other = 0; other < N; other++)
      if (other != me) addRider(r, n++, ids[other]);

    Schedule s;
    join(s, ids[me], r);
    TEST_ASSERT_TRUE(s.claimed());
    TEST_ASSERT_FALSE_MESSAGE(taken[s.slot()], "two joiners computed the same slot");
    taken[s.slot()] = true;
  }
}

void test_more_cars_than_slots_wait_rather_than_double_up() {
  // The old scheduler fell back to selfId % MAX_SLOTS and transmitted on top
  // of somebody. With every slot held a newcomer stays unleased and keeps to
  // the shared window.
  Rider r[ROSTER] = {};
  for (uint8_t s = 0; s < MAX_SLOTS; s++) addLeased(r, s, 1000u + s, s, 1);
  Schedule s;
  join(s, 5000, r);
  TEST_ASSERT_FALSE(s.claimed());
  for (uint32_t phase = 0; phase < SCHEDULE_MS; phase++)
    if (s.inSlotAtPhase(phase)) TEST_ASSERT_TRUE(phase % BLOCK_MS >= SHARED_OFFSET_MS);
}

void test_a_lapsed_lease_frees_its_slot() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1, 0);          // last heard at zero
  addLeased(r, 1, 200, 1, 1, LEASE_MS);   // still beaconing
  Schedule s;
  s.rebuild(300, false, r, ROSTER, LEASE_MS);
  r[1].atMs = LEASE_MS + JOIN_LISTEN_MS;
  s.rebuild(300, false, r, ROSTER, LEASE_MS + JOIN_LISTEN_MS);
  TEST_ASSERT_EQUAL_UINT8(2, s.known());
  TEST_ASSERT_EQUAL_UINT8(0, s.slot());

  // One millisecond short of the lease and it would still be held.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 100, 0, 1, 1);
  addLeased(q, 1, 200, 1, 1, LEASE_MS);
  Schedule t;
  t.rebuild(300, false, q, ROSTER, LEASE_MS);
  q[1].atMs = JOIN_LISTEN_MS + 1;
  q[0].atMs = JOIN_LISTEN_MS + 1;
  t.rebuild(300, false, q, ROSTER, LEASE_MS + JOIN_LISTEN_MS);
  TEST_ASSERT_EQUAL_UINT8(4, t.slot());
}

void test_a_car_alone_for_a_lease_lets_its_slot_go_and_listens_again() {
  // It cannot know whether its slot was reissued while it was away, so it
  // comes back as a newcomer rather than reclaiming it.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());

  now += LEASE_MS;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_UINT8(1, s.known());

  // Back in range: a fresh listen before any claim.
  touch(r, now + 10);
  s.rebuild(300, false, r, ROSTER, now + 10);
  TEST_ASSERT_FALSE(s.claimed());
  touch(r, now + 10 + JOIN_LISTEN_MS);
  s.rebuild(300, false, r, ROSTER, now + 10 + JOIN_LISTEN_MS);
  TEST_ASSERT_TRUE(s.claimed());
}

void test_a_duplicate_node_number_does_not_corrupt_the_claim() {
  // Two boards with one node number is a real fault nothing here can fix, but
  // the twin must not count as a car or block our slot.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 1, 1);
  addLeased(r, 1, 300, 0, 1); // our own number, on slot 0
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(2, s.known());
  TEST_ASSERT_EQUAL_UINT8(0, s.slot());
}

void test_generation_compare_survives_the_wrap() {
  TEST_ASSERT_TRUE(leaseOlder(65535, 0));
  TEST_ASSERT_FALSE(leaseOlder(0, 65535));
  TEST_ASSERT_FALSE(leaseOlder(5, 5));
  TEST_ASSERT_TRUE(leaseOlder(5, 6));
}

void test_a_fresh_board_adopts_the_ride_generation_outright() {
  // A wrap-safe compare against its own zero would call 40000 older.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 40000);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT16(40001, s.leaseGeneration());
}

// ---- Timing -----------------------------------------------------------------

void test_a_frame_that_cannot_finish_in_the_slot_does_not_start() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());
  const uint32_t opens = slotStartMs(s.slot());

  TEST_ASSERT_FALSE(s.inSlotAtPhase(opens - 1));
  TEST_ASSERT_TRUE(s.inSlotAtPhase(opens));
  TEST_ASSERT_TRUE(s.inSlotAtPhase(opens + SLOT_MS - SLOT_GUARD_MS));
  TEST_ASSERT_FALSE(s.inSlotAtPhase(opens + SLOT_MS - SLOT_GUARD_MS + 1));
  TEST_ASSERT_FALSE(s.inSlotAtPhase(opens + SLOT_MS - 1));
}

void test_a_car_alone_speaks_any_time() {
  Schedule s;
  TEST_ASSERT_TRUE(s.inSlotAtPhase(999));
  TEST_ASSERT_TRUE(s.inSlot(123));
  Rider none[ROSTER] = {};
  s.rebuild(300, false, none, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT8(1, s.known());
  TEST_ASSERT_TRUE(s.inSlot(0));
}

void test_slot_window_opens_once_per_second_after_a_sync() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1); // the reference, on slot 0
  Schedule s;
  join(s, 300, r);
  const uint32_t opens = slotStartMs(s.slot());
  TEST_ASSERT_EQUAL_UINT32(100, s.parentId());

  s.syncTo(10000, 0);
  TEST_ASSERT_TRUE(s.synced());
  TEST_ASSERT_FALSE(s.inSlot(10000));
  TEST_ASSERT_TRUE(s.inSlot(10000 + opens));
  TEST_ASSERT_FALSE(s.inSlot(10000 + opens + SLOT_MS - SLOT_GUARD_MS + 1));
  TEST_ASSERT_TRUE(s.inSlot(10000 + SCHEDULE_MS + opens));
  TEST_ASSERT_FALSE(s.inSlot(10000 + BLOCK_MS + opens)); // not every block
}

void test_syncing_allows_for_the_sender_waiting_for_a_tick() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule a, b;
  join(a, 300, r);
  join(b, 300, r);
  a.syncTo(10000, 0);
  b.syncTo(10000, 0, 3);
  const uint32_t mine = slotStartMs(a.slot());
  TEST_ASSERT_TRUE(b.inSlot(10000 + mine - 3));
  TEST_ASSERT_FALSE(a.inSlot(10000 + mine - 3));
}

void test_sync_reads_the_slot_from_the_beacon_not_the_roster() {
  // The parent moved from slot 6 to 9 in the beacon we just heard. Backing out
  // the roster's 6 would put us a block out for a whole second.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 6, 1);
  Schedule s;
  join(s, 300, r);
  const uint32_t heard = 50000;
  s.syncTo(heard, 9);
  const uint32_t began = heard - slotStartMs(9);
  TEST_ASSERT_TRUE(s.inSlot(began + slotStartMs(s.slot())));
}

void test_a_beacon_from_the_shared_window_does_not_set_the_clock() {
  Schedule s;
  s.syncTo(1234, SLOT_NONE);
  TEST_ASSERT_FALSE(s.synced());
}

void test_slots_keep_running_across_the_millis_wrap() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  join(s, 300, r);
  s.syncTo(0xFFFFFF00, 0);
  TEST_ASSERT_TRUE(s.inSlot((uint32_t)(0xFFFFFF00 + SCHEDULE_MS + slotStartMs(s.slot()))));
}

void test_gps_slots_need_no_reference_car() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 2, 1);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_FALSE(s.synced());
  TEST_ASSERT_TRUE(s.inSlot(0)); // no epoch, so it free-runs
  const uint32_t opens = slotStartMs(s.slot());
  TEST_ASSERT_TRUE(s.inSlotAtPhase(opens));
  TEST_ASSERT_FALSE(s.inSlotAtPhase(opens + SLOT_MS));
}

// ---- The reference ----------------------------------------------------------

void test_a_reference_travels_past_the_cars_that_can_hear_it() {
  Rider r[ROSTER] = {};
  addBelieving(r, 0, 300, 2, /*refId=*/100, /*refHops=*/1);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  TEST_ASSERT_EQUAL_UINT8(2, s.hopsToReference());
  TEST_ASSERT_EQUAL_UINT32(300, s.parentId());
}

void test_the_reference_is_no_hops_from_itself_and_syncs_to_nobody() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 1, 1);
  Schedule s;
  join(s, 100, r);
  TEST_ASSERT_TRUE(s.weAreReference());
  TEST_ASSERT_EQUAL_UINT8(0, s.hopsToReference());
  TEST_ASSERT_EQUAL_UINT32(0, s.parentId());
}

void test_an_unleased_car_is_not_the_reference_while_a_leased_one_is_heard() {
  // A reference that has just rebooted keeps its node number but listens for
  // a lease; its beacons from the shared window cannot set anyone's clock. The
  // job goes to the best leased car until it has one again.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 1, 1);
  addLeased(r, 1, 700, 2, 1);
  Schedule s;
  s.rebuild(100, false, r, ROSTER, 0);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_UINT32(500, s.referenceId());

  // And the same seen from a leased car: the unleased 100 does not win.
  Rider q[ROSTER] = {};
  addRider(q, 0, 100);
  addLeased(q, 1, 700, 2, 1);
  Schedule t;
  join(t, 500, q);
  TEST_ASSERT_TRUE(t.weAreReference());

  // A GPS-locked car keeps time off its own pulse, leased or not.
  q[0].pos.clockLocked = true;
  t.rebuild(500, false, q, ROSTER, 1000 + JOIN_LISTEN_MS);
  TEST_ASSERT_EQUAL_UINT32(100, t.referenceId());
}

void test_a_reference_with_nobody_leased_declares_the_epoch() {
  Rider r[ROSTER] = {};
  addRider(r, 0, 500);
  Schedule s;
  s.rebuild(100, false, r, ROSTER, 0);
  TEST_ASSERT_TRUE(s.weAreReference());
  s.startEpoch(0);
  TEST_ASSERT_TRUE(s.synced());

  // With a leased car in earshot it waits for that car's beacon instead, so a
  // car that has lost its clock does not drag the ride onto a new one.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 500, 3, 1);
  Schedule t;
  t.rebuild(100, true, q, ROSTER, 0); // locked, so the reference regardless
  TEST_ASSERT_TRUE(t.weAreReference());
  t.startEpoch(0);
  TEST_ASSERT_FALSE(t.synced());

  // Alone it does not bother: the epoch would only get in the way later.
  Schedule alone;
  Rider none[ROSTER] = {};
  alone.rebuild(100, false, none, ROSTER, 0);
  alone.startEpoch(0);
  TEST_ASSERT_FALSE(alone.synced());
}

void test_without_an_epoch_any_leased_beacon_sets_the_clock() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 5, 1);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT32(100, s.parentId());
  // Not only the parent: 900 will do until there is an epoch.
  TEST_ASSERT_TRUE(s.takesClockFrom(900));
  s.syncTo(7000, 5);
  TEST_ASSERT_TRUE(s.synced());
  // After that, the parent only.
  TEST_ASSERT_FALSE(s.takesClockFrom(900));
  TEST_ASSERT_TRUE(s.takesClockFrom(100));
}

void test_the_nearest_route_to_the_reference_wins() {
  Rider r[ROSTER] = {};
  addBelieving(r, 0, 300, 2, 100, 3);
  addBelieving(r, 1, 400, 5, 100, 1);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(400, s.parentId());
  TEST_ASSERT_EQUAL_UINT8(2, s.hopsToReference());
}

void test_a_parent_must_be_heard_directly_and_hold_a_lease() {
  // 200 is nearer the reference but unleased, so its beacon marks no slot.
  // 100 is the reference but only heard relayed. 300 is the one to sync to.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  r[0].hopsAway = 1;
  addRider(r, 1, 200);
  r[1].pos.refId = 100;
  r[1].pos.refHops = 0;
  addBelieving(r, 2, 300, 3, 100, 1);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  TEST_ASSERT_EQUAL_UINT32(300, s.parentId());
}

void test_a_relayed_locked_reference_beats_a_free_running_one_in_earshot() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 200, 1, 1);
  addBelieving(r, 1, 300, 2, 700, 1, /*refLocked=*/true);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
  TEST_ASSERT_TRUE(s.referenceLocked());
  TEST_ASSERT_EQUAL_UINT32(300, s.parentId());
}

void test_a_route_longer_than_the_cap_is_not_believed() {
  Rider r[ROSTER] = {};
  addBelieving(r, 0, 300, 2, 100, MAX_REF_HOPS);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(300, s.referenceId());
}

void test_a_convoy_strung_out_converges_on_one_reference() {
  // Four cars in a line, each hearing only the cars beside it, beaconing once
  // a second. They lease after the join listen, then the head's claim walks
  // down the line one car per beacon.
  const uint32_t id[4] = {100, 200, 300, 400};
  Schedule s[4];
  Position belief[4];
  for (int i = 0; i < 4; i++) belief[i] = Position{};
  for (int round = 0; round < 10; round++) {
    const uint32_t now = 1000 + (uint32_t)round * SCHEDULE_MS;
    for (int me = 0; me < 4; me++) {
      Rider roster[ROSTER] = {};
      size_t n = 0;
      for (int other = 0; other < 4; other++) {
        if (other != me - 1 && other != me + 1) continue;
        addRider(roster, n, id[other], now);
        roster[n].pos = belief[other];
        n++;
      }
      s[me].rebuild(id[me], false, roster, ROSTER, now);
    }
    for (int me = 0; me < 4; me++) {
      // Each hears its neighbours' lease beacons, so its slot map says so.
      memset(belief[me].slotMap, 0, sizeof(belief[me].slotMap));
      for (int other = me - 1; other <= me + 1; other += 2)
        if (other >= 0 && other < 4 && s[other].claimed())
          belief[me].slotMap[s[other].slot()] = slotTag(id[other]);
      belief[me].slot = s[me].slot();
      belief[me].leaseGen = s[me].leaseGeneration();
      belief[me].schedGen = s[me].generation();
      belief[me].refId = s[me].referenceId();
      belief[me].refHops = s[me].hopsToReference();
      belief[me].refLocked = s[me].referenceLocked();
      belief[me].fitToKeepTime = s[me].fitToKeepTime();
      belief[me].refFit = s[me].referenceFit();
    }
  }
  for (int me = 0; me < 4; me++) {
    TEST_ASSERT_EQUAL_UINT32(100, s[me].referenceId());
    TEST_ASSERT_EQUAL_UINT8(me, s[me].hopsToReference());
  }
  TEST_ASSERT_TRUE(s[0].weAreReference());
  TEST_ASSERT_EQUAL_UINT32(100, s[1].parentId());
  TEST_ASSERT_EQUAL_UINT32(200, s[2].parentId());
  TEST_ASSERT_EQUAL_UINT32(300, s[3].parentId());
}

void test_the_lowest_node_number_is_the_reference() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 1, 1);
  addLeased(r, 1, 900, 2, 1);
  Schedule s;
  join(s, 100, r);
  TEST_ASSERT_TRUE(s.weAreReference());
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(1, s.slot());
  TEST_ASSERT_NOT_EQUAL(2, s.slot());
}

void test_a_locked_car_outranks_a_lower_numbered_free_running_one() {
  Rider r[ROSTER] = {};
  addRider(r, 0, 100);
  addLocked(r, 1, 700);
  Schedule s;
  s.rebuild(300, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
}

void test_the_lowest_locked_car_wins_among_several() {
  Rider r[ROSTER] = {};
  addRider(r, 0, 100);
  addLocked(r, 1, 900);
  addLocked(r, 2, 500);
  Schedule s;
  s.rebuild(300, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(500, s.referenceId());
}

void test_our_own_lock_counts_too() {
  Rider r[ROSTER] = {};
  addRider(r, 0, 100);
  addRider(r, 1, 900);
  Schedule s;
  s.rebuild(300, true, r, ROSTER, 0);
  TEST_ASSERT_TRUE(s.weAreReference());
}

void test_nobody_locked_falls_back_to_the_lowest_number() {
  Rider r[ROSTER] = {};
  addRider(r, 0, 100);
  addRider(r, 1, 900);
  Schedule s;
  s.rebuild(300, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
}

void test_sync_backs_out_the_reference_slot() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLocked(r, 1, 700);
  r[1].pos.slot = 2;
  r[1].pos.leaseGen = 1;
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
  TEST_ASSERT_EQUAL_UINT32(700, s.parentId());
  const uint8_t mine = s.slot();
  TEST_ASSERT_NOT_EQUAL(0, mine);
  TEST_ASSERT_NOT_EQUAL(2, mine);

  s.syncTo(10000, 2);
  const uint32_t began = 10000 - slotStartMs(2);
  TEST_ASSERT_TRUE(s.inSlot(began + slotStartMs(mine)));
  TEST_ASSERT_FALSE(s.inSlot(10000));
  TEST_ASSERT_FALSE(s.inSlot(began));
}

// ---- Fit to keep time: a weak radio never keeps the clock -------------------

// A lease beacon on `slot` whose slot map shows `us` on `usSlot`, or nobody
// when usSlot is SLOT_NONE.
static Position beaconOn(uint8_t slot, uint8_t usSlot = SLOT_NONE, uint32_t us = 0) {
  Position p{};
  p.slot = slot;
  if (usSlot < MAX_SLOTS) p.slotMap[usSlot] = slotTag(us);
  return p;
}

void test_a_car_that_is_not_fit_does_not_keep_time_whatever_its_number() {
  // 100 has the lowest number, like the moto's radio on the bench, and hears
  // the ride too poorly to keep time for it.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 500, 4, 1);
  r[1].pos.fitToKeepTime = true;
  Schedule s;
  const uint32_t now = join(s, 900, r);
  TEST_ASSERT_EQUAL_UINT32(500, s.referenceId());
  TEST_ASSERT_TRUE(s.referenceFit());

  // GPS lock still comes first.
  r[0].pos.clockLocked = true;
  s.rebuild(900, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());

  // Both fit: the lowest number, as before build 40.
  r[0].pos.clockLocked = false;
  r[0].pos.fitToKeepTime = true;
  s.rebuild(900, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
}

void test_a_belief_carries_its_references_fitness_until_we_hear_it_ourselves() {
  // 700 is out of our range; 300 names it, and says it is fit.
  Rider r[ROSTER] = {};
  addBelieving(r, 0, 300, 2, 700, 1);
  r[0].pos.refFit = true;
  addLeased(r, 1, 200, 4, 1);
  Schedule s;
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
  TEST_ASSERT_TRUE(s.referenceFit());

  // Once 700's own beacon arrives it speaks for itself, and it is not fit.
  addLeased(r, 2, 700, 8, 1);
  s.rebuild(500, false, r, ROSTER, 0);
  TEST_ASSERT_EQUAL_UINT32(200, s.referenceId());
}

void test_fitness_needs_half_the_links_solid_both_ways_and_a_third_to_keep_it() {
  // Only 200's map ever shows us; we hear the others, they do not hear us.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 200, 4, 1);
  addLeased(r, 1, 300, 8, 1);
  addLeased(r, 2, 400, 12, 1);
  addLeased(r, 3, 600, 16, 1);
  addLeased(r, 4, 700, 20, 1);
  Schedule s;
  const uint32_t joined = join(s, 100, r);
  TEST_ASSERT_EQUAL_UINT8(0, s.slot());
  r[0].pos.slotMap[0] = slotTag(100);

  for (uint32_t k = 1; k <= 20; k++) {
    const uint32_t now = joined + k * SCHEDULE_MS;
    touch(r, now);
    s.heardBeacon(200, beaconOn(4, 0, 100), now);
    s.heardBeacon(300, beaconOn(8), now);
    if (k > 12) s.heardBeacon(400, beaconOn(12), now);
    if (k > 14) {
      s.heardBeacon(600, beaconOn(16), now);
      s.heardBeacon(700, beaconOn(20), now);
    }
    s.rebuild(100, false, r, ROSTER, now);
    // 200 is solid once heard in FIT_SEEN_SECONDS with its map showing us, one
    // of the two cars we hear, and that has to hold for FIT_HOLD_MS.
    if (k == 11) TEST_ASSERT_FALSE(s.fitToKeepTime());
    if (k == 12) TEST_ASSERT_TRUE(s.fitToKeepTime());
    // One solid of three keeps it.
    if (k == 14) TEST_ASSERT_TRUE(s.fitToKeepTime());
    // One of five does not, once that has held for FIT_HOLD_MS: five heard
    // from k = 16, so the record last agreed at k = 15.
    if (k == 17) TEST_ASSERT_TRUE(s.fitToKeepTime());
    if (k == 18) TEST_ASSERT_FALSE(s.fitToKeepTime());
  }

  // One solid of three never wins it.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 200, 4, 1);
  addLeased(q, 1, 300, 8, 1);
  addLeased(q, 2, 400, 12, 1);
  Schedule t;
  const uint32_t tJoined = join(t, 100, q);
  q[0].pos.slotMap[0] = slotTag(100);
  for (uint32_t k = 1; k <= 10; k++) {
    const uint32_t now = tJoined + k * SCHEDULE_MS;
    touch(q, now);
    t.heardBeacon(200, beaconOn(4, 0, 100), now);
    t.heardBeacon(300, beaconOn(8), now);
    t.heardBeacon(400, beaconOn(12), now);
    t.rebuild(100, false, q, ROSTER, now);
    TEST_ASSERT_FALSE(t.fitToKeepTime());
  }
}

// ---- A radio that hears the ride poorly -------------------------------------

void test_a_car_that_hears_the_ride_poorly_listens_twice_as_long() {
  // Two leased cars heard one beacon in four, as the moto's radio heard the
  // others on the bench (20-35 %).
  Rider r[ROSTER] = {};
  addLeased(r, 0, 500, 0, 1);
  addLeased(r, 1, 700, 4, 1);
  Schedule s;
  const uint32_t first = 1000;
  for (uint32_t now = first; now <= first + 2 * JOIN_LISTEN_MS; now += 500) {
    if ((now - first) % 4000 == 0) {
      touch(r, now);
      s.heardBeacon(500, beaconOn(0), now);
      s.heardBeacon(700, beaconOn(4), now);
    }
    s.rebuild(300, false, r, ROSTER, now);
    char at[40];
    snprintf(at, sizeof(at), "claimed %u ms after the first beacon", (unsigned)(now - first));
    if (now < first + 2 * JOIN_LISTEN_MS) TEST_ASSERT_FALSE_MESSAGE(s.claimed(), at);
  }
  TEST_ASSERT_EQUAL_UINT8(8, s.slot());

  // Heard every second: the ordinary listen.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 500, 0, 1);
  addLeased(q, 1, 700, 4, 1);
  Schedule t;
  for (uint32_t now = first; now <= first + JOIN_LISTEN_MS; now += 500) {
    if ((now - first) % 1000 == 0) {
      touch(q, now);
      t.heardBeacon(500, beaconOn(0), now);
      t.heardBeacon(700, beaconOn(4), now);
    }
    t.rebuild(300, false, q, ROSTER, now);
  }
  TEST_ASSERT_EQUAL_UINT8(8, t.slot());
}

void test_a_second_lost_lease_backs_off_before_claiming_again() {
  // Outranked, a car with a good view moves at once, however often: that is
  // what settles a merge.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 900, 0, 4);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(4, s.slot());
  addLeased(r, 1, 100, 4, 2, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(8, s.slot());
  addLeased(r, 2, 50, 8, 1, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT8(12, s.slot());

  // Drowned twice before settling (two neighbours whose maps show nobody on
  // our slot): the second listen gets the random extra.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 100, 0, 1);
  addLeased(q, 1, 900, 8, 1);
  Schedule t;
  t.drawSharedTurn(0xABCD0000u); // backoff jitter: 0xABCD % 1000 = 981 ms
  now = join(t, 300, q);
  TEST_ASSERT_TRUE(t.claimed());
  now += HEARD_WINDOW_MS + 100;
  touch(q, now);
  t.rebuild(300, false, q, ROSTER, now);
  TEST_ASSERT_FALSE(t.claimed());
  now += JOIN_LISTEN_MS; // the first: an ordinary listen
  touch(q, now);
  t.rebuild(300, false, q, ROSTER, now);
  TEST_ASSERT_TRUE(t.claimed());
  now += HEARD_WINDOW_MS + 100;
  touch(q, now);
  t.rebuild(300, false, q, ROSTER, now);
  TEST_ASSERT_FALSE(t.claimed());
  touch(q, now + JOIN_LISTEN_MS + 980);
  t.rebuild(300, false, q, ROSTER, now + JOIN_LISTEN_MS + 980);
  TEST_ASSERT_FALSE(t.claimed());
  touch(q, now + JOIN_LISTEN_MS + 981);
  t.rebuild(300, false, q, ROSTER, now + JOIN_LISTEN_MS + 981);
  TEST_ASSERT_TRUE(t.claimed());
}

void test_silence_from_neighbours_heard_poorly_does_not_drown_a_lease() {
  // test_a_drowned_lease_listens_again_before_reclaiming, but we hear both
  // neighbours one beacon in three, so their silence about us says little.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 8, 1);
  Schedule s;
  const uint32_t joined = join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());
  for (uint32_t k = 1; k <= 7; k += 3) {
    const uint32_t at = joined + k * SCHEDULE_MS;
    s.heardBeacon(100, beaconOn(0), at);
    s.heardBeacon(900, beaconOn(8), at);
  }
  touch(r, joined + 7 * SCHEDULE_MS);
  s.rebuild(300, false, r, ROSTER, joined + 7 * SCHEDULE_MS + 100);
  TEST_ASSERT_TRUE(s.claimed());
}

void test_a_lease_no_map_has_shown_for_a_while_is_given_up() {
  // One neighbour, so silence alone never counts as a clash. But its map never
  // shows us, and the ride would let our lease lapse without hearing it.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  const uint32_t joined = join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());
  touch(r, joined + UNHEARD_MS - 1);
  s.rebuild(300, false, r, ROSTER, joined + UNHEARD_MS - 1);
  TEST_ASSERT_TRUE(s.claimed());
  touch(r, joined + UNHEARD_MS);
  s.rebuild(300, false, r, ROSTER, joined + UNHEARD_MS);
  TEST_ASSERT_FALSE(s.claimed());

  // A map that shows us keeps the lease, relayed copies included.
  Rider q[ROSTER] = {};
  addLeased(q, 0, 100, 0, 1);
  Schedule t;
  const uint32_t tJoined = join(t, 300, q);
  const uint8_t mine = t.slot();
  q[0].pos.slotMap[mine] = slotTag(300);
  q[0].hopsAway = 1;
  for (uint32_t at = tJoined + 1000; at <= tJoined + 2 * UNHEARD_MS; at += 1000) {
    touch(q, at);
    t.rebuild(300, false, q, ROSTER, at);
  }
  TEST_ASSERT_EQUAL_UINT8(mine, t.slot());
}

static const uint64_t SEC = 1000000ULL;

static void lockClock(RideClock& c, uint64_t lastAt) {
  for (uint32_t i = PULSE_LOCK_RUN; i > 0; i--) c.onPulse(lastAt - (uint64_t)i * PULSE_PERIOD_US);
  c.onPulse(lastAt);
}

void test_a_mixed_ride_puts_everyone_on_one_schedule() {
  // One car on its GPS pulse, one on beacons, and they agree on the second.
  RideClock clock;
  clock.reset();
  const uint64_t pulse = 4242 * SEC;
  lockClock(clock, pulse);
  TEST_ASSERT_TRUE(cycleDividesSecond(SCHEDULE_MS));

  Rider gpsRoster[ROSTER] = {};
  addLeased(gpsRoster, 0, 300, 0, 1);
  Schedule gps;
  join(gps, 700, gpsRoster, /*locked=*/true);
  TEST_ASSERT_TRUE(gps.weAreReference());
  const uint8_t gpsSlot = gps.slot();
  TEST_ASSERT_NOT_EQUAL(0, gpsSlot);

  uint64_t sendAt = 0;
  for (uint32_t ms = 0; ms < SCHEDULE_MS; ms++) {
    uint32_t phase = 0;
    TEST_ASSERT_TRUE(clock.phaseMs(pulse + (uint64_t)ms * 1000, SCHEDULE_MS, phase));
    if (gps.inSlotAtPhase(phase)) {
      sendAt = pulse + (uint64_t)ms * 1000;
      break;
    }
  }
  TEST_ASSERT_EQUAL_UINT64(pulse + (uint64_t)slotStartMs(gpsSlot) * 1000, sendAt);

  const uint32_t heardAtMs = 55555;
  Rider phoneRoster[ROSTER] = {};
  addLocked(phoneRoster, 0, 700);
  phoneRoster[0].pos.slot = gpsSlot;
  phoneRoster[0].pos.leaseGen = gps.leaseGeneration();
  phoneRoster[0].pos.schedGen = gps.generation();
  Schedule phone;
  join(phone, 300, phoneRoster);
  TEST_ASSERT_EQUAL_UINT32(700, phone.parentId());
  const uint8_t phoneSlot = phone.slot();
  TEST_ASSERT_NOT_EQUAL(gpsSlot, phoneSlot);
  phone.syncTo(heardAtMs, gpsSlot);
  const uint32_t began = heardAtMs - slotStartMs(gpsSlot);
  TEST_ASSERT_TRUE(phone.inSlot(began + slotStartMs(phoneSlot)));
  TEST_ASSERT_FALSE(phone.inSlot(heardAtMs));
}

// ---- The wire ---------------------------------------------------------------

void test_the_position_carries_the_lease_and_the_reference() {
  Position p{};
  p.lat = 351102700;
  p.slot = 31;
  p.leaseGen = 513;
  p.schedGen = 777;
  p.refId = 0xDEADBEEF;
  p.refHops = 3;
  p.refLocked = true;
  p.clockLocked = true;
  p.fitToKeepTime = true;
  p.refFit = true;

  uint8_t wire[POSITION_MIN];
  size_t n = encodePosition(p, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(POSITION_MIN, n);
  Position got{};
  TEST_ASSERT_TRUE(decodePosition(wire, n, got));
  TEST_ASSERT_EQUAL_UINT8(31, got.slot);
  TEST_ASSERT_EQUAL_UINT16(513, got.leaseGen);
  TEST_ASSERT_EQUAL_UINT16(777, got.schedGen);
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, got.refId);
  TEST_ASSERT_EQUAL_UINT8(3, got.refHops);
  TEST_ASSERT_TRUE(got.refLocked);
  TEST_ASSERT_TRUE(got.clockLocked);
  TEST_ASSERT_TRUE(got.fitToKeepTime);
  TEST_ASSERT_TRUE(got.refFit);
  TEST_ASSERT_EQUAL_INT32(351102700, got.lat);

  // Unleased goes out as SLOT_NONE and comes back as SLOT_NONE.
  Position none{};
  n = encodePosition(none, wire, sizeof(wire));
  TEST_ASSERT_TRUE(decodePosition(wire, n, got));
  TEST_ASSERT_EQUAL_UINT8(SLOT_NONE, got.slot);
}

void test_a_version_1_frame_is_refused() {
  // What a build-29 board sends. Its position layout has the slot in a nibble
  // and no generations, so reading it as version 2 would misplace the name
  // and invent leases. It is dropped, and a mixed ride sees each other over
  // LoRa only.
  const uint8_t body[] = {1, 2, 3};
  Frame f;
  f.type = FRAME_POSITION;
  f.src = 1;
  f.id = 2;
  f.payload = body;
  f.len = sizeof(body);
  uint8_t wire[FRAME_MAX];
  size_t n = encodeFrame(f, wire, sizeof(wire));
  Frame got;
  TEST_ASSERT_TRUE(decodeFrame(wire, n, got));
  wire[1] = (uint8_t)((1 << 4) | FRAME_POSITION);
  TEST_ASSERT_FALSE(decodeFrame(wire, n, got));
  // Nor a build 30-37 one, whose position has no fix identity, nor a build
  // 38-39 one, which says nothing of fitness to keep time.
  wire[1] = (uint8_t)((2 << 4) | FRAME_POSITION);
  TEST_ASSERT_FALSE(decodeFrame(wire, n, got));
  wire[1] = (uint8_t)((3 << 4) | FRAME_POSITION);
  TEST_ASSERT_FALSE(decodeFrame(wire, n, got));
}

// ---- Extra beacons ----------------------------------------------------------

static uint32_t bits(std::initializer_list<uint8_t> slots) {
  uint32_t mask = 0;
  for (uint8_t s : slots) mask |= 1u << s;
  return mask;
}

// Leased riders on every block-0 slot except the one in `exceptRow`, so a
// joiner lands in block 1 once block 0 is full.
static size_t fillBlockZero(Rider* r, size_t n, uint8_t exceptRow) {
  for (uint8_t row = 0; row < SLOTS_PER_BLOCK; row++)
    if (row != exceptRow) addLeased(r, n++, 5000u + row, (uint8_t)(row * BLOCKS), 1);
  return n;
}

void test_a_car_alone_in_its_row_gets_the_other_three() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(4, s.slot());
  TEST_ASSERT_EQUAL_HEX32(bits({5, 6, 7}), s.extraSlots());
}

void test_two_in_a_row_each_get_the_block_opposite() {
  // Block 0 full, so we land on row 0 block 1 beside the car on slot 0. Free
  // blocks 2 and 3 go one each, 500 ms after each lease: 2 Hz, evenly spaced.
  Rider r[ROSTER] = {};
  size_t n = fillBlockZero(r, 0, 0);
  addLeased(r, n++, 100, 0, 1);
  Schedule s;
  join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(1, s.slot());
  TEST_ASSERT_EQUAL_HEX32(bits({3}), s.extraSlots());

  // And the car on slot 0, seeing us on slot 1, takes block 2.
  Rider q[ROSTER] = {};
  n = fillBlockZero(q, 0, 0);
  addLeased(q, n++, 300, 1, 1);
  Schedule t;
  join(t, 100, q);
  TEST_ASSERT_EQUAL_UINT8(0, t.slot());
  TEST_ASSERT_EQUAL_HEX32(bits({2}), t.extraSlots());
}

void test_three_in_a_row_leave_the_last_block_to_the_middle_one() {
  Rider r[ROSTER] = {};
  size_t n = fillBlockZero(r, 0, 0);
  addLeased(r, n++, 100, 0, 1);
  addLeased(r, n++, 200, 2, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(1, s.slot());
  TEST_ASSERT_EQUAL_HEX32(bits({3}), s.extraSlots());

  // A full row has nothing left.
  addLeased(r, n++, 400, 3, 1, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_HEX32(0, s.extraSlots());
}

void test_a_neighbours_slot_map_counts_as_a_lease_in_the_row() {
  // Somebody we cannot hear holds slot 6 (row 1, block 2) and a neighbour
  // hears them. Slot 7 then goes to block 2 by the preceding rule, not to us.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_EQUAL_UINT8(4, s.slot());
  r[0].pos.slotMap[6] = slotTag(777);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_HEX32(bits({5}), s.extraSlots());
}

void test_extras_stop_while_a_car_waits_for_a_slot() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_NOT_EQUAL(0, s.extraSlots());

  // A joiner, still listening: its first claim must find the free slots quiet.
  addRider(r, 1, 900, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_HEX32(0, s.extraSlots());

  // A car that has lost a contested slot is waiting too.
  r[1].pos.slot = 0;
  r[1].pos.leaseGen = 7;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_HEX32(0, s.extraSlots());

  // Once it holds a slot of its own, extras come back, recomputed.
  r[1].pos.slot = 8;
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_HEX32(bits({5, 6, 7}), s.extraSlots());
}

void test_no_extras_alone_or_unleased() {
  Schedule alone;
  Rider none[ROSTER] = {};
  alone.rebuild(300, false, none, ROSTER, 0);
  TEST_ASSERT_EQUAL_HEX32(0, alone.extraSlots());
  TEST_ASSERT_FALSE(alone.inExtraSlot(0));

  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1, 1000);
  Schedule listening;
  listening.rebuild(300, false, r, ROSTER, 1000);
  TEST_ASSERT_EQUAL_HEX32(0, listening.extraSlots());
}

void test_an_extra_slot_opens_only_in_its_own_window() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  join(s, 300, r);
  const uint32_t five = slotStartMs(5);
  TEST_ASSERT_TRUE(s.inExtraSlotAtPhase(five));
  TEST_ASSERT_TRUE(s.inExtraSlotAtPhase(five + SLOT_MS - SLOT_GUARD_MS));
  TEST_ASSERT_FALSE(s.inExtraSlotAtPhase(five + SLOT_MS - SLOT_GUARD_MS + 1));
  // Never in our lease slot, nor anybody else's.
  TEST_ASSERT_FALSE(s.inExtraSlotAtPhase(slotStartMs(4)));
  TEST_ASSERT_FALSE(s.inExtraSlotAtPhase(slotStartMs(0)));
  // Without an epoch there is no knowing where slot 5 is.
  TEST_ASSERT_FALSE(s.inExtraSlot(12345));
  s.syncTo(10000, 0);
  TEST_ASSERT_TRUE(s.inExtraSlot(10000 + five));
}

void test_extras_are_disjoint_and_fill_every_occupied_row() {
  // Every ride size from 2 to 32, in the lease order cars fall into. Each car
  // computes its extras from the same roster; together they never overlap
  // each other or a lease, never pass 4 Hz, and leave no slot of an occupied
  // row idle. Past a full roster (MAX_RIDERS others) a car may be missing
  // somebody, so extras stop altogether.
  uint8_t order[MAX_SLOTS];
  uint8_t k = 0;
  for (uint8_t block = 0; block < BLOCKS; block++)
    for (uint8_t row = 0; row < SLOTS_PER_BLOCK; row++) order[k++] = (uint8_t)(row * BLOCKS + block);

  for (uint8_t cars = 2; cars <= MAX_SLOTS; cars++) {
    uint32_t used = 0;
    for (uint8_t i = 0; i < cars; i++) used |= 1u << order[i];
    uint32_t extras = 0;
    for (uint8_t me = 0; me < cars; me++) {
      // Everyone else leased, so our own slot is the first free one in lease
      // order and the join lands where this car sits.
      Rider r[ROSTER] = {};
      size_t n = 0;
      for (uint8_t other = 0; other < cars; other++)
        if (other != me) addLeased(r, n++, 1000u + other, order[other], 1);
      Schedule s;
      join(s, 1000u + me, r);
      TEST_ASSERT_EQUAL_UINT8(order[me], s.slot());
      const uint32_t mine = s.extraSlots();
      if (cars > MAX_RIDERS) {
        TEST_ASSERT_EQUAL_HEX32(0, mine);
        continue;
      }
      TEST_ASSERT_EQUAL_HEX32(0, mine & used);
      TEST_ASSERT_EQUAL_HEX32(0, mine & extras);
      TEST_ASSERT_TRUE(__builtin_popcount(mine) <= 3);
      extras |= mine;
    }
    if (cars > MAX_RIDERS) continue;
    uint32_t occupiedRows = 0;
    for (uint8_t i = 0; i < cars; i++) occupiedRows |= 0xFu << (order[i] / BLOCKS * BLOCKS);
    TEST_ASSERT_EQUAL_HEX32(occupiedRows, used | extras);
  }
}

void test_a_drowned_lease_listens_again_before_reclaiming() {
  // Two neighbours whose maps show nobody on our slot for a whole window: a
  // clash. We drop the lease, stop extras, and queue again as a joiner.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 8, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());
  now += HEARD_WINDOW_MS + 100;
  touch(r, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_HEX32(0, s.extraSlots());

  touch(r, now + 1000);
  s.rebuild(300, false, r, ROSTER, now + 1000);
  TEST_ASSERT_FALSE(s.claimed());
  touch(r, now + JOIN_LISTEN_MS);
  s.rebuild(300, false, r, ROSTER, now + JOIN_LISTEN_MS);
  TEST_ASSERT_TRUE(s.claimed());
}

// ---- Why a lease was given up (build 46) --------------------------------------

void test_a_drowned_lease_says_so() {
  // test_a_drowned_lease_listens_again_before_reclaiming, read back.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 8, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  const uint8_t mine = s.slot();
  const uint16_t gen = s.leaseGeneration();
  TEST_ASSERT_EQUAL_UINT32(0, s.losses());
  now += HEARD_WINDOW_MS + 100;
  touch(r, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_UINT32(1, s.losses());
  const SlotLoss& loss = s.lastLoss();
  TEST_ASSERT_EQUAL_UINT8(LOSS_DROWNED, loss.why);
  TEST_ASSERT_EQUAL_UINT8(mine, loss.slot);
  TEST_ASSERT_EQUAL_UINT16(gen, loss.leaseGen);
  TEST_ASSERT_EQUAL_UINT32(HEARD_WINDOW_MS + 100, loss.heldMs);
  TEST_ASSERT_EQUAL_UINT8(2, loss.judges);
  TEST_ASSERT_EQUAL_UINT8(0, loss.hearUs);
  TEST_ASSERT_EQUAL_UINT8(0, loss.hearOther);
  TEST_ASSERT_FALSE(loss.established);
  // A lease this young goes on the pass the clash starts.
  TEST_ASSERT_EQUAL_UINT32(0, loss.clashMs);
}

void test_an_unheard_lease_says_so() {
  // test_a_lease_no_map_has_shown_for_a_while_is_given_up: one neighbour, so
  // its silence is never a clash, only a lease nobody reports hearing.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  const uint32_t joined = join(s, 300, r);
  touch(r, joined + UNHEARD_MS);
  s.rebuild(300, false, r, ROSTER, joined + UNHEARD_MS);
  TEST_ASSERT_FALSE(s.claimed());
  const SlotLoss& loss = s.lastLoss();
  TEST_ASSERT_EQUAL_UINT8(LOSS_UNHEARD, loss.why);
  TEST_ASSERT_EQUAL_UINT32(UNHEARD_MS, loss.heardUsAgoMs);
  TEST_ASSERT_TRUE(loss.established);
  TEST_ASSERT_EQUAL_UINT8(1, loss.judges);
}

void test_an_outranked_lease_says_so() {
  // Every map shows us, so nothing is drowned; but a car holding our slot on
  // an older lease turns up, and the older lease keeps it.
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  addLeased(r, 1, 900, 8, 1);
  Schedule s;
  uint32_t now = join(s, 300, r);
  const uint8_t mine = s.slot();
  TEST_ASSERT_TRUE(leaseOlder(1, s.leaseGeneration()));
  addLeased(r, 2, 200, mine, 1);
  for (size_t i = 0; i < 3; i++) r[i].pos.slotMap[mine] = slotTag(300);
  now += HEARD_WINDOW_MS + 100;
  touch(r, now);
  s.rebuild(300, false, r, ROSTER, now);
  TEST_ASSERT_EQUAL_UINT32(1, s.losses());
  TEST_ASSERT_EQUAL_UINT8(LOSS_OUTRANKED, s.lastLoss().why);
  TEST_ASSERT_EQUAL_UINT8(mine, s.lastLoss().slot);
  TEST_ASSERT_EQUAL_UINT8(3, s.lastLoss().hearUs);
}

void test_a_lease_left_alone_says_so() {
  Rider r[ROSTER] = {};
  addLeased(r, 0, 100, 0, 1);
  Schedule s;
  const uint32_t joined = join(s, 300, r);
  TEST_ASSERT_TRUE(s.claimed());
  // Nobody heard for a whole lease.
  s.rebuild(300, false, r, ROSTER, joined + LEASE_MS);
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_EQUAL_UINT8(LOSS_ALONE, s.lastLoss().why);
  TEST_ASSERT_EQUAL_UINT32(1, s.losses());
  // And an unleased car that stays alone loses nothing more.
  s.rebuild(300, false, r, ROSTER, joined + 2 * LEASE_MS);
  TEST_ASSERT_EQUAL_UINT32(1, s.losses());
}

void test_a_loss_formats_for_serial_and_the_phone() {
  SlotLoss loss;
  loss.why = LOSS_OUTRANKED | LOSS_DROWNED;
  loss.slot = 0;
  loss.leaseGen = 21;
  loss.heldMs = 12400;
  loss.heardUsAgoMs = 3100;
  loss.clashMs = 3000;
  loss.hearOther = 1;
  loss.judges = 2;
  loss.established = true;
  char out[200];
  TEST_ASSERT_TRUE(formatSlotLoss(loss, 7, false, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("ls why=3 n=7 s=0 g=21 held=12400 hu=3100 cl=3000 us=0 ot=1 jd=2 es=1 pr=0", out);
  TEST_ASSERT_TRUE(formatSlotLoss(loss, 7, true, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("{\"ls\":{\"why\":3,\"n\":7,\"s\":0,\"g\":21,\"held\":12400,\"hu\":3100,\"cl\":3000,"
                           "\"us\":0,\"ot\":1,\"jd\":2,\"es\":1,\"pr\":0}}",
                           out);
  // Too small is nothing, not an overrun.
  TEST_ASSERT_EQUAL_UINT32(0, formatSlotLoss(loss, 7, false, out, 10));

  char words[40];
  TEST_ASSERT_EQUAL_STRING("outranked+drowned", slotLossWords(loss.why, words, sizeof(words)));
  TEST_ASSERT_EQUAL_STRING("alone", slotLossWords(LOSS_ALONE, words, sizeof(words)));
  TEST_ASSERT_EQUAL_STRING("none", slotLossWords(0, words, sizeof(words)));
  char tiny[6];
  TEST_ASSERT_EQUAL_STRING("outra", slotLossWords(LOSS_OUTRANKED, tiny, sizeof(tiny)));
}

void test_the_extra_flag_round_trips() {
  Position p{};
  p.extra = true;
  p.slot = 4;
  uint8_t wire[POSITION_MIN];
  size_t n = encodePosition(p, wire, sizeof(wire));
  Position got{};
  TEST_ASSERT_TRUE(decodePosition(wire, n, got));
  TEST_ASSERT_TRUE(got.extra);
  TEST_ASSERT_FALSE(got.clockLocked);
  p.extra = false;
  n = encodePosition(p, wire, sizeof(wire));
  TEST_ASSERT_TRUE(decodePosition(wire, n, got));
  TEST_ASSERT_FALSE(got.extra);
}

// ---- The simulator ----------------------------------------------------------
//
// Millisecond steps. Each car runs the module's beacon logic against its own
// drifting millis(): a 5 ms tick, a 1 s deadline grid, the schedule deciding
// when it may start. A frame is on the air for AIR_MS and is lost at any
// receiver that hears another frame overlapping it, or is transmitting
// itself. Receivers sync to direct copies from their parent before dedupe,
// note the position and rebuild, as drainRadio does. Forwards (FAST_HOPS = 2)
// are delivered 20 ms per hop without taking airtime: flooding is step 5's
// problem, and modelling it here would measure that instead of the schedule.
// A link can also lose a share of its frames outright (lossPct), each way,
// forwards included: the weak radio the bench keeps.

namespace sim {

// A position frame: 77 bytes of ESP-NOW payload (55 of position, 14 of header,
// 8 of tag) plus about 43 of 802.11 framing, near 120 bytes at 250 kbps. The
// receive callback fires once it has all arrived, so the listener's time stamp
// is this late too.
const uint32_t AIR_MS = 4;
const uint32_t TICK_MS = 5;
const uint32_t SYNC_BIAS_MS = 3; // as the module: TICK_MS / 2 + 1
const uint32_t RELAY_MS = 20;
const uint8_t RELAY_HOPS = 2;

uint32_t rng = 12345;
uint32_t rand32() {
  rng = rng * 1664525u + 1013904223u;
  return rng >> 8;
}

struct Tx {
  size_t src;
  uint32_t id;
  uint32_t start, end;
  Position pos;
  uint32_t fixAt; // when the position it carries was measured
};

struct Delivery {
  uint32_t at;
  size_t to, src;
  uint32_t id;
  Position pos;
  uint8_t hopsAway;
  size_t via = SIZE_MAX; // the forwarder, for a relayed copy
};

struct Car {
  uint32_t id = 0;
  bool on = false;
  bool gps = false;
  uint32_t bootAt = 0;
  uint32_t localBase = 0;
  double driftPpm = 0;
  uint32_t tickPhase = 0;
  size_t rosterSeats = MAX_RIDERS;
  Schedule sched;
  Rider roster[ROSTER];
  std::vector<uint32_t> seen; // newest frame id heard from each car
  bool want = false, sentOnce = false;
  uint32_t nextBeacon = 0;
  uint32_t frameId = 0; // lives in NVS, so it survives a reboot
  size_t lastSyncSrc = SIZE_MAX;
  uint32_t lastSyncId = 0;
  uint32_t sent = 0;
  uint32_t slotChanges = 0;
  uint8_t lastSlot = SLOT_NONE;
  // How often the phone hands the radio a new fix. An extra beacon only goes
  // out when there is one it has not sent.
  uint32_t fixEveryMs = 250;
  uint32_t sentFix = 0;
  uint32_t lastSendAt = 0;
  uint32_t extrasSent = 0;
  uint8_t announced = SLOT_NONE;
};

struct World {
  std::vector<Car> cars;
  std::vector<std::vector<bool>> link;
  // Per cent of frames lost on a link that is up, sender to receiver.
  std::vector<std::vector<uint8_t>> lossPct;
  std::vector<Tx> air;
  std::vector<Delivery> pending;
  uint32_t now = 0;

  // Losses where both frames were in leased slots: a schedule failure. Losses
  // involving the shared window are joiners contending, which is expected.
  uint32_t statsFrom = 0;
  uint32_t leasedCollisions = 0;
  uint32_t otherCollisions = 0;
  // Last time the ride was not fully leased on distinct slots.
  uint32_t lastUnsettled = 0;
  // Print every leased-slot collision, for chasing one down by hand.
  bool verbose = false;
  // Direct receptions of each sender's frames since mark(), summed over all
  // receivers. And the age of each fix the first time a receiver hears it,
  // which is what a faster rate is for: repeats of an old fix do not count.
  std::vector<uint32_t> heardFrom;
  std::vector<std::vector<uint32_t>> newestFixHeard;
  uint64_t ageSumMs = 0;
  uint32_t ageCount = 0;

  explicit World(size_t n, uint32_t seed) {
    rng = seed;
    cars.resize(n);
    heardFrom.assign(n, 0);
    newestFixHeard.assign(n, std::vector<uint32_t>(n, 0));
    link.assign(n, std::vector<bool>(n, true));
    lossPct.assign(n, std::vector<uint8_t>(n, 0));
    for (size_t i = 0; i < n; i++) {
      // Hash-like node numbers, as Meshtastic's are.
      bool unique;
      do {
        cars[i].id = 0x10000000u + (rand32() & 0x0FFFFFFFu);
        unique = true;
        for (size_t j = 0; j < i; j++) unique = unique && cars[j].id != cars[i].id;
      } while (!unique);
      cars[i].driftPpm = (double)((int32_t)(rand32() % 81) - 40); // +/- 40 ppm crystals
      cars[i].seen.assign(n, 0);
      link[i][i] = false;
    }
  }

  uint32_t local(const Car& c, uint32_t t) const {
    const double elapsed = (double)(t - c.bootAt);
    return c.localBase + (uint32_t)(elapsed * (1.0 + c.driftPpm * 1e-6));
  }

  void setLink(size_t a, size_t b, bool up) {
    link[a][b] = up;
    link[b][a] = up;
  }

  void setLoss(size_t a, size_t b, uint8_t pct) {
    lossPct[a][b] = pct;
    lossPct[b][a] = pct;
  }

  // Drawn only on a lossy link, so the lossless scenarios keep their numbers.
  bool lost(size_t from, size_t to) { return lossPct[from][to] > 0 && rand32() % 100 < lossPct[from][to]; }

  void boot(size_t i) {
    Car& c = cars[i];
    c.on = true;
    c.bootAt = now;
    c.localBase = rand32() % 100000;
    c.tickPhase = rand32() % TICK_MS;
    c.sched.reset();
    for (size_t k = 0; k < ROSTER; k++) c.roster[k] = Rider{};
    std::fill(c.seen.begin(), c.seen.end(), 0);
    c.want = c.sentOnce = false;
    c.lastSendAt = now;
    c.announced = SLOT_NONE;
    c.nextBeacon = 0;
    c.lastSyncSrc = SIZE_MAX;
    c.lastSlot = SLOT_NONE;
  }

  void off(size_t i) { cars[i].on = false; }

  void note(Car& c, uint32_t src, const Position& p, uint8_t hopsAway, uint32_t at) {
    Rider* seat = nullptr;
    size_t filled = 0;
    for (size_t k = 0; k < ROSTER; k++) {
      if (!c.roster[k].used) continue;
      filled++;
      if (c.roster[k].id == src) seat = &c.roster[k];
    }
    if (seat == nullptr && filled < c.rosterSeats)
      for (size_t k = 0; k < ROSTER && seat == nullptr; k++)
        if (!c.roster[k].used) seat = &c.roster[k];
    if (seat == nullptr) {
      // Full: like Mesh::note, only a car gone stale gives up its seat.
      for (size_t k = 0; k < ROSTER; k++)
        if (c.roster[k].used && (uint32_t)(at - c.roster[k].atMs) > RIDER_STALE_MS)
          seat = &c.roster[k];
      if (seat == nullptr) return;
    }
    seat->id = src;
    seat->pos = p;
    seat->atMs = at;
    seat->hopsAway = hopsAway;
    seat->via = HEARD_FAST;
    seat->used = true;
  }

  void receive(const Delivery& d) {
    Car& c = cars[d.to];
    if (!c.on) return;
    const uint32_t at = local(c, now);
    const uint32_t srcId = cars[d.src].id;

    if (d.hopsAway == 0 && !d.pos.extra && c.sched.takesClockFrom(srcId) &&
        !(d.src == c.lastSyncSrc && d.id == c.lastSyncId)) {
      c.lastSyncSrc = d.src;
      c.lastSyncId = d.id;
      c.sched.syncTo(at, d.pos.slot, SYNC_BIAS_MS);
    }

    if (d.id <= c.seen[d.src]) return;
    c.seen[d.src] = d.id;
    note(c, srcId, d.pos, d.hopsAway, at);
    if (d.hopsAway == 0 && !d.pos.extra) c.sched.heardBeacon(srcId, d.pos, at);
    c.sched.rebuild(c.id, c.gps, c.roster, ROSTER, at);

    // Extras are never forwarded.
    if (d.hopsAway < RELAY_HOPS && !d.pos.extra)
      for (size_t k = 0; k < cars.size(); k++)
        if (link[d.to][k] && k != d.src)
          pending.push_back({now + RELAY_MS, k, d.src, d.id, d.pos, (uint8_t)(d.hopsAway + 1), d.to});
  }

  uint32_t fixNow(const Car& c) const { return now / c.fixEveryMs; }

  Tx beaconFrom(size_t i, uint32_t at) {
    Car& c = cars[i];
    Tx tx;
    tx.src = i;
    tx.id = ++c.frameId;
    tx.start = now;
    tx.end = now + AIR_MS;
    tx.fixAt = fixNow(c) * c.fixEveryMs;
    tx.pos.slot = c.sched.slot();
    tx.pos.leaseGen = c.sched.leaseGeneration();
    tx.pos.schedGen = c.sched.generation();
    c.sched.fillSlotMap(at, tx.pos.slotMap);
    tx.pos.clockLocked = c.gps;
    tx.pos.refId = c.sched.referenceId();
    tx.pos.refHops = c.sched.hopsToReference();
    tx.pos.refLocked = c.sched.referenceLocked();
    tx.pos.fitToKeepTime = c.sched.announceFit();
    tx.pos.refFit = c.sched.referenceFit();
    c.sentFix = fixNow(c);
    c.lastSendAt = now;
    return tx;
  }

  // TougeFastModule::sendExtraBeacon: a fix not yet sent, in an extra slot.
  bool extraTick(size_t i, uint32_t at) {
    Car& c = cars[i];
    if (!c.sentOnce || fixNow(c) == c.sentFix || now - c.lastSendAt < 100) return false;
    const bool mine =
        c.gps ? c.sched.inExtraSlotAtPhase(now % SCHEDULE_MS) : c.sched.inExtraSlot(at);
    if (!mine) return false;
    Tx tx = beaconFrom(i, at);
    tx.pos.extra = true;
    air.push_back(tx);
    c.extrasSent++;
    return true;
  }

  void tick(size_t i) {
    Car& c = cars[i];
    const uint32_t at = local(c, now);
    if (extraTick(i, at)) return;
    if (!c.want) {
      if (!c.sentOnce || (int32_t)(at - c.nextBeacon) >= 0) {
        c.want = true;
        c.sched.rebuild(c.id, c.gps, c.roster, ROSTER, at);
        c.sched.drawSharedTurn(rand32());
      }
    }
    // As the module: a new lease is announced in its first slot.
    if (c.sched.claimed() && c.sched.slot() != c.announced) c.want = true;
    if (!c.want) return;
    if (c.sched.weAreReference()) c.sched.startEpoch(at);
    // A PPS edge on the true second, whatever the crystal thinks.
    const bool mine = c.gps ? c.sched.inSlotAtPhase(now % SCHEDULE_MS) : c.sched.inSlot(at);
    if (!mine) return;

    c.want = false;
    c.announced = c.sched.slot();
    if (c.nextBeacon == 0) c.nextBeacon = at;
    c.nextBeacon = nextOnGrid(c.nextBeacon, 1000, at);
    c.sentOnce = true;
    c.sent++;
    air.push_back(beaconFrom(i, at));
  }

  void endTransmissions() {
    for (size_t t = 0; t < air.size(); t++) {
      if (air[t].end != now) continue;
      const Tx tx = air[t];
      for (size_t j = 0; j < cars.size(); j++) {
        if (!cars[j].on || !link[tx.src][j] || lost(tx.src, j)) continue;
        const Tx* clash = nullptr;
        for (const Tx& other : air) {
          if (&other == &air[t]) continue;
          const bool overlaps = other.start < tx.end && other.end > tx.start;
          if (overlaps && (other.src == j || link[j][other.src])) clash = &other;
        }
        if (clash == nullptr) {
          if (now >= statsFrom) {
            heardFrom[tx.src]++;
            uint32_t& newest = newestFixHeard[j][tx.src];
            if (tx.fixAt > newest) {
              newest = tx.fixAt;
              ageSumMs += now - tx.fixAt;
              ageCount++;
            }
          }
          receive({now, j, tx.src, tx.id, tx.pos, 0});
        } else if (now >= statsFrom) {
          const bool bothLeased = tx.pos.slot < MAX_SLOTS && clash->pos.slot < MAX_SLOTS;
          (bothLeased ? leasedCollisions : otherCollisions)++;
          if (verbose && bothLeased)
            printf("t=%u at car %zu: car %zu (slot %u, sent %u) hit car %zu (slot %u, sent %u)\n",
                   (unsigned)now, j, tx.src, tx.pos.slot, (unsigned)tx.start, clash->src,
                   clash->pos.slot, (unsigned)clash->start);
        }
      }
    }
  }

  bool interferes(size_t a, size_t b) const {
    if (link[a][b]) return true;
    for (size_t k = 0; k < cars.size(); k++)
      if (cars[k].on && link[a][k] && link[b][k]) return true;
    return false;
  }

  // Every car on holds a lease, and no two that could collide share one.
  bool settled() const {
    for (size_t a = 0; a < cars.size(); a++) {
      if (!cars[a].on) continue;
      if (!cars[a].sched.claimed()) return false;
      for (size_t b = a + 1; b < cars.size(); b++)
        if (cars[b].on && cars[a].sched.slot() == cars[b].sched.slot() && interferes(a, b))
          return false;
    }
    return true;
  }

  void run(uint32_t forMs) {
    const uint32_t until = now + forMs;
    for (; now < until; now++) {
      endTransmissions();
      for (size_t k = 0; k < pending.size();) {
        if (pending[k].at == now) {
          const Delivery d = pending[k];
          pending[k] = pending.back();
          pending.pop_back();
          if (d.via != SIZE_MAX && lost(d.via, d.to)) continue;
          receive(d);
        } else {
          k++;
        }
      }
      for (size_t i = 0; i < cars.size(); i++) {
        Car& c = cars[i];
        if (c.on && (now - c.bootAt) % TICK_MS == c.tickPhase) tick(i);
      }
      for (size_t t = 0; t < air.size();) {
        if (air[t].end + 50 < now) {
          air[t] = air.back();
          air.pop_back();
        } else {
          t++;
        }
      }
      if (now % 100 == 0) {
        if (!settled()) lastUnsettled = now;
        for (Car& c : cars) {
          if (!c.on) continue;
          if (c.sched.slot() != c.lastSlot) c.slotChanges++;
          c.lastSlot = c.sched.slot();
        }
      }
    }
  }

  // Start counting afresh: collisions, and slot moves by cars already leased.
  void mark() {
    statsFrom = now;
    leasedCollisions = otherCollisions = 0;
    for (Car& c : cars) {
      c.slotChanges = 0;
      c.sent = 0;
      c.extrasSent = 0;
    }
    std::fill(heardFrom.begin(), heardFrom.end(), 0);
    // Ages count only fixes taken after this point, so the first one heard
    // from each car is not a stale one from before the mark.
    for (auto& row : newestFixHeard) std::fill(row.begin(), row.end(), now);
    ageSumMs = 0;
    ageCount = 0;
  }

  // How many times a second car i was heard directly by one of the cars in
  // range of it, averaged over those cars, since mark().
  double rateHeard(size_t i) const {
    size_t hearers = 0;
    for (size_t j = 0; j < cars.size(); j++)
      if (j != i && cars[j].on && link[i][j]) hearers++;
    if (hearers == 0 || now == statsFrom) return 0;
    return heardFrom[i] * 1000.0 / hearers / (double)(now - statsFrom);
  }

  size_t references() const {
    size_t n = 0;
    for (const Car& c : cars)
      if (c.on && c.sched.weAreReference()) n++;
    return n;
  }
};

void report(const char* what, const World& w, uint32_t eventAt) {
  char line[200];
  snprintf(line, sizeof(line), "%s: settled %u ms after the event, %u leased-slot collisions, %u shared",
           what, (unsigned)(w.lastUnsettled + 100 - eventAt), (unsigned)w.leasedCollisions,
           (unsigned)w.otherCollisions);
  TEST_MESSAGE(line);
}

// Power every car on over three seconds and let the ride settle.
void bootAll(World& w, size_t first, size_t last) {
  std::vector<uint32_t> at;
  for (size_t i = first; i < last; i++) at.push_back(rand32() % 3000);
  for (uint32_t t = 0; t < 3000; t++) {
    for (size_t i = first; i < last; i++)
      if (at[i - first] == t) w.boot(i);
    w.run(1);
  }
}

uint32_t extrasSent(const World& w) {
  uint32_t n = 0;
  for (const Car& c : w.cars) n += c.extrasSent;
  return n;
}

// Every car's frames a second as its neighbours hear them, summed.
double framesPerSecond(const World& w) {
  double total = 0;
  for (size_t i = 0; i < w.cars.size(); i++)
    if (w.cars[i].on) total += w.rateHeard(i);
  return total;
}

// What watch() saw, sampled every 100 ms: how long `car` shared a slot with a
// car it could collide with (in all, and the longest stretch), how long it held
// a slot, how long the ride named more than one reference, how long any car
// named `car` (and how long a car other than `car` did: the ones that would
// take their clock from it), and how many times each car's reference changed.
struct Watch {
  size_t car = SIZE_MAX;
  uint32_t sharedMs = 0, longestSharedMs = 0, stretchMs = 0, leasedMs = 0;
  uint32_t splitMs = 0, namedCarMs = 0, namedByOthersMs = 0;
  std::vector<uint32_t> lastRef, refChanges;

  uint32_t mostRefChanges() const {
    uint32_t most = 0;
    for (uint32_t n : refChanges) most = n > most ? n : most;
    return most;
  }
};

void watch(World& w, Watch& out, uint32_t forMs) {
  const size_t n = w.cars.size();
  if (out.lastRef.empty()) {
    out.refChanges.assign(n, 0);
    for (const Car& c : w.cars) out.lastRef.push_back(c.sched.referenceId());
  }
  for (uint32_t t = 0; t < forMs; t += 100) {
    w.run(100);
    bool shared = false;
    if (out.car < n && w.cars[out.car].on && w.cars[out.car].sched.claimed()) {
      out.leasedMs += 100;
      for (size_t j = 0; j < n; j++)
        if (j != out.car && w.cars[j].on && w.cars[j].sched.slot() == w.cars[out.car].sched.slot() &&
            w.interferes(out.car, j))
          shared = true;
    }
    out.stretchMs = shared ? out.stretchMs + 100 : 0;
    if (shared) out.sharedMs += 100;
    if (out.stretchMs > out.longestSharedMs) out.longestSharedMs = out.stretchMs;

    uint32_t named = 0;
    bool split = false, namesCar = false, othersNameCar = false;
    for (size_t k = 0; k < n; k++) {
      if (!w.cars[k].on) continue;
      const uint32_t ref = w.cars[k].sched.referenceId();
      if (named == 0) named = ref;
      else if (ref != named) split = true;
      if (out.car < n && ref == w.cars[out.car].id) {
        namesCar = true;
        if (k != out.car) othersNameCar = true;
      }
      if (ref != out.lastRef[k]) {
        out.refChanges[k]++;
        out.lastRef[k] = ref;
      }
    }
    if (split) out.splitMs += 100;
    if (namesCar) out.namedCarMs += 100;
    if (othersNameCar) out.namedByOthersMs += 100;
  }
}

} // namespace sim

using sim::World;

// The bound every scenario is held to: listen, claim, one conflict round, and
// a second for the clock, with room over what the runs actually take.
static const uint32_t SETTLE_BOUND_MS = 10000;

static void carPark(size_t n, uint32_t seed, size_t seats) {
  World w(n, seed);
  for (sim::Car& c : w.cars) c.rosterSeats = seats;
  sim::bootAll(w, 0, n);
  w.run(15000);
  TEST_ASSERT_TRUE_MESSAGE(w.settled(), "not every car holds a distinct lease");
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= 3000 + SETTLE_BOUND_MS);
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  sim::report("car park", w, 3000);

  // Steady state: a minute with no leased-slot collision, no slot moves, and
  // every car on air once a second.
  w.mark();
  w.run(60000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  for (const sim::Car& c : w.cars) {
    TEST_ASSERT_EQUAL_UINT32(0, c.slotChanges);
    TEST_ASSERT_UINT32_WITHIN(1, 60, c.sent);
  }
}

void test_sim_25_cars_power_on_together() { carPark(25, 1, MAX_RIDERS); }

void test_sim_29_cars_fill_the_firmware_roster() { carPark(MAX_RIDERS + 1, 2, MAX_RIDERS); }

void test_sim_30_cars_with_one_more_roster_seat() {
  // 30 cars need 29 seats per roster; the firmware has MAX_RIDERS = 28. The
  // schedule itself has the slots; see the next test for what 28 seats does.
  carPark(30, 3, 29);
}

void test_sim_30_cars_overflow_a_28_seat_roster() {
  // Each car can only hold 28 of its 29 neighbours, so one lease per car is
  // invisible to it. Recorded rather than asserted as a failure, because it
  // is a roster limit, not a schedule one: the numbers go in the test log.
  World w(30, 4);
  sim::bootAll(w, 0, 30);
  w.run(15000);
  w.mark();
  w.run(30000);
  char line[160];
  snprintf(line, sizeof(line), "30 cars, 28 seats: settled=%d, %u leased-slot collisions in 30 s",
           (int)w.settled(), (unsigned)w.leasedCollisions);
  TEST_MESSAGE(line);
}

void test_sim_seeds_are_not_lucky() {
  // The same 25-car car park across more seeds, each held to the same bound.
  for (uint32_t seed = 10; seed < 20; seed++) {
    World w(25, seed);
    sim::bootAll(w, 0, 25);
    w.run(15000);
    TEST_ASSERT_TRUE(w.settled());
    TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= 3000 + SETTLE_BOUND_MS);
    w.mark();
    w.run(20000);
    TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  }
}

void test_sim_joins_do_not_move_anyone() {
  World w(25, 5);
  sim::bootAll(w, 0, 20);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  w.mark();

  // Five more arrive a few seconds apart.
  const uint32_t firstJoin = w.now;
  for (size_t i = 20; i < 25; i++) {
    w.boot(i);
    w.run(2000);
  }
  const uint32_t lastJoin = w.now - 2000;
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= lastJoin + SETTLE_BOUND_MS);
  for (size_t i = 0; i < 20; i++) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  sim::report("joins", w, firstJoin);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

void test_sim_drops_and_arrivals_on_a_full_ride() {
  // 29 cars fill the firmware roster. Five leave and five others arrive,
  // then everything runs past LEASE_MS so the leavers' leases lapse.
  const size_t full = MAX_RIDERS + 1;
  World w(full + 5, 6);
  sim::bootAll(w, 0, full);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());

  for (size_t i = 0; i < 5; i++) w.off(i);
  w.mark();
  const uint32_t at = w.now;
  for (size_t i = full; i < full + 5; i++) w.boot(i);
  w.run(LEASE_MS + 10000);

  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= at + SETTLE_BOUND_MS);
  for (size_t i = 5; i < full; i++) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  sim::report("drops and arrivals", w, at);

  w.mark();
  w.run(20000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

static void rebootThree(uint32_t seed) {
  World w(25, seed);
  sim::bootAll(w, 0, 25);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  w.mark();

  // Three cars reboot, the reference among them.
  size_t ref = 0;
  for (size_t i = 0; i < 25; i++)
    if (w.cars[i].sched.weAreReference()) ref = i;
  const size_t rebooted[3] = {ref, (ref + 7) % 25, (ref + 13) % 25};
  const uint32_t at = w.now;
  for (size_t k = 0; k < 3; k++) w.boot(rebooted[k]);
  w.run(20000);

  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= at + SETTLE_BOUND_MS);
  for (size_t i = 0; i < 25; i++) {
    const bool wasRebooted = i == rebooted[0] || i == rebooted[1] || i == rebooted[2];
    if (!wasRebooted) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  }
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  if (seed == 7) sim::report("reboots", w, at);
}

void test_sim_reboots_rejoin_without_stealing() {
  for (uint32_t seed = 7; seed < 17; seed++) rebootThree(seed);
}

void test_sim_a_car_back_from_out_of_range_does_not_steal() {
  // A car is out of range past LEASE_MS, its slot goes to a newcomer, and it
  // drives back. It let its lease go while alone, so it rejoins as a newcomer.
  World w(27, 8);
  sim::bootAll(w, 0, 25);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());

  const size_t away = 4;
  const uint8_t itsSlot = w.cars[away].sched.slot();
  for (size_t j = 0; j < w.cars.size(); j++) w.setLink(away, j, false);
  w.run(LEASE_MS + 1000);
  TEST_ASSERT_FALSE(w.cars[away].sched.claimed());

  // Fill the ride so the lowest free slot is the one it left.
  w.boot(25);
  w.run(8000);
  w.mark();
  const uint8_t taken = w.cars[25].sched.slot();

  for (size_t j = 0; j < w.cars.size(); j++) w.setLink(away, j, j != away);
  const uint32_t back = w.now;
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= back + SETTLE_BOUND_MS);
  TEST_ASSERT_EQUAL_UINT8(taken, w.cars[25].sched.slot());
  TEST_ASSERT_EQUAL_UINT32(0, w.cars[25].slotChanges);
  TEST_ASSERT_TRUE(w.cars[away].sched.claimed());
  TEST_ASSERT_NOT_EQUAL(taken, w.cars[away].sched.slot());
  char line[120];
  snprintf(line, sizeof(line), "out of range: left slot %u, newcomer took %u, returner now on %u",
           (unsigned)itsSlot, (unsigned)taken, (unsigned)w.cars[away].sched.slot());
  TEST_MESSAGE(line);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

void test_sim_losing_the_reference_changes_no_lease() {
  World w(25, 9);
  sim::bootAll(w, 0, 25);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());

  size_t ref = 0;
  for (size_t i = 0; i < 25; i++)
    if (w.cars[i].sched.weAreReference()) ref = i;
  uint16_t genBefore[25];
  for (size_t i = 0; i < 25; i++) genBefore[i] = w.cars[i].sched.generation();

  w.mark();
  w.off(ref);
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  TEST_ASSERT_FALSE(w.cars[ref].sched.weAreReference() && w.cars[ref].on);
  for (size_t i = 0; i < 25; i++) {
    if (i == ref) continue;
    TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
    TEST_ASSERT_FALSE(leaseOlder(w.cars[i].sched.generation(), genBefore[i]));
  }
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

void test_sim_two_groups_merge_into_one_schedule() {
  // 13 and 12 cars that have never heard each other each settle on their own
  // reference, clock and leases, then meet.
  World w(25, 11);
  for (size_t a = 0; a < 13; a++)
    for (size_t b = 13; b < 25; b++) w.setLink(a, b, false);
  sim::bootAll(w, 0, 25);
  w.run(15000);
  TEST_ASSERT_EQUAL_UINT32(2, w.references());

  // Both sides started from slot 0, so most slots clash.
  size_t clashes = 0;
  for (size_t a = 0; a < 13; a++)
    for (size_t b = 13; b < 25; b++)
      if (w.cars[a].sched.slot() == w.cars[b].sched.slot()) clashes++;
  TEST_ASSERT_TRUE(clashes >= 10);

  // Each side is sending extras in the slots it thinks are free, which are
  // the other side's leases.
  w.mark();
  w.run(3000);
  TEST_ASSERT_TRUE(sim::extrasSent(w) > 0);

  w.mark();
  const uint32_t met = w.now;
  for (size_t a = 0; a < 13; a++)
    for (size_t b = 13; b < 25; b++) w.setLink(a, b, true);
  w.run(20000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= met + SETTLE_BOUND_MS);
  sim::report("merge", w, met);

  // And it stays merged, with the second filled again: 25 cars use all 32.
  w.mark();
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  for (const sim::Car& c : w.cars) TEST_ASSERT_EQUAL_UINT32(0, c.slotChanges);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 32.0f, (float)sim::framesPerSecond(w));
}

void test_sim_a_strung_out_convoy_keeps_its_slots_apart() {
  // 25 cars in a line, each hearing the four either side. Positions reach
  // twelve either side through two forwards, so every car in a two-hop
  // interference range is on the roster; the clock is handed down the line.
  World w(25, 12);
  for (size_t a = 0; a < 25; a++)
    for (size_t b = 0; b < 25; b++)
      w.setLink(a, b, a != b && (a > b ? a - b : b - a) <= 4);
  sim::bootAll(w, 0, 25);
  w.run(20000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  sim::report("convoy", w, 3000);
  w.mark();
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

void test_sim_some_cars_on_gps() {
  // Five cars keep time off their own pulse; one of them is the reference,
  // and the rest follow its beacons down the chain.
  World w(25, 13);
  for (size_t i = 0; i < 5; i++) w.cars[i * 5].gps = true;
  sim::bootAll(w, 0, 25);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  for (const sim::Car& c : w.cars)
    if (c.sched.weAreReference()) TEST_ASSERT_TRUE(c.gps);
  w.mark();
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
}

// ---- Extra beacons, simulated -----------------------------------------------

// Settle `cars`, then measure 20 s. Every car must be heard at 1 + its extra
// slots a second, the ride must use min(32, 4N) frames a second, and the
// number of cars at 4, 2 and 1 Hz must match the row arithmetic in
// schedule.h. No collision between scheduled frames, extras included.
static void ratesFor(size_t cars, uint32_t seed, size_t seats, int at4, int at2, int at1) {
  World w(cars, seed);
  for (sim::Car& c : w.cars) c.rosterSeats = seats;
  sim::bootAll(w, 0, cars);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  w.mark();
  w.run(20000);

  int count[5] = {};
  double slowest = 99, fastest = 0;
  for (size_t i = 0; i < cars; i++) {
    const int expected = 1 + __builtin_popcount(w.cars[i].sched.extraSlots());
    const double heard = w.rateHeard(i);
    TEST_ASSERT_FLOAT_WITHIN(0.25f, (float)expected, (float)heard);
    count[expected]++;
    if (heard < slowest) slowest = heard;
    if (heard > fastest) fastest = heard;
  }
  TEST_ASSERT_EQUAL_INT(at4, count[4]);
  TEST_ASSERT_EQUAL_INT(at2, count[2]);
  TEST_ASSERT_EQUAL_INT(at1, count[1]);
  const double expectedTotal = cars * 4.0 < MAX_SLOTS ? cars * 4.0 : (double)MAX_SLOTS;
  TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)expectedTotal, (float)sim::framesPerSecond(w));
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);

  char line[160];
  snprintf(line, sizeof(line),
           "%u cars: %d at 4 Hz, %d at 2, %d at 1; heard %.2f-%.2f Hz; %.1f frames/s; "
           "new fix %u ms old when first heard",
           (unsigned)cars, at4, at2, at1, slowest, fastest, sim::framesPerSecond(w),
           (unsigned)(w.ageCount ? w.ageSumMs / w.ageCount : 0));
  TEST_MESSAGE(line);
}

void test_sim_rates_3_cars() { ratesFor(3, 31, MAX_RIDERS, 3, 0, 0); }
void test_sim_rates_10_cars() { ratesFor(10, 32, MAX_RIDERS, 6, 4, 0); }
void test_sim_rates_25_cars() { ratesFor(25, 33, MAX_RIDERS, 0, 7, 18); }
// 32 cars need 31 roster seats; the firmware has MAX_RIDERS = 28.
void test_sim_rates_32_cars() { ratesFor(32, 34, MAX_SLOTS - 1, 0, 0, 32); }

void test_sim_joins_take_a_slot_while_extras_run() {
  // Ten cars with every free slot in use as an extra. Three more arrive two
  // seconds apart; extras give way while each listens, so it claims into
  // silence, and nobody already leased moves.
  World w(13, 21);
  sim::bootAll(w, 0, 10);
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  w.mark();
  w.run(3000);
  TEST_ASSERT_TRUE(sim::extrasSent(w) > 0);

  w.mark();
  const uint32_t firstJoin = w.now;
  for (size_t i = 10; i < 13; i++) {
    w.boot(i);
    w.run(2000);
  }
  const uint32_t lastJoin = w.now - 2000;
  w.run(15000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= lastJoin + SETTLE_BOUND_MS);
  for (size_t i = 0; i < 10; i++) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  sim::report("joins with extras", w, firstJoin);

  // Extras come back for the new layout: 13 cars fill the second again.
  w.mark();
  w.run(20000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 32.0f, (float)sim::framesPerSecond(w));
}

void test_sim_extras_cut_the_age_of_a_1hz_phone_fix() {
  // Most phones give one GPS fix a second, so extras carry no more fixes than
  // the lease beacon would. What they buy is the wait: a new fix goes out in
  // the next of four slots instead of the next one.
  World few(3, 41);
  for (sim::Car& c : few.cars) c.fixEveryMs = 1000;
  sim::bootAll(few, 0, 3);
  few.run(15000);
  few.mark();
  few.run(30000);
  const uint32_t fewAge = (uint32_t)(few.ageSumMs / few.ageCount);

  World full(32, 42);
  for (sim::Car& c : full.cars) {
    c.fixEveryMs = 1000;
    c.rosterSeats = MAX_SLOTS - 1;
  }
  sim::bootAll(full, 0, 32);
  full.run(15000);
  full.mark();
  full.run(30000);
  const uint32_t fullAge = (uint32_t)(full.ageSumMs / full.ageCount);

  char line[120];
  snprintf(line, sizeof(line), "1 Hz phone fix, age when first heard: 3 cars %u ms, 32 cars %u ms",
           (unsigned)fewAge, (unsigned)fullAge);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(fewAge < 250);
  TEST_ASSERT_TRUE(fullAge > 400);
}

// ---- A weak radio, simulated ------------------------------------------------
//
// The bench on 2026-09-24: the moto's radio heard at -82 to -91 dBm, the other
// two at about -50, with the lowest node number of the three. With a phone on
// every radio (BLE shares the 2.4 GHz radio) the good links lost 35-50 % of
// beacons each way and the moto's 65-80 %. Modelled as 40 % and 75 %, each
// way, forwards included.

static const uint32_t WEAK_ID = 0x0FFFFFFFu; // below every id World draws

static void reportWatch(const char* what, const sim::Watch& seen, uint32_t forMs) {
  char line[200];
  snprintf(line, sizeof(line),
           "%s: weak car on a slot %u%% of the time, on a held one %u ms (longest %u); "
           "references split %u ms, at most %u changes a car",
           what, (unsigned)(seen.leasedMs * 100 / forMs), (unsigned)seen.sharedMs,
           (unsigned)seen.longestSharedMs, (unsigned)seen.splitMs, (unsigned)seen.mostRefChanges());
  TEST_MESSAGE(line);
}

void test_sim_a_weak_radio_keeps_to_a_free_slot_and_off_the_clock() {
  const size_t cars = 6, weak = 0;
  World w(cars, 51);
  w.cars[weak].id = WEAK_ID;
  for (size_t j = 1; j < cars; j++) w.setLoss(weak, j, 75);
  for (size_t a = 1; a < cars; a++)
    for (size_t b = a + 1; b < cars; b++) w.setLoss(a, b, 40);
  sim::bootAll(w, 0, cars);
  w.run(30000);
  TEST_ASSERT_TRUE_MESSAGE(w.settled(), "every car, the weak one too, on a slot of its own");

  w.mark();
  sim::Watch seen;
  seen.car = weak;
  const uint32_t forMs = 180000;
  sim::watch(w, seen, forMs);
  reportWatch("weak radio among five", seen, forMs);
  // It never sits on a slot somebody else holds for more than a moment, holds
  // one of its own nearly all the time, and nobody moves for it.
  TEST_ASSERT_TRUE(seen.longestSharedMs <= 2000);
  TEST_ASSERT_TRUE(seen.sharedMs <= 3000);
  TEST_ASSERT_TRUE(seen.leasedMs >= forMs / 100 * 95);
  for (size_t i = 0; i < cars; i++)
    if (i != weak) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  // Nobody keeps time off it, and the ride agrees on who does, steadily.
  TEST_ASSERT_EQUAL_UINT32(0, seen.namedCarMs);
  TEST_ASSERT_TRUE(seen.splitMs <= forMs / 100);
  TEST_ASSERT_TRUE(seen.mostRefChanges() <= 3);

  // The reference drives off. The ride elects another, still not the weak
  // car, and no lease moves.
  size_t ref = weak;
  for (size_t i = 0; i < cars; i++)
    if (w.cars[i].sched.weAreReference()) ref = i;
  TEST_ASSERT_TRUE(ref != weak);
  w.mark();
  w.off(ref);
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  TEST_ASSERT_FALSE(w.cars[weak].sched.weAreReference());
  const size_t witness = ref == 1 ? 2 : 1;
  for (size_t i = 0; i < cars; i++) {
    if (i == ref) continue;
    TEST_ASSERT_EQUAL_UINT32(w.cars[witness].sched.referenceId(), w.cars[i].sched.referenceId());
    if (i != weak) TEST_ASSERT_EQUAL_UINT32(0, w.cars[i].slotChanges);
  }
}

void test_sim_the_bench_three_radios_with_the_weak_one_lowest() {
  // The moto hears the Samsung's radio at a quarter of its frames and the V3
  // not at all; the Samsung's radio and the V3 lose 40 %, as good links with a
  // phone attached do. On builds 38 and 39 the moto kept the clock, the V3 took
  // it two hops away, and the moto came up on the V3's slot; on build 43, with
  // no car fit at these rates, the moto kept it again.
  const size_t moto = 0, samsung = 1, v3 = 2;
  World w(3, 52);
  w.cars[moto].id = WEAK_ID;
  w.setLink(moto, v3, false);
  w.setLoss(moto, samsung, 75);
  w.setLoss(samsung, v3, 40);
  sim::bootAll(w, 0, 3);
  w.run(30000);
  TEST_ASSERT_TRUE(w.settled());

  w.mark();
  sim::Watch seen;
  seen.car = moto;
  const uint32_t forMs = 180000;
  sim::watch(w, seen, forMs);
  reportWatch("the bench", seen, forMs);
  TEST_ASSERT_TRUE(seen.longestSharedMs <= 2000);
  // One judge, heard one frame in four: the odd lease given up for want of a
  // map showing it is allowed, as long as it is rejoined cleanly.
  TEST_ASSERT_TRUE(seen.leasedMs >= forMs / 100 * 90);
  TEST_ASSERT_EQUAL_UINT32(0, w.cars[samsung].slotChanges);
  TEST_ASSERT_EQUAL_UINT32(0, w.cars[v3].slotChanges);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  // The moto names itself whenever it goes REFERENCE_LAPSE_MS without hearing
  // the Samsung's radio, about one window in ten at 75 % loss. Nobody follows
  // it and it keeps its epoch, so the ride's clock is untouched; its own
  // disagreement is the only split allowed.
  TEST_ASSERT_EQUAL_UINT32(0, seen.namedByOthersMs);
  TEST_ASSERT_TRUE(seen.namedCarMs <= forMs / 10);
  TEST_ASSERT_TRUE(seen.splitMs <= seen.namedCarMs + forMs / 100);
  // The V3 is the reference or hears it directly.
  TEST_ASSERT_TRUE(w.cars[v3].sched.hopsToReference() <= 1);
  // At these rates the two good radios qualify to keep time and the moto does
  // not, which is what keeps the clock off it rather than luck.
  TEST_ASSERT_TRUE(w.cars[samsung].sched.fitToKeepTime());
  TEST_ASSERT_TRUE(w.cars[v3].sched.fitToKeepTime());
  TEST_ASSERT_FALSE(w.cars[moto].sched.fitToKeepTime());
}

void test_sim_the_reference_holds_while_link_quality_jitters() {
  // Six cars in range of each other, every link losing 25-50 % of its frames
  // (the bench's good links with a phone attached), redrawn every ten seconds
  // for five minutes. Link records wobble with every lost beacon; the
  // reference must not follow it.
  const size_t cars = 6;
  World w(cars, 61);
  sim::bootAll(w, 0, cars);
  w.run(20000);
  TEST_ASSERT_EQUAL_UINT32(1, w.references());

  w.mark();
  sim::Watch seen;
  for (int period = 0; period < 30; period++) {
    for (size_t a = 0; a < cars; a++)
      for (size_t b = a + 1; b < cars; b++) w.setLoss(a, b, (uint8_t)(25 + sim::rand32() % 26));
    sim::watch(w, seen, 10000);
  }
  char line[120];
  snprintf(line, sizeof(line), "jittery links: references split %u ms, at most %u changes a car in 5 min",
           (unsigned)seen.splitMs, (unsigned)seen.mostRefChanges());
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(seen.mostRefChanges() <= 3);
  TEST_ASSERT_TRUE(seen.splitMs <= 3000);
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_slots_tile_the_second_and_leave_the_shared_windows_clear);
  RUN_TEST(test_consecutive_slots_fall_in_different_blocks);
  RUN_TEST(test_the_chain_is_short_enough_to_stay_inside_a_slot);
  RUN_TEST(test_a_newcomer_listens_before_claiming);
  RUN_TEST(test_an_unleased_car_speaks_only_in_the_shared_window);
  RUN_TEST(test_a_held_slot_survives_a_car_joining_or_leaving);
  RUN_TEST(test_the_older_lease_keeps_a_contested_slot);
  RUN_TEST(test_equal_generations_go_to_the_lower_node_number);
  RUN_TEST(test_a_newcomer_never_takes_an_incumbents_slot);
  RUN_TEST(test_cars_joining_together_land_on_distinct_slots_first_time);
  RUN_TEST(test_more_cars_than_slots_wait_rather_than_double_up);
  RUN_TEST(test_a_lapsed_lease_frees_its_slot);
  RUN_TEST(test_a_car_alone_for_a_lease_lets_its_slot_go_and_listens_again);
  RUN_TEST(test_a_duplicate_node_number_does_not_corrupt_the_claim);
  RUN_TEST(test_generation_compare_survives_the_wrap);
  RUN_TEST(test_a_fresh_board_adopts_the_ride_generation_outright);
  RUN_TEST(test_a_frame_that_cannot_finish_in_the_slot_does_not_start);
  RUN_TEST(test_a_car_alone_speaks_any_time);
  RUN_TEST(test_slot_window_opens_once_per_second_after_a_sync);
  RUN_TEST(test_syncing_allows_for_the_sender_waiting_for_a_tick);
  RUN_TEST(test_sync_reads_the_slot_from_the_beacon_not_the_roster);
  RUN_TEST(test_a_beacon_from_the_shared_window_does_not_set_the_clock);
  RUN_TEST(test_slots_keep_running_across_the_millis_wrap);
  RUN_TEST(test_gps_slots_need_no_reference_car);
  RUN_TEST(test_a_reference_travels_past_the_cars_that_can_hear_it);
  RUN_TEST(test_the_reference_is_no_hops_from_itself_and_syncs_to_nobody);
  RUN_TEST(test_an_unleased_car_is_not_the_reference_while_a_leased_one_is_heard);
  RUN_TEST(test_a_reference_with_nobody_leased_declares_the_epoch);
  RUN_TEST(test_without_an_epoch_any_leased_beacon_sets_the_clock);
  RUN_TEST(test_the_nearest_route_to_the_reference_wins);
  RUN_TEST(test_a_parent_must_be_heard_directly_and_hold_a_lease);
  RUN_TEST(test_a_relayed_locked_reference_beats_a_free_running_one_in_earshot);
  RUN_TEST(test_a_route_longer_than_the_cap_is_not_believed);
  RUN_TEST(test_a_convoy_strung_out_converges_on_one_reference);
  RUN_TEST(test_the_lowest_node_number_is_the_reference);
  RUN_TEST(test_a_locked_car_outranks_a_lower_numbered_free_running_one);
  RUN_TEST(test_the_lowest_locked_car_wins_among_several);
  RUN_TEST(test_our_own_lock_counts_too);
  RUN_TEST(test_nobody_locked_falls_back_to_the_lowest_number);
  RUN_TEST(test_sync_backs_out_the_reference_slot);
  RUN_TEST(test_a_car_that_is_not_fit_does_not_keep_time_whatever_its_number);
  RUN_TEST(test_a_belief_carries_its_references_fitness_until_we_hear_it_ourselves);
  RUN_TEST(test_fitness_needs_half_the_links_solid_both_ways_and_a_third_to_keep_it);
  RUN_TEST(test_a_car_that_hears_the_ride_poorly_listens_twice_as_long);
  RUN_TEST(test_a_second_lost_lease_backs_off_before_claiming_again);
  RUN_TEST(test_silence_from_neighbours_heard_poorly_does_not_drown_a_lease);
  RUN_TEST(test_a_lease_no_map_has_shown_for_a_while_is_given_up);
  RUN_TEST(test_a_mixed_ride_puts_everyone_on_one_schedule);
  RUN_TEST(test_the_position_carries_the_lease_and_the_reference);
  RUN_TEST(test_a_version_1_frame_is_refused);
  RUN_TEST(test_sim_25_cars_power_on_together);
  RUN_TEST(test_sim_29_cars_fill_the_firmware_roster);
  RUN_TEST(test_sim_30_cars_with_one_more_roster_seat);
  RUN_TEST(test_sim_30_cars_overflow_a_28_seat_roster);
  RUN_TEST(test_sim_seeds_are_not_lucky);
  RUN_TEST(test_sim_joins_do_not_move_anyone);
  RUN_TEST(test_sim_drops_and_arrivals_on_a_full_ride);
  RUN_TEST(test_sim_reboots_rejoin_without_stealing);
  RUN_TEST(test_sim_a_car_back_from_out_of_range_does_not_steal);
  RUN_TEST(test_sim_losing_the_reference_changes_no_lease);
  RUN_TEST(test_sim_two_groups_merge_into_one_schedule);
  RUN_TEST(test_sim_a_strung_out_convoy_keeps_its_slots_apart);
  RUN_TEST(test_sim_some_cars_on_gps);
  RUN_TEST(test_a_car_alone_in_its_row_gets_the_other_three);
  RUN_TEST(test_two_in_a_row_each_get_the_block_opposite);
  RUN_TEST(test_three_in_a_row_leave_the_last_block_to_the_middle_one);
  RUN_TEST(test_a_neighbours_slot_map_counts_as_a_lease_in_the_row);
  RUN_TEST(test_extras_stop_while_a_car_waits_for_a_slot);
  RUN_TEST(test_no_extras_alone_or_unleased);
  RUN_TEST(test_an_extra_slot_opens_only_in_its_own_window);
  RUN_TEST(test_extras_are_disjoint_and_fill_every_occupied_row);
  RUN_TEST(test_a_drowned_lease_listens_again_before_reclaiming);
  RUN_TEST(test_a_drowned_lease_says_so);
  RUN_TEST(test_an_unheard_lease_says_so);
  RUN_TEST(test_an_outranked_lease_says_so);
  RUN_TEST(test_a_lease_left_alone_says_so);
  RUN_TEST(test_a_loss_formats_for_serial_and_the_phone);
  RUN_TEST(test_the_extra_flag_round_trips);
  RUN_TEST(test_sim_rates_3_cars);
  RUN_TEST(test_sim_rates_10_cars);
  RUN_TEST(test_sim_rates_25_cars);
  RUN_TEST(test_sim_rates_32_cars);
  RUN_TEST(test_sim_joins_take_a_slot_while_extras_run);
  RUN_TEST(test_sim_extras_cut_the_age_of_a_1hz_phone_fix);
  RUN_TEST(test_sim_a_weak_radio_keeps_to_a_free_slot_and_off_the_clock);
  RUN_TEST(test_sim_the_bench_three_radios_with_the_weak_one_lowest);
  RUN_TEST(test_sim_the_reference_holds_while_link_quality_jitters);
  return UNITY_END();
}
