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
  TEST_ASSERT_EQUAL_UINT8(2, s.slot()); // lowest free, nobody ahead of us
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
  TEST_ASSERT_EQUAL_UINT8(2, s.slot());
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
  TEST_ASSERT_EQUAL_UINT8(2, t.slot());
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
      belief[me].slot = s[me].slot();
      belief[me].leaseGen = s[me].leaseGeneration();
      belief[me].schedGen = s[me].generation();
      belief[me].refId = s[me].referenceId();
      belief[me].refHops = s[me].hopsToReference();
      belief[me].refLocked = s[me].referenceLocked();
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

  uint8_t wire[64];
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
};

struct Delivery {
  uint32_t at;
  size_t to, src;
  uint32_t id;
  Position pos;
  uint8_t hopsAway;
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
};

struct World {
  std::vector<Car> cars;
  std::vector<std::vector<bool>> link;
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

  explicit World(size_t n, uint32_t seed) {
    rng = seed;
    cars.resize(n);
    link.assign(n, std::vector<bool>(n, true));
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

    if (d.hopsAway == 0 && c.sched.takesClockFrom(srcId) &&
        !(d.src == c.lastSyncSrc && d.id == c.lastSyncId)) {
      c.lastSyncSrc = d.src;
      c.lastSyncId = d.id;
      c.sched.syncTo(at, d.pos.slot, SYNC_BIAS_MS);
    }

    if (d.id <= c.seen[d.src]) return;
    c.seen[d.src] = d.id;
    note(c, srcId, d.pos, d.hopsAway, at);
    if (d.hopsAway == 0) c.sched.heardSlot(d.pos.slot, srcId, at);
    c.sched.rebuild(c.id, c.gps, c.roster, ROSTER, at);

    if (d.hopsAway < RELAY_HOPS)
      for (size_t k = 0; k < cars.size(); k++)
        if (link[d.to][k] && k != d.src)
          pending.push_back({now + RELAY_MS, k, d.src, d.id, d.pos, (uint8_t)(d.hopsAway + 1)});
  }

  void tick(size_t i) {
    Car& c = cars[i];
    const uint32_t at = local(c, now);
    if (!c.want) {
      if (!c.sentOnce || (int32_t)(at - c.nextBeacon) >= 0) {
        c.want = true;
        c.sched.rebuild(c.id, c.gps, c.roster, ROSTER, at);
        c.sched.drawSharedTurn(rand32());
      }
    }
    if (!c.want) return;
    if (c.sched.weAreReference()) c.sched.startEpoch(at);
    // A PPS edge on the true second, whatever the crystal thinks.
    const bool mine = c.gps ? c.sched.inSlotAtPhase(now % SCHEDULE_MS) : c.sched.inSlot(at);
    if (!mine) return;

    c.want = false;
    if (c.nextBeacon == 0) c.nextBeacon = at;
    c.nextBeacon = nextOnGrid(c.nextBeacon, 1000, at);
    c.sentOnce = true;
    c.sent++;

    Tx tx;
    tx.src = i;
    tx.id = ++c.frameId;
    tx.start = now;
    tx.end = now + AIR_MS;
    tx.pos.slot = c.sched.slot();
    tx.pos.leaseGen = c.sched.leaseGeneration();
    tx.pos.schedGen = c.sched.generation();
    c.sched.fillSlotMap(at, tx.pos.slotMap);
    tx.pos.clockLocked = c.gps;
    tx.pos.refId = c.sched.referenceId();
    tx.pos.refHops = c.sched.hopsToReference();
    tx.pos.refLocked = c.sched.referenceLocked();
    air.push_back(tx);
  }

  void endTransmissions() {
    for (size_t t = 0; t < air.size(); t++) {
      if (air[t].end != now) continue;
      const Tx tx = air[t];
      for (size_t j = 0; j < cars.size(); j++) {
        if (!cars[j].on || !link[tx.src][j]) continue;
        const Tx* clash = nullptr;
        for (const Tx& other : air) {
          if (&other == &air[t]) continue;
          const bool overlaps = other.start < tx.end && other.end > tx.start;
          if (overlaps && (other.src == j || link[j][other.src])) clash = &other;
        }
        if (clash == nullptr) {
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
    }
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

  w.mark();
  const uint32_t met = w.now;
  for (size_t a = 0; a < 13; a++)
    for (size_t b = 13; b < 25; b++) w.setLink(a, b, true);
  w.run(20000);
  TEST_ASSERT_TRUE(w.settled());
  TEST_ASSERT_EQUAL_UINT32(1, w.references());
  TEST_ASSERT_TRUE(w.lastUnsettled + 100 <= met + SETTLE_BOUND_MS);
  sim::report("merge", w, met);

  // And it stays merged.
  w.mark();
  w.run(30000);
  TEST_ASSERT_EQUAL_UINT32(0, w.leasedCollisions);
  for (const sim::Car& c : w.cars) TEST_ASSERT_EQUAL_UINT32(0, c.slotChanges);
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
  return UNITY_END();
}
