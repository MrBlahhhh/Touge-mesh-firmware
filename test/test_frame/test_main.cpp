// Host tests. Nothing here touches Arduino, which is the point: the wire
// format and the key derivation are the two things that must agree between a
// phone, a board and every other board, so they are the two things that get
// checked without a board in the loop.
//
// The ride vectors were generated from the same inputs Invite.kt uses. If one
// of them fails after a change here, the change broke compatibility with every
// phone already running, not just the test.

#include <unity.h>
#include <string.h>
#include <stdio.h>
#include "frame.h"
#include "ride.h"
#include "sha256.h"
#include "mesh.h"
#include "schedule.h"
#include "rideclock.h"
#include "hmac.h"
#include "hop.h"

using namespace touge;

static void hex(const uint8_t* d, size_t n, char* out) {
  for (size_t i = 0; i < n; i++) sprintf(out + i * 2, "%02x", d[i]);
  out[n * 2] = 0;
}

static void checkSha(const char* in, size_t len, const char* expect) {
  uint8_t d[SHA256_LEN];
  char got[SHA256_LEN * 2 + 1];
  sha256((const uint8_t*)in, len, d);
  hex(d, SHA256_LEN, got);
  TEST_ASSERT_EQUAL_STRING(expect, got);
}

// The padding boundaries are where a hand-written SHA-256 goes wrong, so each
// one either side of 56 and 120 bytes is pinned rather than sampled.
void test_sha256_vectors() {
  checkSha("", 0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  checkSha("abc", 3, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

  struct Case { int n; const char* want; };
  Case cases[] = {
      {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
      {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
      {57, "f13b2d724659eb3bf47f2dd6af1accc87b81f09f59f2b75e5c0bed6589dfe8c6"},
      {63, "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34"},
      {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
      {119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
      {120, "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c"},
  };
  char buf[128];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    memset(buf, 'a', (size_t)cases[i].n);
    checkSha(buf, (size_t)cases[i].n, cases[i].want);
  }
}

void test_ride_matches_the_app() {
  Ride r;
  TEST_ASSERT_TRUE(deriveRide("abcdefghjkmnpqrs", r));
  TEST_ASSERT_EQUAL_STRING("tg-54a4c5", r.channelName);

  char psk[SHA256_LEN * 2 + 1];
  hex(r.psk, PSK_LEN, psk);
  TEST_ASSERT_EQUAL_STRING("233b6bf75aeffdaf3d362850cbe9ce4b48791eff24ceb7e21056fa2e9054b2d5", psk);
}

void test_fast_net_derives_from_the_psk() {
  Ride r;
  TEST_ASSERT_TRUE(deriveRide("abcdefghjkmnpqrs", r));

  FastNet f;
  TEST_ASSERT_TRUE(deriveFast(r.psk, PSK_LEN, f));

  char k[SHA256_LEN * 2 + 1];
  hex(f.key, PSK_LEN, k);
  TEST_ASSERT_EQUAL_STRING("e2d1ca0ab8500b22494be1bd73410c2d6ecd703a5dcf53f43efdb3e6072e80a7", k);
  TEST_ASSERT_EQUAL_UINT8(1, f.wifiChannel);
  TEST_ASSERT_EQUAL_UINT8(0x6e, f.chanByte);
}

void test_fast_key_is_not_the_channel_psk() {
  // Two radios sharing one key with two different nonce schemes is one
  // bookkeeping slip away from a repeated counter, so the ESP-NOW key is
  // derived from the channel PSK rather than being it.
  Ride r;
  TEST_ASSERT_TRUE(deriveRide("abcdefghjkmnpqrs", r));
  FastNet f;
  TEST_ASSERT_TRUE(deriveFast(r.psk, PSK_LEN, f));
  TEST_ASSERT_NOT_EQUAL(0, memcmp(f.key, r.psk, PSK_LEN));
}

void test_fast_net_changes_with_the_psk() {
  // A one-bit change in the PSK has to move the key, and must not leave two
  // rides sharing a key because only the channel number happened to differ.
  Ride a, b;
  TEST_ASSERT_TRUE(deriveRide("abcdefghjkmnpqrs", a));
  TEST_ASSERT_TRUE(deriveRide("abcdefghjkmnpqrt", b));
  FastNet fa, fb;
  deriveFast(a.psk, PSK_LEN, fa);
  deriveFast(b.psk, PSK_LEN, fb);
  TEST_ASSERT_NOT_EQUAL(0, memcmp(fa.key, fb.key, PSK_LEN));
}

void test_fast_net_rejects_null() {
  FastNet f;
  TEST_ASSERT_FALSE(deriveFast(NULL, PSK_LEN, f));
}

void test_ride_rejects_short_keys() {
  Ride r;
  TEST_ASSERT_FALSE(deriveRide("", r));
  TEST_ASSERT_FALSE(deriveRide("short", r));
  TEST_ASSERT_FALSE(deriveRide(NULL, r));
  // Exactly at the floor is a real key.
  TEST_ASSERT_TRUE(deriveRide("abcdefgh", r));
}

void test_wifi_channel_is_always_legal() {
  // Channels 12 to 14 are not legal in the US and a board that picks one goes
  // silently deaf with no error anywhere, so the derivation is checked across
  // many keys rather than trusted on the strength of one.
  char key[] = "aaaaaaaakey00000";
  for (int i = 0; i < 200; i++) {
    key[3] = (char)('a' + (i % 26));
    key[5] = (char)('a' + (i / 26));
    Ride r;
    TEST_ASSERT_TRUE(deriveRide(key, r));
    FastNet f;
    TEST_ASSERT_TRUE(deriveFast(r.psk, PSK_LEN, f));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(1, f.wifiChannel);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(11, f.wifiChannel);
  }
}

void test_frame_round_trip() {
  const uint8_t body[] = {1, 2, 3, 4, 5};
  Frame f;
  f.type = FRAME_POSITION;
  f.src = 0xDEADBEEF;
  f.id = 0x89ABCDEF;
  f.hops = 3;
  f.chan = 0x54;
  f.payload = body;
  f.len = sizeof(body);

  uint8_t wire[FRAME_MAX];
  size_t n = encodeFrame(f, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(FRAME_HEADER + sizeof(body), n);

  Frame got;
  TEST_ASSERT_TRUE(decodeFrame(wire, n, got));
  TEST_ASSERT_EQUAL_UINT8(FRAME_POSITION, got.type);
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, got.src);
  TEST_ASSERT_EQUAL_UINT32(0x89ABCDEF, got.id);
  TEST_ASSERT_EQUAL_UINT8(3, got.hops);
  TEST_ASSERT_EQUAL_UINT8(0x54, got.chan);
  TEST_ASSERT_EQUAL_UINT16(sizeof(body), got.len);
  TEST_ASSERT_EQUAL_MEMORY(body, got.payload, sizeof(body));
}

void test_frame_rejects_junk() {
  uint8_t wire[FRAME_MAX];
  Frame f;
  f.type = FRAME_TEXT;
  f.len = 0;
  size_t n = encodeFrame(f, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(FRAME_HEADER, n);

  Frame got;
  TEST_ASSERT_FALSE(decodeFrame(NULL, n, got));
  TEST_ASSERT_FALSE(decodeFrame(wire, FRAME_HEADER - 1, got));

  uint8_t bad[FRAME_MAX];
  memcpy(bad, wire, n);
  bad[0] = 'X';
  TEST_ASSERT_FALSE(decodeFrame(bad, n, got));

  memcpy(bad, wire, n);
  bad[1] = (uint8_t)((9 << 4) | FRAME_TEXT); // a version from the future
  TEST_ASSERT_FALSE(decodeFrame(bad, n, got));

  // A length field that disagrees with what actually arrived. Left unchecked
  // this hands the caller a payload pointer past the end of the buffer.
  memcpy(bad, wire, n);
  bad[12] = 0x7F;
  bad[13] = 0xFF;
  TEST_ASSERT_FALSE(decodeFrame(bad, n, got));
}

void test_frame_refuses_to_overflow() {
  const uint8_t body[8] = {0};
  Frame f;
  f.type = FRAME_POSITION;
  f.payload = body;
  f.len = sizeof(body);

  uint8_t small[FRAME_HEADER + 4];
  TEST_ASSERT_EQUAL_UINT32(0, encodeFrame(f, small, sizeof(small)));

  // A type wider than its nibble would corrupt the version field, and every
  // other node would read the result as a stray packet.
  uint8_t wire[FRAME_MAX];
  f.type = 0x20;
  TEST_ASSERT_EQUAL_UINT32(0, encodeFrame(f, wire, sizeof(wire)));

  // A payload claiming more than a frame can carry.
  f.type = FRAME_POSITION;
  f.len = FRAME_MAX_PAYLOAD + 1;
  TEST_ASSERT_EQUAL_UINT32(0, encodeFrame(f, wire, sizeof(wire)));
}

void test_position_round_trip() {
  Position p;
  p.lat = 374419983;   // 37.4419983
  p.lon = -1221419420; // -122.1419420, negative to catch a sign bug
  p.headingDeg = 184;
  p.speedMph = 62;
  p.batteryPct = 77;
  p.hasFix = true;
  p.phoneAttached = false;
  p.clockLocked = true;
  p.slot = 6;
  strcpy(p.name, "mattmoto");

  uint8_t buf[64];
  size_t n = encodePosition(p, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT32(POSITION_MIN + 8, n);

  Position got;
  TEST_ASSERT_TRUE(decodePosition(buf, n, got));
  TEST_ASSERT_EQUAL_INT32(p.lat, got.lat);
  TEST_ASSERT_EQUAL_INT32(p.lon, got.lon);
  TEST_ASSERT_EQUAL_UINT16(184, got.headingDeg);
  TEST_ASSERT_EQUAL_UINT8(62, got.speedMph);
  TEST_ASSERT_EQUAL_UINT8(77, got.batteryPct);
  TEST_ASSERT_TRUE(got.hasFix);
  TEST_ASSERT_FALSE(got.phoneAttached);
  // Flags in the low nibble of one byte, slot in the high one. A packing slip
  // here would have cars quietly reading each other's slot as a flag.
  TEST_ASSERT_TRUE(got.clockLocked);
  TEST_ASSERT_EQUAL_UINT8(6, got.slot);
  TEST_ASSERT_EQUAL_STRING("mattmoto", got.name);
}

void test_position_heading_quantises_to_two_degrees() {
  // Two-degree steps are deliberate, so an odd heading coming back even is
  // correct rather than a rounding bug. What must not happen is a wrap: 359
  // has to stay just short of north, not land the long way round.
  struct Case { uint16_t in; uint16_t want; };
  Case cases[] = {
      {0, 0}, {1, 0}, {2, 2}, {45, 44}, {180, 180}, {359, 358}, {360, 0}, {361, 0},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    Position p;
    p.headingDeg = cases[i].in;
    uint8_t buf[32];
    size_t n = encodePosition(p, buf, sizeof(buf));
    Position got;
    TEST_ASSERT_TRUE(decodePosition(buf, n, got));
    TEST_ASSERT_EQUAL_UINT16(cases[i].want, got.headingDeg);
  }
}

void test_position_truncates_a_long_name() {
  // A newer build may send a longer name than this one can hold. Dropping the
  // position would take the car off the map; clipping the name does not.
  uint8_t buf[64];
  memset(buf, 0, sizeof(buf));
  memset(buf + POSITION_MIN, 'x', 40);

  Position got;
  TEST_ASSERT_TRUE(decodePosition(buf, POSITION_MIN + 40, got));
  TEST_ASSERT_EQUAL_UINT32(sizeof(got.name) - 1, strlen(got.name));
}

void test_position_rejects_a_short_payload() {
  uint8_t buf[POSITION_MIN];
  memset(buf, 0, sizeof(buf));
  Position got;
  TEST_ASSERT_FALSE(decodePosition(buf, POSITION_MIN - 1, got));
  TEST_ASSERT_FALSE(decodePosition(NULL, POSITION_MIN, got));
  TEST_ASSERT_TRUE(decodePosition(buf, POSITION_MIN, got));
}

// ---- Flood suppression and the roster --------------------------------------

void test_dedupe_forwards_a_packet_once() {
  Mesh m;
  m.reset();
  TEST_ASSERT_TRUE(m.firstSight(1, 100, 0));
  TEST_ASSERT_FALSE(m.firstSight(1, 100, 10));
  TEST_ASSERT_FALSE(m.firstSight(1, 100, 1000));
  // A different packet from the same node, and the same id from a different
  // node, are both new. Keying on only one of the two would silence a car.
  TEST_ASSERT_TRUE(m.firstSight(1, 101, 10));
  TEST_ASSERT_TRUE(m.firstSight(2, 100, 10));
}

void test_dedupe_forgets_after_the_window() {
  Mesh m;
  m.reset();
  TEST_ASSERT_TRUE(m.firstSight(1, 100, 0));
  TEST_ASSERT_FALSE(m.firstSight(1, 100, SEEN_TTL_MS));
  // Past the window the entry is treated as empty, so a rider who dropped out
  // and came back is not silenced by a stale record of their old traffic.
  TEST_ASSERT_TRUE(m.firstSight(1, 100, SEEN_TTL_MS + 1));
}

void test_dedupe_survives_more_traffic_than_it_has_slots() {
  Mesh m;
  m.reset();
  // Fill every slot, then keep going. The table evicts rather than refusing,
  // so the most recent traffic is always covered.
  for (uint32_t i = 0; i < SEEN_SLOTS * 2; i++) TEST_ASSERT_TRUE(m.firstSight(1, i, 0));
  TEST_ASSERT_FALSE(m.firstSight(1, SEEN_SLOTS * 2 - 1, 0));
}

static Position posNamed(const char* name) {
  Position p;
  p.lat = 374419983;
  p.lon = -1221419420;
  p.hasFix = true;
  if (name) strncpy(p.name, name, sizeof(p.name) - 1);
  return p;
}

void test_roster_updates_in_place() {
  Mesh m;
  m.reset();
  TEST_ASSERT_NOT_NULL(m.note(7, posNamed("jackie"), HEARD_FAST, -40, 0, 1000, 11));
  TEST_ASSERT_EQUAL_UINT32(1, m.count());
  TEST_ASSERT_NOT_NULL(m.note(7, posNamed("jackie"), HEARD_LORA, -110, 2, 2000, 11));
  TEST_ASSERT_EQUAL_UINT32(1, m.count());

  const Rider* r = m.find(7);
  TEST_ASSERT_NOT_NULL(r);
  TEST_ASSERT_EQUAL_UINT8(HEARD_LORA, r->via);
  TEST_ASSERT_EQUAL_UINT8(2, r->hopsAway);
  TEST_ASSERT_EQUAL_INT16(-110, r->rssi);
}

void test_roster_keeps_a_name_between_name_pings() {
  // The name rides along only now and then, because forty characters on every
  // ping is pure airtime. An unnamed ping must not blank the roster entry.
  Mesh m;
  m.reset();
  m.note(7, posNamed("jackie"), HEARD_FAST, -40, 0, 1000, 11);
  m.note(7, posNamed(nullptr), HEARD_FAST, -41, 0, 2000, 11);
  TEST_ASSERT_EQUAL_STRING("jackie", m.find(7)->pos.name);
}

void test_roster_will_not_bump_a_car_you_are_driving_behind() {
  Mesh m;
  m.reset();
  for (uint32_t i = 0; i < MAX_RIDERS; i++)
    TEST_ASSERT_NOT_NULL(m.note(i + 1, posNamed("x"), HEARD_FAST, -40, 0, 1000, 11));
  TEST_ASSERT_EQUAL_UINT32(MAX_RIDERS, m.count());

  // Everyone is current, so a newcomer is turned away rather than evicting
  // someone whose position is still live on the screen.
  TEST_ASSERT_NULL(m.note(99, posNamed("late"), HEARD_FAST, -40, 0, 1000, 11));

  // Once a seat has gone quiet, the newcomer takes it.
  uint32_t later = 1000 + RIDER_STALE_MS + 1;
  m.note(2, posNamed("x"), HEARD_FAST, -40, 0, later, 11); // keep this one fresh
  TEST_ASSERT_NOT_NULL(m.note(99, posNamed("late"), HEARD_FAST, -40, 0, later, 11));
  TEST_ASSERT_NOT_NULL(m.find(99));
  TEST_ASSERT_NOT_NULL(m.find(2));
}

void test_roster_drops_only_after_a_very_long_silence() {
  Mesh m;
  m.reset();
  m.note(7, posNamed("jackie"), HEARD_LORA, -110, 3, 1000, 11);

  // Stale is not gone. A car that vanishes off the screen every time it dips
  // behind a ridge is worse than one that says it was last seen 90 seconds ago.
  m.age(1000 + RIDER_STALE_MS + 1);
  TEST_ASSERT_NOT_NULL(m.find(7));

  m.age(1000 + RIDER_DROP_MS + 1);
  TEST_ASSERT_NULL(m.find(7));
}

// ---- Forward jitter and suppression ----------------------------------------

static void deferOne(Mesh &m, uint32_t src, uint32_t id, uint32_t dueMs) {
  uint8_t wire[FRAME_HEADER];
  Frame f;
  f.type = FRAME_POSITION;
  f.src = src;
  f.id = id;
  size_t n = encodeFrame(f, wire, sizeof(wire));
  TEST_ASSERT_TRUE(m.defer(wire, n, src, id, dueMs));
}

void test_copies_counts_every_arrival() {
  Mesh m;
  m.reset();
  TEST_ASSERT_EQUAL_UINT8(0, m.copies(1, 100, 0));
  m.firstSight(1, 100, 0);
  TEST_ASSERT_EQUAL_UINT8(1, m.copies(1, 100, 0));
  m.firstSight(1, 100, 1);
  m.firstSight(1, 100, 2);
  TEST_ASSERT_EQUAL_UINT8(3, m.copies(1, 100, 2));
  // Gone once the window passes, along with the rest of the entry.
  TEST_ASSERT_EQUAL_UINT8(0, m.copies(1, 100, SEEN_TTL_MS + 3));
}

void test_the_weakest_hearer_forwards_first() {
  // The car that heard the frame most faintly is the one furthest out, and so
  // the one whose forward reaches somewhere the original did not. It goes
  // first; everyone nearer hears that forward while still holding their own
  // copy and drops it. Before this the order was random, which meant the most
  // useful relay was as likely to be suppressed as to be the one that ran.
  const uint32_t spread = 90;
  const uint32_t far = forwardDelayMs(FORWARD_FAR_DBM, spread, 0);
  const uint32_t mid = forwardDelayMs(-70, spread, 0);
  const uint32_t near = forwardDelayMs(FORWARD_NEAR_DBM, spread, 0);

  TEST_ASSERT_EQUAL_UINT32(0, far);
  TEST_ASSERT_EQUAL_UINT32(spread, near);
  TEST_ASSERT_TRUE(mid > far);
  TEST_ASSERT_TRUE(mid < near);
}

void test_signal_beyond_the_ends_of_the_range_is_clamped() {
  const uint32_t spread = 90;
  // Nothing sorts below the far end or above the near one, and in particular
  // nothing produces a delay past the window it was given.
  TEST_ASSERT_EQUAL_UINT32(forwardDelayMs(FORWARD_FAR_DBM, spread, 0),
                           forwardDelayMs(-120, spread, 0));
  TEST_ASSERT_EQUAL_UINT32(forwardDelayMs(FORWARD_NEAR_DBM, spread, 0),
                           forwardDelayMs(-10, spread, 0));
}

void test_a_board_that_cannot_measure_signal_waits_longest() {
  // Some cores hand the callback no RSSI at all and it arrives as zero, which
  // as a signal reading means "extremely close". That is the right way round
  // to be wrong: a board that cannot measure how far away a sender is should
  // not be the one elected to relay for it.
  const uint32_t spread = 90;
  TEST_ASSERT_EQUAL_UINT32(forwardDelayMs(FORWARD_NEAR_DBM, spread, 0),
                           forwardDelayMs(0, spread, 0));
}

void test_two_cars_at_the_same_distance_do_not_transmit_together() {
  // Identical signal, so nothing about the ordering separates them. A few
  // milliseconds of noise does.
  const uint32_t spread = 90;
  bool differed = false;
  for (uint32_t t = 0; t < FORWARD_TIE_MS; t++)
    if (forwardDelayMs(-70, spread, t) != forwardDelayMs(-70, spread, 0)) differed = true;
  TEST_ASSERT_TRUE(differed);
  // And the noise never grows into the next car's place in the order.
  TEST_ASSERT_TRUE(forwardDelayMs(-70, spread, FORWARD_TIE_MS - 1) <
                   forwardDelayMs(FORWARD_NEAR_DBM, spread, 0));
}

void test_the_forwarding_window_widens_with_the_neighbourhood() {
  // The failure was a density one: a window that gives three cars time to
  // suppress each other gives nine cars no time at all.
  TEST_ASSERT_EQUAL_UINT32(FORWARD_JITTER_MS, forwardSpreadMs(1));
  TEST_ASSERT_EQUAL_UINT32(FORWARD_JITTER_MS, forwardSpreadMs(SUPPRESS_AFTER));
  TEST_ASSERT_TRUE(forwardSpreadMs(9) > forwardSpreadMs(3));
  TEST_ASSERT_TRUE(forwardSpreadMs(MAX_RIDERS) >= forwardSpreadMs(9));
}

void test_the_forwarding_window_is_bounded() {
  // A forward that arrives after the next beacon is worth nothing, so the
  // window stops widening well before a cycle.
  TEST_ASSERT_EQUAL_UINT32(FORWARD_JITTER_MAX_MS, forwardSpreadMs(MAX_RIDERS));
  TEST_ASSERT_TRUE(forwardSpreadMs(MAX_RIDERS) <= FORWARD_JITTER_MAX_MS);
  // Including the tie-break noise on top of the widest window.
  TEST_ASSERT_TRUE(forwardDelayMs(FORWARD_NEAR_DBM, forwardSpreadMs(MAX_RIDERS),
                                  FORWARD_TIE_MS - 1) < 250);
}

void test_an_empty_neighbourhood_still_waits() {
  // Nobody heard yet is not the same as nobody there. A board that has just
  // come up has an empty roster and must not treat that as permission to
  // transmit the instant a frame lands.
  TEST_ASSERT_EQUAL_UINT32(FORWARD_JITTER_MS, forwardSpreadMs(0));
}

void test_a_position_reaches_the_phone_once_per_interval() {
  // The lane carries four positions a second per car, and each one became its
  // own packet on a BLE link that manages about thirty a second in total. At
  // twenty-eight cars that is a hundred and twelve, so the radio's queue filled
  // and it dropped the newest of everything - positions, status and voice.
  Mesh m;
  Position p{};
  m.note(7, p, HEARD_FAST, -60, 0, 1000, 1);

  // First sight of a car always goes: there is nothing to throttle against.
  TEST_ASSERT_TRUE(m.phoneDue(7, 1000, 1000));
  // The next three beacons of that second do not.
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, 1250));
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, 1500));
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, 1999));
  // And the next second does.
  TEST_ASSERT_TRUE(m.phoneDue(7, 1000, 2000));
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, 2250));
}

void test_each_car_is_throttled_on_its_own_clock() {
  // Twenty-eight cars at one a second is twenty-eight packets a second, not
  // one: the budget is per car, so the roster does not starve behind whichever
  // car happened to be heard first.
  Mesh m;
  Position p{};
  m.note(7, p, HEARD_FAST, -60, 0, 1000, 1);
  m.note(9, p, HEARD_FAST, -60, 0, 1000, 1);

  TEST_ASSERT_TRUE(m.phoneDue(7, 1000, 1000));
  TEST_ASSERT_TRUE(m.phoneDue(9, 1000, 1000));
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, 1500));
  TEST_ASSERT_FALSE(m.phoneDue(9, 1000, 1500));
}

void test_a_car_not_on_the_roster_is_not_withheld() {
  // Nothing to throttle against, and withholding a position because we have
  // nowhere to record having sent it would be the wrong way round.
  Mesh m;
  TEST_ASSERT_TRUE(m.phoneDue(12345, 1000, 5000));
}

void test_the_phone_throttle_survives_the_millis_wrap() {
  Mesh m;
  Position p{};
  m.note(7, p, HEARD_FAST, -60, 0, 0xFFFFFF00, 1);
  TEST_ASSERT_TRUE(m.phoneDue(7, 1000, 0xFFFFFF00));
  TEST_ASSERT_FALSE(m.phoneDue(7, 1000, (uint32_t)(0xFFFFFF00 + 500)));
  TEST_ASSERT_TRUE(m.phoneDue(7, 1000, (uint32_t)(0xFFFFFF00 + 1000)));
}

void test_a_forward_waits_for_its_jitter() {
  Mesh m;
  m.reset();
  m.firstSight(1, 100, 0);
  deferOne(m, 1, 100, 10);

  Forward out;
  TEST_ASSERT_FALSE(m.nextDue(0, out));
  TEST_ASSERT_FALSE(m.nextDue(9, out));
  TEST_ASSERT_TRUE(m.nextDue(10, out));
  TEST_ASSERT_EQUAL_UINT32(1, out.src);
  TEST_ASSERT_EQUAL_UINT32(100, out.id);
  // Handed out once. A slot that stayed armed would rebroadcast on every pass.
  TEST_ASSERT_FALSE(m.nextDue(11, out));
}

void test_a_forward_overtaken_by_neighbours_is_dropped() {
  // The whole point of the jitter: while this one waited, other cars
  // rebroadcast the same packet. Everyone in earshot has it, so transmitting
  // now would be pure interference.
  Mesh m;
  m.reset();
  m.firstSight(1, 100, 0);
  deferOne(m, 1, 100, 10);

  for (uint8_t i = 1; i < SUPPRESS_AFTER; i++) m.firstSight(1, 100, 5);
  TEST_ASSERT_EQUAL_UINT8(SUPPRESS_AFTER, m.copies(1, 100, 5));

  Forward out;
  TEST_ASSERT_FALSE(m.nextDue(10, out));
  TEST_ASSERT_EQUAL_UINT32(1, m.suppressed());
}

void test_a_forward_nobody_else_made_still_goes() {
  Mesh m;
  m.reset();
  m.firstSight(1, 100, 0);
  deferOne(m, 1, 100, 10);
  // One short of the threshold: somebody still needs to hear this.
  for (uint8_t i = 2; i < SUPPRESS_AFTER; i++) m.firstSight(1, 100, 5);

  Forward out;
  TEST_ASSERT_TRUE(m.nextDue(10, out));
  TEST_ASSERT_EQUAL_UINT32(0, m.suppressed());
}

void test_forward_queue_drops_rather_than_delaying_what_is_waiting() {
  Mesh m;
  m.reset();
  uint8_t wire[FRAME_HEADER];
  Frame f;
  f.type = FRAME_POSITION;
  size_t n = encodeFrame(f, wire, sizeof(wire));

  for (size_t i = 0; i < FORWARD_SLOTS; i++)
    TEST_ASSERT_TRUE(m.defer(wire, n, 1, (uint32_t)i, 10));
  // Full. A late forward is worth less than the ones already queued, and on a
  // ride this busy somebody else is forwarding anyway.
  TEST_ASSERT_FALSE(m.defer(wire, n, 1, 999, 10));
}

void test_a_forward_scheduled_across_the_millis_wrap_still_fires() {
  // millis() wraps every 49 days. Comparing the wrong way round here would
  // park a frame in the queue until the next wrap came round.
  Mesh m;
  m.reset();
  m.firstSight(1, 100, 0xFFFFFFF0);
  deferOne(m, 1, 100, 0xFFFFFFFA);

  Forward out;
  TEST_ASSERT_FALSE(m.nextDue(0xFFFFFFF5, out));
  TEST_ASSERT_TRUE(m.nextDue(0x00000004, out));
}

void test_packet_ids_never_restart_at_zero() {
  // The id is half the AES-CTR nonce. Counting from zero after a reboot would
  // replay every nonce this node has already used under the same channel key.
  Mesh m;
  m.reset();
  m.seedIds(50000);
  TEST_ASSERT_EQUAL_UINT32(50001, m.nextId());
  TEST_ASSERT_EQUAL_UINT32(50002, m.nextId());
  TEST_ASSERT_EQUAL_UINT32(50002, m.lastId());
}

// ---- Distance -------------------------------------------------------------

void test_distance_is_close_enough_to_be_a_gate() {
  // One ten-thousandth of a degree of latitude is 11.13 m anywhere on earth.
  TEST_ASSERT_UINT32_WITHIN(1, 11, distanceM(374419983, -1221419420, 374420983, -1221419420));
  TEST_ASSERT_EQUAL_UINT32(0, distanceM(374419983, -1221419420, 374419983, -1221419420));

  // A degree of longitude shortens towards the poles. At 60 degrees north it
  // is worth half what it is at the equator, and a gate that missed this
  // would fire at twice the distance up there.
  uint32_t atEquator = distanceM(0, 0, 0, 10000);
  uint32_t atSixty = distanceM(600000000, 0, 600000000, 10000);
  TEST_ASSERT_UINT32_WITHIN(atEquator / 40, atEquator / 2, atSixty);
}

void test_distance_does_not_overflow_on_a_full_span_of_longitude() {
  // Longitude runs to +/-1.8e9 at this scale, so a difference across the whole
  // range is 3.6e9 and overflows the signed 32-bit type the operands are
  // stored in. Wrapped, it comes out around 7,700 km instead of 40,000, so a
  // loose assertion here would pass with the bug still in place.
  //
  // Flat projection, so this is the long way round the equator rather than the
  // 200 m step across the date line. That limit is fine for a gate measuring
  // how far one car moved between beacons, and wrong enough to be obvious if
  // anyone ever tries to use it for something else.
  uint32_t d = distanceM(0, -1799999000, 0, 1799999000);
  TEST_ASSERT_UINT32_WITHIN(400000, 40074000, d);
}

// ---- Slots ----------------------------------------------------------------

// A car nobody has heard claim a slot yet.
static void addRider(Rider *r, size_t i, uint32_t id) {
  r[i].id = id;
  r[i].used = true;
  r[i].atMs = 0;
  r[i].pos.slot = SLOT_NONE;
}

static void addRiderOn(Rider *r, size_t i, uint32_t id, uint8_t slot) {
  addRider(r, i, id);
  r[i].pos.slot = slot;
}

// A car that holds a slot and has an opinion about who is keeping time.
static void addRiderBelieving(Rider *r, size_t i, uint32_t id, uint8_t slot, uint32_t refId,
                              uint8_t refHops, bool refLocked = false) {
  addRiderOn(r, i, id, slot);
  r[i].pos.refId = refId;
  r[i].pos.refHops = refHops;
  r[i].pos.refLocked = refLocked;
}

void test_we_take_a_slot_nobody_else_holds() {
  // Not the lowest free one. The search enters the ring at our own node
  // number, so that boards booting together with nothing on the roster do not
  // all find slot zero free and all take it. Which free slot it is does not
  // matter; that it is free does.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 500, 0);
  addRiderOn(riders, 1, 100, 2);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_TRUE(s.slot() < MAX_SLOTS);
  TEST_ASSERT_NOT_EQUAL(0, s.slot());
  TEST_ASSERT_NOT_EQUAL(2, s.slot());
  TEST_ASSERT_EQUAL_UINT8(3, s.known());
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  TEST_ASSERT_FALSE(s.weAreReference());
}

void test_a_claimed_slot_survives_a_car_joining() {
  // The whole reason for claiming rather than deriving from rank. Under rank,
  // a car arriving with a lower node number pushed everyone above it onto a
  // new slot at the same instant, so gaining a car cost the convoy a cycle of
  // collisions. A held slot does not move for an arrival.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 500, 0);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  uint8_t mine = s.slot();
  TEST_ASSERT_TRUE(s.claimed());

  // A car with a lower number than ours turns up, on a slot of its own.
  addRiderOn(riders, 1, 100, (uint8_t)((mine + 1) % MAX_SLOTS));
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());

  // And one leaves.
  riders[0].used = false;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());
}

void test_the_lower_node_number_wins_a_contested_slot() {
  // Two cars out of earshot of each other pick the same slot, then meet. Both
  // apply the same rule to the same facts, so exactly one of them moves.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 3); // lower than us, and on our slot

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(3, s.slot()); // 100 holds it and outranks us

  // And the other way round: we hold one, and a lower number arrives on it.
  Schedule t;
  Rider alone[MAX_RIDERS] = {};
  addRiderOn(alone, 0, 900, 0);
  t.rebuild(300, false, alone, MAX_RIDERS, 0);
  const uint8_t ours = t.slot();
  TEST_ASSERT_TRUE(t.claimed());

  addRiderOn(alone, 1, 100, ours); // lower number, takes our slot from under us
  t.rebuild(300, false, alone, MAX_RIDERS, 0);
  TEST_ASSERT_NOT_EQUAL(ours, t.slot());
  TEST_ASSERT_TRUE(t.claimed());
}

void test_a_higher_node_number_does_not_take_our_slot() {
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  const uint8_t mine = s.slot();
  TEST_ASSERT_TRUE(s.claimed());

  // 900 sorts above us and lands on our slot, so it is the one that has to
  // move, not us.
  addRiderOn(riders, 1, 900, mine);
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT8(mine, s.slot());
}

void test_every_car_lands_on_a_slot_of_its_own() {
  // Four cars, each seeing the other three with the slots they have claimed.
  // Nobody hands them out and nobody ends up doubled up.
  const uint32_t ids[4] = {900, 100, 500, 300};
  const uint8_t held[4] = {3, 0, 2, 1};

  for (int me = 0; me < 4; me++) {
    Rider roster[MAX_RIDERS] = {};
    size_t n = 0;
    for (int other = 0; other < 4; other++)
      if (other != me) addRiderOn(roster, n++, ids[other], held[other]);

    Schedule s;
    s.rebuild(ids[me], false, roster, MAX_RIDERS, 0);
    // Not which slot it lands on - that depends where the search enters the
    // ring - but that it is nobody else's.
    TEST_ASSERT_TRUE(s.claimed());
    for (int other = 0; other < 4; other++)
      if (other != me) TEST_ASSERT_NOT_EQUAL(held[other], s.slot());
    TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  }
}

void test_a_reference_travels_past_the_cars_that_can_hear_it() {
  // We cannot hear 100 at all. The car in front of us can, and says so, which
  // is the whole mechanism: the claim walks the convoy rather than depending
  // on everybody being in range of one car.
  Rider riders[MAX_RIDERS] = {};
  addRiderBelieving(riders, 0, 300, 2, /*refId=*/100, /*refHops=*/1);

  Schedule s;
  s.rebuild(500, false, riders, MAX_RIDERS, 0);

  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  TEST_ASSERT_FALSE(s.weAreReference());
  // Two hops: 300 is one from the reference, and we are one from 300.
  TEST_ASSERT_EQUAL_UINT8(2, s.hopsToReference());
  // And the clock comes from 300, because a beacon from 100 is never coming.
  TEST_ASSERT_EQUAL_UINT32(300, s.parentId());
  TEST_ASSERT_EQUAL_UINT8(2, s.syncSlot());
}

void test_the_reference_is_no_hops_from_itself_and_syncs_to_nobody() {
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 500, 1);

  Schedule s;
  s.rebuild(100, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.weAreReference());
  TEST_ASSERT_EQUAL_UINT8(0, s.hopsToReference());
  TEST_ASSERT_EQUAL_UINT32(0, s.parentId());
  // Its own slot is what syncTo would subtract, so declaring an epoch does not
  // move its transmissions.
  TEST_ASSERT_EQUAL_UINT8(s.slot(), s.syncSlot());
}

void test_the_nearest_route_to_the_reference_wins() {
  // Two neighbours both know the way; the clock comes from the shorter route,
  // because every hop is another car's beacon timing to inherit.
  Rider riders[MAX_RIDERS] = {};
  addRiderBelieving(riders, 0, 300, 2, /*refId=*/100, /*refHops=*/3);
  addRiderBelieving(riders, 1, 400, 5, /*refId=*/100, /*refHops=*/1);

  Schedule s;
  s.rebuild(500, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
  TEST_ASSERT_EQUAL_UINT32(400, s.parentId());
  TEST_ASSERT_EQUAL_UINT8(5, s.syncSlot());
  TEST_ASSERT_EQUAL_UINT8(2, s.hopsToReference());
}

void test_a_relayed_locked_reference_beats_a_free_running_one_in_earshot() {
  // The rule is unchanged - locked beats unlocked, then lowest number - but it
  // now applies to cars we cannot hear. A GPS-disciplined car three back still
  // takes the job from the free-running car sitting next to us.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 200, 1); // free-running, right here, lower than us
  addRiderBelieving(riders, 1, 300, 2, /*refId=*/700, /*refHops=*/1, /*refLocked=*/true);

  Schedule s;
  s.rebuild(500, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
  TEST_ASSERT_TRUE(s.referenceLocked());
  TEST_ASSERT_EQUAL_UINT32(300, s.parentId());
}

void test_a_route_longer_than_the_cap_is_not_believed() {
  // Two cars naming each other as the way to a reference that has gone away
  // will count upward for ever. The cap turns that into a few wasted beacons.
  Rider riders[MAX_RIDERS] = {};
  addRiderBelieving(riders, 0, 300, 2, /*refId=*/100, /*refHops=*/MAX_REF_HOPS);

  Schedule s;
  s.rebuild(500, false, riders, MAX_RIDERS, 0);
  // 100 is not adopted, so the best thing we can actually see wins.
  TEST_ASSERT_EQUAL_UINT32(300, s.referenceId());
}

void test_a_convoy_strung_out_converges_on_one_reference() {
  // Four cars in a line, each able to hear only the ones beside it. The tail
  // has never heard the head and never will.
  //
  // This is the failure that made the per-observer election worth fixing: the
  // head elected itself, the tail elected the lowest car in the tail, and both
  // were free to decide the channel was bad and hop - taking half the ride
  // each. Neither half was ever "lost", because each could hear plenty of
  // cars, so neither went looking and nothing reconverged them.
  const uint32_t id[4] = {100, 200, 300, 400};
  Schedule s[4];
  Position belief[4];
  for (int i = 0; i < 4; i++) {
    belief[i] = Position{};
    belief[i].refHops = REF_UNREACHABLE;
  }

  // Six beacon rounds. The claim moves one car per round, so four would do.
  for (int round = 0; round < 6; round++) {
    for (int me = 0; me < 4; me++) {
      Rider roster[MAX_RIDERS] = {};
      size_t n = 0;
      for (int other = 0; other < 4; other++) {
        if (other != me - 1 && other != me + 1) continue; // out of range
        addRider(roster, n, id[other]);
        roster[n].pos = belief[other];
        n++;
      }
      s[me].rebuild(id[me], false, roster, MAX_RIDERS, 0);
    }
    // What each car puts on the air next time round.
    for (int me = 0; me < 4; me++) {
      belief[me].refId = s[me].referenceId();
      belief[me].refHops = s[me].hopsToReference();
      belief[me].refLocked = s[me].referenceLocked();
      belief[me].slot = s[me].slot();
    }
  }

  for (int me = 0; me < 4; me++) TEST_ASSERT_EQUAL_UINT32(100, s[me].referenceId());

  // Each one a hop further out, and each taking its clock from the car in
  // front rather than from a beacon it cannot receive.
  TEST_ASSERT_EQUAL_UINT8(0, s[0].hopsToReference());
  TEST_ASSERT_EQUAL_UINT8(1, s[1].hopsToReference());
  TEST_ASSERT_EQUAL_UINT8(2, s[2].hopsToReference());
  TEST_ASSERT_EQUAL_UINT8(3, s[3].hopsToReference());

  TEST_ASSERT_EQUAL_UINT32(0, s[0].parentId());
  TEST_ASSERT_EQUAL_UINT32(100, s[1].parentId());
  TEST_ASSERT_EQUAL_UINT32(200, s[2].parentId());
  TEST_ASSERT_EQUAL_UINT32(300, s[3].parentId());

  // Exactly one car thinks it is in charge.
  int anchors = 0;
  for (int me = 0; me < 4; me++)
    if (s[me].weAreReference()) anchors++;
  TEST_ASSERT_EQUAL_INT(1, anchors);
}

void test_the_position_carries_the_reference_across_the_wire() {
  Position p{};
  p.lat = 351102700;
  p.lon = -790018400;
  p.slot = 5;
  p.refId = 0xDEADBEEF;
  p.refHops = 3;
  p.refLocked = true;

  uint8_t wire[64];
  size_t n = encodePosition(p, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(POSITION_MIN, n);

  Position got{};
  TEST_ASSERT_TRUE(decodePosition(wire, n, got));
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, got.refId);
  TEST_ASSERT_EQUAL_UINT8(3, got.refHops);
  TEST_ASSERT_TRUE(got.refLocked);
  TEST_ASSERT_EQUAL_UINT8(5, got.slot);
  TEST_ASSERT_EQUAL_INT32(351102700, got.lat);
}

void test_the_lowest_node_number_is_the_reference() {
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 500, 1);
  addRiderOn(riders, 1, 900, 2);

  Schedule s;
  s.rebuild(100, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.weAreReference());
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(1, s.slot());
  TEST_ASSERT_NOT_EQUAL(2, s.slot());
}

void test_a_car_alone_does_not_wait_for_a_schedule() {
  // Nothing to collide with, and no reference beacon will ever arrive. Waiting
  // for a sync here would mean never transmitting at all.
  Schedule s;
  Rider none[MAX_RIDERS] = {};
  s.rebuild(300, false, none, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT8(1, s.known());
  TEST_ASSERT_TRUE(s.inSlot(0, 250));
  TEST_ASSERT_TRUE(s.inSlot(123, 250));
}

void test_slot_window_opens_once_per_cycle() {
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0); // the reference, on slot zero
  addRiderOn(riders, 1, 900, 2);

  const uint32_t cycle = 250;
  const uint32_t width = Schedule::slotWidthMs(cycle); // 250 / 9 = 27
  TEST_ASSERT_EQUAL_UINT32(27, width);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  const uint8_t mine = s.slot();
  TEST_ASSERT_NOT_EQUAL(0, mine); // 100 holds that, and it is the reference
  const uint32_t opens = (uint32_t)mine * width;
  // The reference advertises slot zero, so its beacon lands on the cycle start
  // with nothing to subtract.
  TEST_ASSERT_EQUAL_UINT8(0, s.syncSlot());

  s.syncTo(1000, cycle); // the reference's beacon landed here, so a cycle began
  TEST_ASSERT_TRUE(s.synced());

  // Our slot runs for one width, once, starting where our slot number puts it.
  TEST_ASSERT_FALSE(s.inSlot(1000, cycle));
  TEST_ASSERT_FALSE(s.inSlot(1000 + opens - 1, cycle));
  TEST_ASSERT_TRUE(s.inSlot(1000 + opens, cycle));
  TEST_ASSERT_TRUE(s.inSlot(1000 + opens + width - 1, cycle));
  TEST_ASSERT_FALSE(s.inSlot(1000 + opens + width, cycle));

  // And again a cycle later, without another sync.
  TEST_ASSERT_TRUE(s.inSlot(1000 + cycle + opens, cycle));
  TEST_ASSERT_FALSE(s.inSlot(1000 + cycle, cycle));
}

void test_slots_keep_running_across_the_millis_wrap() {
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);
  addRiderOn(riders, 1, 900, 2);

  const uint32_t cycle = 250;
  const uint32_t width = Schedule::slotWidthMs(cycle);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  const uint32_t opens = (uint32_t)s.slot() * width;
  s.syncTo(0xFFFFFF00, cycle);
  // 0xFFFFFF00 + 250 wraps past zero. Unsigned subtraction carries the phase
  // through; signed would put the slot 49 days away.
  TEST_ASSERT_TRUE(s.inSlot((uint32_t)(0xFFFFFF00 + cycle + opens), cycle));
}

void test_more_cars_than_slots_doubles_up_rather_than_falling_off() {
  // Every slot taken and one more car. Doubling up costs those two a
  // collision; falling off the end of the cycle would cost the newcomer every
  // transmission it ever made.
  //
  // This used to hand out slots 0..MAX_RIDERS-1, so with nine slots and
  // twenty-eight riders everything past the ninth advertised a slot number no
  // slot has and was read as unclaimed. It filled nine slots by accident and
  // asserted only that a slot was claimed, which is true of every path through
  // this code including the ones that are wrong.
  Rider riders[MAX_RIDERS] = {};
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    addRiderOn(riders, i, (uint32_t)(i + 1), (uint8_t)(i % MAX_SLOTS));
  }

  Schedule s;
  s.rebuild(1000, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_TRUE(s.slot() < MAX_SLOTS);

  // Every slot was owned, so the claim fell through to the node number. Naming
  // the rule rather than the outcome: this is the line that says twenty-eight
  // cars share nine slots by arithmetic and nothing stops them landing on the
  // same one.
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(1000 % MAX_SLOTS), s.slot());

  // And it is a genuine collision. Somebody already advertises the slot we
  // just took, which is the cost being accepted here and the reason two-hop
  // colouring is the real answer.
  bool shared = false;
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    if (riders[i].used && riders[i].pos.slot == s.slot()) shared = true;
  }
  TEST_ASSERT_TRUE_MESSAGE(shared, "expected to double up on an occupied slot");
}

void test_a_full_roster_crowds_every_slot() {
  // Twenty-eight cars into nine slots. The previous version of this built an
  // array, counted the array, and asserted the arithmetic it had just done -
  // it never called rebuild at all, so it could not have caught a scheduler
  // that handed every car the same slot.
  //
  // This runs the real claim for every car against the same roster and counts
  // what the scheduler actually produced.
  Rider riders[MAX_RIDERS] = {};
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    addRiderOn(riders, i, (uint32_t)(i + 1), (uint8_t)(i % MAX_SLOTS));
  }

  uint8_t perSlot[MAX_SLOTS] = {};
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    Schedule s;
    s.rebuild(riders[i].id, false, riders, MAX_RIDERS, 0);
    TEST_ASSERT_TRUE(s.claimed());
    TEST_ASSERT_TRUE(s.slot() < MAX_SLOTS);
    perSlot[s.slot()]++;
  }

  uint8_t deepest = 0;
  uint8_t empty = 0;
  for (uint8_t sIdx = 0; sIdx < MAX_SLOTS; sIdx++) {
    if (perSlot[sIdx] > deepest) deepest = perSlot[sIdx];
    if (perSlot[sIdx] == 0) empty++;
  }

  // Every slot gets used and more than one car lands on some of them. Both
  // halves matter: all in one slot would be a broken scheduler, and one car
  // per slot would mean the roster no longer exceeds the slots and this test
  // has stopped testing anything.
  TEST_ASSERT_EQUAL_UINT8(0, empty);
  TEST_ASSERT_TRUE_MESSAGE(deepest > 1, "a full roster must share slots; that is the known cost");
}

void test_a_duplicate_node_number_does_not_corrupt_the_claim() {
  // Two boards with the same node number is a real failure and nothing here
  // can fix it, but it must not also corrupt the head count or hand our own
  // slot away to what is really us.
  // The duplicate is put on the very slot we would otherwise pick, so that a
  // claim from our own id blocking us would show up as us moving off it.
  const uint8_t would = (uint8_t)(300 % MAX_SLOTS);
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);
  addRiderOn(riders, 1, 300, would); // our own number, claiming a slot

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT8(2, s.known());
  TEST_ASSERT_TRUE(s.claimed());
  // It reads as free, because the only claim on it is from our own id.
  TEST_ASSERT_EQUAL_UINT8(would, s.slot());
}

void test_a_departed_car_does_not_hold_its_slot_forever() {
  // The roster deliberately keeps a car for ten minutes so that one over a
  // ridge does not vanish off the map. Its slot is a different question with a
  // different answer, and both were being read off the same roster: five cars
  // leaving a ride held five of nine slots for ten minutes while the cars
  // still on the road crowded into what was left.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);
  riders[0].atMs = 0;

  // Heard a moment ago. The claim stands and we go somewhere else.
  Schedule s;
  s.rebuild(900, false, riders, MAX_RIDERS, 1000);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_NOT_EQUAL(0, s.slot());

  // Quiet for longer than a dropout, still on the roster. 900 % 9 is 0, so it
  // enters the ring exactly where the departed car was sitting: if the claim
  // still counted, this would have to move.
  Schedule t;
  t.rebuild(900, false, riders, MAX_RIDERS, SLOT_LAPSE_MS + 1);
  TEST_ASSERT_TRUE(t.claimed());
  TEST_ASSERT_EQUAL_UINT8(0, t.slot());
}

void test_boards_starting_together_do_not_all_take_slot_zero() {
  // Every board boots with a roster it has not filled in yet. Scanning from
  // zero meant every one of them found slot zero free and took it, so a car
  // park of cars switched on together transmitted in a single slot and then
  // spent nine beacon rounds unpicking it by node number, colliding on the low
  // slots throughout. Entering the ring at the node number spreads that first
  // guess before anybody has heard anybody.
  //
  // These nine ids land on nine different slots, which is the mechanism doing
  // exactly what it is for. Real node numbers are hash-like and will collide
  // sometimes; the lowest-number rule is what settles those.
  const uint32_t ids[9] = {11, 22, 33, 44, 55, 66, 77, 88, 99};
  bool taken[MAX_SLOTS] = {};
  int distinct = 0;
  for (int i = 0; i < 9; i++) {
    Rider nobody[MAX_RIDERS] = {};
    Schedule s;
    s.rebuild(ids[i], false, nobody, MAX_RIDERS, 0);
    TEST_ASSERT_TRUE(s.claimed());
    if (!taken[s.slot()]) {
      taken[s.slot()] = true;
      distinct++;
    }
  }
  TEST_ASSERT_EQUAL_INT(9, distinct);
}

void test_an_unclaimed_car_free_runs_until_it_has_been_heard() {
  // Chicken and egg: a car has to be heard before anyone will leave it a slot,
  // and it has to transmit to be heard. So before its first beacon it ignores
  // the schedule entirely rather than waiting for a turn nobody has given it.
  Schedule s;
  TEST_ASSERT_FALSE(s.claimed());
  TEST_ASSERT_TRUE(s.inSlotAtPhase(0, 250));
  TEST_ASSERT_TRUE(s.inSlotAtPhase(200, 250));
}

// ---- The GPS-disciplined clock ---------------------------------------------

static const uint64_t SEC = 1000000ULL;

// A pulse has to prove itself before it is believed, so a test that wants a
// locked clock has to feed a run of well-spaced edges ending at `lastAt`.
static void lockClock(RideClock &c, uint64_t lastAt) {
  for (uint32_t i = PULSE_LOCK_RUN; i > 0; i--)
    c.onPulse(lastAt - (uint64_t)i * PULSE_PERIOD_US);
  c.onPulse(lastAt);
}

void test_one_pulse_is_not_enough_to_be_believed() {
  // The failure being guarded against: a board whose variant declares a PPS
  // pin with no receiver fitted, which is a bare V4 without the expansion kit.
  // The pin floats, noise fires the handler, and an unguarded clock would have
  // that board announce itself as the one keeping time for the whole ride.
  RideClock c;
  c.reset();
  c.onPulse(10 * SEC);
  TEST_ASSERT_FALSE(c.locked(10 * SEC));

  c.onPulse(11 * SEC);
  TEST_ASSERT_FALSE(c.locked(11 * SEC));
  c.onPulse(12 * SEC);
  TEST_ASSERT_FALSE(c.locked(12 * SEC));

  // Fourth edge, third good gap.
  c.onPulse(13 * SEC);
  TEST_ASSERT_TRUE(c.locked(13 * SEC));
}

void test_edges_at_the_wrong_spacing_never_lock() {
  RideClock c;
  c.reset();
  // Noise: plausible rate, wrong period. Ten of them and it is still not a
  // clock, because none of them landed when a pulse was due.
  for (int i = 1; i <= 10; i++) c.onPulse((uint64_t)i * 700000ULL);
  TEST_ASSERT_FALSE(c.locked(7 * SEC));

  // Just outside the tolerance is still wrong.
  RideClock d;
  d.reset();
  for (int i = 1; i <= 10; i++)
    d.onPulse((uint64_t)i * (PULSE_PERIOD_US + PULSE_TOLERANCE_US + 1000));
  TEST_ASSERT_FALSE(d.locked(11 * SEC));
}

void test_a_dropped_pulse_does_not_cost_the_lock() {
  // Two seconds is still a real edge, just a second late. Losing the lock over
  // it would mean a receiver that stutters once never keeps one.
  RideClock c;
  c.reset();
  lockClock(c, 10 * SEC);
  TEST_ASSERT_TRUE(c.locked(10 * SEC));

  c.onPulse(12 * SEC);
  TEST_ASSERT_TRUE(c.locked(12 * SEC));

  // Three is a receiver in trouble, and the run starts over.
  c.onPulse(15 * SEC);
  TEST_ASSERT_FALSE(c.locked(15 * SEC));
}

void test_the_cycle_must_divide_a_second() {
  // Everything else here rests on this. A cycle that does not divide a second
  // would put the cycle boundary somewhere new after every pulse.
  TEST_ASSERT_TRUE(cycleDividesSecond(250));
  TEST_ASSERT_TRUE(cycleDividesSecond(200));
  TEST_ASSERT_TRUE(cycleDividesSecond(1000));
  TEST_ASSERT_FALSE(cycleDividesSecond(300));
  TEST_ASSERT_FALSE(cycleDividesSecond(0));
  TEST_ASSERT_FALSE(cycleDividesSecond(1500));
}

void test_no_phase_before_the_first_pulse() {
  RideClock c;
  c.reset();
  uint32_t phase = 99;
  TEST_ASSERT_FALSE(c.phaseMs(SEC, 250, phase));
  TEST_ASSERT_FALSE(c.locked(SEC));
}

void test_phase_counts_from_the_pulse() {
  RideClock c;
  c.reset();
  lockClock(c, 10 * SEC);

  uint32_t phase = 0;
  TEST_ASSERT_TRUE(c.phaseMs(10 * SEC, 250, phase));
  TEST_ASSERT_EQUAL_UINT32(0, phase);

  TEST_ASSERT_TRUE(c.phaseMs(10 * SEC + 30000, 250, phase)); // 30 ms in
  TEST_ASSERT_EQUAL_UINT32(30, phase);

  // 260 ms after the pulse is 10 ms into the second cycle of that second.
  TEST_ASSERT_TRUE(c.phaseMs(10 * SEC + 260000, 250, phase));
  TEST_ASSERT_EQUAL_UINT32(10, phase);
}

void test_a_missed_pulse_does_not_move_the_cycle() {
  // The reason the cycle has to divide a second: a whole second later is a
  // whole number of cycles later, so one dropped pulse costs nothing.
  RideClock c;
  c.reset();
  lockClock(c, 10 * SEC);

  uint32_t phase = 0;
  TEST_ASSERT_TRUE(c.phaseMs(10 * SEC + 1200000, 250, phase)); // 1.2 s later
  TEST_ASSERT_EQUAL_UINT32(200, phase);
}

void test_a_stopped_clock_reports_no_phase() {
  // A receiver that loses its fix holds the last edge forever. Trusting it
  // would look locked while drifting a second further out every second, which
  // is worse than having no clock at all.
  RideClock c;
  c.reset();
  lockClock(c, 10 * SEC);

  uint32_t phase = 0;
  TEST_ASSERT_TRUE(c.phaseMs(10 * SEC + PULSE_STALE_US, 250, phase));
  TEST_ASSERT_FALSE(c.phaseMs(10 * SEC + PULSE_STALE_US + 1, 250, phase));
  TEST_ASSERT_FALSE(c.locked(10 * SEC + 10 * SEC));

  // A fresh run of pulses brings it back. One would not: a single edge after
  // a long silence is exactly what noise looks like.
  lockClock(c, 20 * SEC);
  TEST_ASSERT_TRUE(c.phaseMs(20 * SEC, 250, phase));
}

void test_clock_refuses_a_cycle_it_cannot_keep() {
  RideClock c;
  c.reset();
  lockClock(c, 10 * SEC);
  uint32_t phase = 0;
  TEST_ASSERT_FALSE(c.phaseMs(10 * SEC, 300, phase));
}

void test_two_cars_on_gps_agree_without_ever_meeting() {
  // The point of the whole exercise. Two cars that have never exchanged a
  // packet, whose local microsecond counters are nowhere near each other,
  // still put the cycle boundary in the same place because both pulses mark
  // the same UTC second.
  RideClock a, b;
  a.reset();
  b.reset();

  const uint64_t aBoot = 5 * SEC;       // a has been up five seconds
  const uint64_t bBoot = 98765 * SEC;   // b for rather longer
  lockClock(a, aBoot);
  lockClock(b, bBoot);

  for (uint32_t offset = 0; offset < 250; offset += 7) {
    uint32_t pa = 0, pb = 1;
    TEST_ASSERT_TRUE(a.phaseMs(aBoot + offset * 1000, 250, pa));
    TEST_ASSERT_TRUE(b.phaseMs(bBoot + offset * 1000, 250, pb));
    TEST_ASSERT_EQUAL_UINT32(pa, pb);
  }
}

void test_gps_slots_need_no_reference_car() {
  // Without GPS a car that is not the reference and has heard no reference
  // beacon free-runs, because it has nothing better to do. With GPS it keeps
  // to its slot regardless, which is what removes the dependence on one car
  // staying in range.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);
  addRiderOn(riders, 1, 900, 2);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_TRUE(s.claimed());
  TEST_ASSERT_FALSE(s.synced());
  TEST_ASSERT_TRUE(s.inSlot(0, 250)); // no epoch, so it free-runs

  const uint32_t width = Schedule::slotWidthMs(250);
  const uint32_t opens = (uint32_t)s.slot() * width;
  TEST_ASSERT_NOT_EQUAL(0, s.slot());
  TEST_ASSERT_FALSE(s.inSlotAtPhase(0, 250));
  TEST_ASSERT_TRUE(s.inSlotAtPhase(opens, 250));
  TEST_ASSERT_FALSE(s.inSlotAtPhase(opens + width, 250));
}

// ---- The mixed ride --------------------------------------------------------
//
// Some cars have a GNSS receiver of their own and some only have a phone.
// Those two populations run off different clocks, and the only thing keeping
// them on one cycle is that the reference car is always one of the locked ones.

static void addLockedRider(Rider *r, size_t i, uint32_t id) {
  addRider(r, i, id);
  r[i].pos.clockLocked = true;
}

void test_a_locked_car_outranks_a_lower_numbered_free_running_one() {
  // The bug this exists to catch: with the reference picked purely by node
  // number, an unlocked car at 100 would take the job, free-run on its own
  // timebase, and every phone-only car would sync to a cycle that the
  // GPS-locked cars know nothing about. They would then collide every time
  // rather than occasionally, which is worse than having no schedule at all.
  Rider riders[MAX_RIDERS] = {};
  addRider(riders, 0, 100);       // lowest, but free-running
  addLockedRider(riders, 1, 700); // locked

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
}

void test_the_lowest_locked_car_wins_among_several() {
  Rider riders[MAX_RIDERS] = {};
  addRider(riders, 0, 100);
  addLockedRider(riders, 1, 900);
  addLockedRider(riders, 2, 500);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(500, s.referenceId());
}

void test_our_own_lock_counts_too() {
  Rider riders[MAX_RIDERS] = {};
  addRider(riders, 0, 100);
  addRider(riders, 1, 900);

  Schedule s;
  s.rebuild(300, true, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(300, s.referenceId());
  TEST_ASSERT_TRUE(s.weAreReference());
}

void test_nobody_locked_falls_back_to_the_lowest_number() {
  Rider riders[MAX_RIDERS] = {};
  addRider(riders, 0, 100);
  addRider(riders, 1, 900);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(100, s.referenceId());
}

void test_sync_backs_out_the_reference_slot() {
  // The consequence of letting a locked car outrank a lower-numbered one: the
  // reference no longer holds slot zero, so its beacon marks its own slot
  // rather than the start of the cycle. Not subtracting that would put every
  // phone-only car a slot or two ahead of everyone else.
  Rider riders[MAX_RIDERS] = {};
  addRiderOn(riders, 0, 100, 0);           // free-running, on slot zero
  addLockedRider(riders, 1, 700);
  riders[1].pos.slot = 2;                  // locked, so it is the reference

  const uint32_t cycle = 250;
  const uint32_t width = Schedule::slotWidthMs(cycle);

  Schedule s;
  s.rebuild(300, false, riders, MAX_RIDERS, 0); // slots 0 and 2 taken
  TEST_ASSERT_EQUAL_UINT32(700, s.referenceId());
  // Read off the reference's own beacon, not derived from its rank.
  TEST_ASSERT_EQUAL_UINT8(2, s.syncSlot());
  TEST_ASSERT_TRUE(s.claimed());
  const uint8_t mine = s.slot();
  TEST_ASSERT_NOT_EQUAL(0, mine);
  TEST_ASSERT_NOT_EQUAL(2, mine);

  // Its beacon lands at t=1000, two slots into the cycle, so the cycle began
  // two slot widths earlier and our own slot is `mine` widths after that.
  s.syncTo(1000, cycle);
  const uint32_t began = 1000 - width * 2;
  TEST_ASSERT_TRUE(s.inSlot(began + (uint32_t)mine * width, cycle));
  TEST_ASSERT_FALSE(s.inSlot(1000, cycle));
  TEST_ASSERT_FALSE(s.inSlot(began, cycle));
}

void test_a_mixed_ride_puts_everyone_on_one_cycle() {
  // End to end. One car has GPS, one does not, and they must end up agreeing
  // on where the cycle boundary is despite getting there different ways.
  const uint32_t cycle = 250;
  const uint32_t width = Schedule::slotWidthMs(cycle);

  // The GPS car, id 700. Its pulse landed at a known microsecond.
  RideClock clock;
  clock.reset();
  const uint64_t pulse = 4242 * SEC;
  lockClock(clock, pulse);

  Rider gpsRoster[MAX_RIDERS] = {};
  addRiderOn(gpsRoster, 0, 300, 0); // the phone-only car already holds slot 0
  Schedule gps;
  gps.rebuild(700, true, gpsRoster, MAX_RIDERS, 0);
  // Locked, so it keeps time even though 300 is the lower number.
  TEST_ASSERT_TRUE(gps.weAreReference());
  TEST_ASSERT_TRUE(gps.claimed());
  const uint8_t gpsSlot = gps.slot();
  TEST_ASSERT_NOT_EQUAL(0, gpsSlot); // 300 holds that

  // Find the instant its slot opens, which is when its beacon goes out.
  uint64_t sendAt = 0;
  for (uint32_t ms = 0; ms < cycle; ms++) {
    uint32_t phase = 0;
    TEST_ASSERT_TRUE(clock.phaseMs(pulse + (uint64_t)ms * 1000, cycle, phase));
    if (gps.inSlotAtPhase(phase, cycle)) {
      sendAt = pulse + (uint64_t)ms * 1000;
      break;
    }
  }
  TEST_ASSERT_EQUAL_UINT64(pulse + (uint64_t)gpsSlot * width * 1000, sendAt);

  // The phone-only car, id 300, hears that beacon and syncs to it. Its own
  // millis() has nothing to do with the other car's microsecond counter.
  const uint32_t heardAtMs = 55555;
  Rider phoneRoster[MAX_RIDERS] = {};
  addLockedRider(phoneRoster, 0, 700);
  phoneRoster[0].pos.slot = gpsSlot; // as advertised in the beacon it just heard
  Schedule phone;
  phone.rebuild(300, false, phoneRoster, MAX_RIDERS, 0);
  TEST_ASSERT_EQUAL_UINT32(700, phone.referenceId());
  TEST_ASSERT_EQUAL_UINT8(gpsSlot, phone.syncSlot());
  TEST_ASSERT_TRUE(phone.claimed());
  const uint8_t phoneSlot = phone.slot();
  TEST_ASSERT_NOT_EQUAL(gpsSlot, phoneSlot); // the two never share one
  phone.syncTo(heardAtMs, cycle);

  // Backing the reference's own slot out of the beacon gives the cycle start
  // on the phone car's millis(), and its slot follows from there.
  const uint32_t began = heardAtMs - (uint32_t)gpsSlot * width;
  TEST_ASSERT_TRUE(phone.inSlot(began + (uint32_t)phoneSlot * width, cycle));
  TEST_ASSERT_FALSE(phone.inSlot(heardAtMs, cycle));
}

// ---- Authentication --------------------------------------------------------

void test_hmac_matches_rfc_4231() {
  // Pinned against the published vectors rather than against itself. A tag
  // that is self-consistently wrong still rejects every forgery and still
  // accepts every genuine frame, so nothing else in this file would notice.
  uint8_t key[20];
  memset(key, 0x0b, sizeof(key));
  uint8_t out[SHA256_LEN];
  char got[SHA256_LEN * 2 + 1];

  hmacSha256(key, sizeof(key), (const uint8_t *)"Hi There", 8, out);
  hex(out, SHA256_LEN, got);
  TEST_ASSERT_EQUAL_STRING("b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7", got);

  const char *jefe = "Jefe";
  const char *q = "what do ya want for nothing?";
  hmacSha256((const uint8_t *)jefe, 4, (const uint8_t *)q, 28, out);
  hex(out, SHA256_LEN, got);
  TEST_ASSERT_EQUAL_STRING("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", got);
}

void test_hmac_handles_a_key_longer_than_a_block() {
  // Over 64 bytes the key is replaced by its own digest. Getting this wrong
  // only shows up with long keys, and ours are 32, so it is checked here
  // rather than discovered later.
  uint8_t key[131];
  memset(key, 0xaa, sizeof(key));
  const char *data = "Test Using Larger Than Block-Size Key - Hash Key First";
  uint8_t out[SHA256_LEN];
  char got[SHA256_LEN * 2 + 1];
  hmacSha256(key, sizeof(key), (const uint8_t *)data, 54, out);
  hex(out, SHA256_LEN, got);
  TEST_ASSERT_EQUAL_STRING("60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54", got);
}

void test_a_frame_tag_covers_what_names_the_frame() {
  // Everything the tag is supposed to bind has to change it. A field left out
  // is a field an attacker can rewrite on a frame that still verifies.
  const uint8_t key[PSK_LEN] = {1, 2, 3};
  const uint8_t cipher[4] = {9, 9, 9, 9};
  uint8_t base[TAG_LEN];
  frameTag(key, PSK_LEN, 100, 200, FRAME_POSITION, 0x54, cipher, sizeof(cipher), base);

  uint8_t other[TAG_LEN];
  frameTag(key, PSK_LEN, 101, 200, FRAME_POSITION, 0x54, cipher, sizeof(cipher), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // sender

  frameTag(key, PSK_LEN, 100, 201, FRAME_POSITION, 0x54, cipher, sizeof(cipher), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // packet id

  frameTag(key, PSK_LEN, 100, 200, FRAME_VOICE, 0x54, cipher, sizeof(cipher), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // type

  frameTag(key, PSK_LEN, 100, 200, FRAME_POSITION, 0x55, cipher, sizeof(cipher), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // channel

  const uint8_t flipped[4] = {9, 9, 8, 9};
  frameTag(key, PSK_LEN, 100, 200, FRAME_POSITION, 0x54, flipped, sizeof(flipped), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // ciphertext

  const uint8_t otherKey[PSK_LEN] = {1, 2, 4};
  frameTag(otherKey, PSK_LEN, 100, 200, FRAME_POSITION, 0x54, cipher, sizeof(cipher), other);
  TEST_ASSERT_FALSE(tagsMatch(base, other, TAG_LEN)); // another ride's key

  // And the same inputs give the same tag, or nothing would ever verify.
  frameTag(key, PSK_LEN, 100, 200, FRAME_POSITION, 0x54, cipher, sizeof(cipher), other);
  TEST_ASSERT_TRUE(tagsMatch(base, other, TAG_LEN));
}

void test_the_tag_does_not_cover_the_hop_count() {
  // Deliberate. Every forwarding node decrements the hop count, so a tag over
  // it would make a forwarded frame fail its own check at the next node and
  // multi-hop would silently stop working.
  //
  // frameTag has no hop parameter at all, which is the enforcement. This test
  // exists so that adding one is a decision somebody has to make on purpose.
  const uint8_t key[PSK_LEN] = {7};
  const uint8_t cipher[2] = {1, 2};
  uint8_t a[TAG_LEN], b[TAG_LEN];
  frameTag(key, PSK_LEN, 5, 6, FRAME_POSITION, 0x11, cipher, sizeof(cipher), a);
  frameTag(key, PSK_LEN, 5, 6, FRAME_POSITION, 0x11, cipher, sizeof(cipher), b);
  TEST_ASSERT_TRUE(tagsMatch(a, b, TAG_LEN));
}

void test_tags_match_compares_everything() {
  uint8_t a[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint8_t b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  TEST_ASSERT_TRUE(tagsMatch(a, b, 8));
  // A difference in the last byte must be caught as surely as one in the
  // first; an early return there is what leaks the tag a byte at a time.
  b[7] = 9;
  TEST_ASSERT_FALSE(tagsMatch(a, b, 8));
  b[7] = 8;
  b[0] = 9;
  TEST_ASSERT_FALSE(tagsMatch(a, b, 8));
  TEST_ASSERT_FALSE(tagsMatch(NULL, b, 8));
}

// ---- Hopping ---------------------------------------------------------------

void test_the_candidates_do_not_overlap_each_other() {
  // One, six and eleven are the only three channels in the band that do not
  // overlap. Hopping between adjacent ones would move the ride without moving
  // it out of the interference, which is the entire point of moving.
  TEST_ASSERT_EQUAL_UINT8(1, HOP_CHANNELS[0]);
  TEST_ASSERT_EQUAL_UINT8(6, HOP_CHANNELS[1]);
  TEST_ASSERT_EQUAL_UINT8(11, HOP_CHANNELS[2]);
  for (size_t i = 0; i < FAST_CHANNELS; i++) {
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(1, HOP_CHANNELS[i]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(11, HOP_CHANNELS[i]);
  }
}

void test_generation_comparison_survives_the_wrap() {
  // Six bits, so it wraps every 64 hops. A straight comparison would make
  // generation 0 look older than 63 forever, and the ride would stop following
  // hops the first time the counter went round.
  TEST_ASSERT_TRUE(hopNewer(2, 1));
  TEST_ASSERT_FALSE(hopNewer(1, 2));
  TEST_ASSERT_FALSE(hopNewer(1, 1));
  TEST_ASSERT_TRUE(hopNewer(0, 63));
  TEST_ASSERT_FALSE(hopNewer(63, 0));
  TEST_ASSERT_TRUE(hopNewer(5, 60));
  TEST_ASSERT_FALSE(hopNewer(60, 5));
}

void test_hop_packs_into_one_byte() {
  for (uint8_t idx = 0; idx < FAST_CHANNELS; idx++) {
    for (uint8_t gen = 0; gen < 64; gen++) {
      uint8_t gotIdx = 0xFF, gotGen = 0xFF;
      hopUnpack(hopPack(idx, gen), gotIdx, gotGen);
      TEST_ASSERT_EQUAL_UINT8(idx, gotIdx);
      TEST_ASSERT_EQUAL_UINT8(gen, gotGen);
    }
  }
}

void test_a_newer_belief_wins_and_an_older_one_is_ignored() {
  Hop h;
  h.begin(0);
  TEST_ASSERT_EQUAL_UINT8(1, h.channel());
  TEST_ASSERT_EQUAL_UINT8(0, h.generation());

  // A car on a later generation moves us.
  TEST_ASSERT_TRUE(h.observe(hopPack(2, 1)));
  TEST_ASSERT_EQUAL_UINT8(11, h.channel());
  TEST_ASSERT_EQUAL_UINT8(1, h.generation());

  // A car still on the old one does not drag us back.
  TEST_ASSERT_FALSE(h.observe(hopPack(0, 0)));
  TEST_ASSERT_EQUAL_UINT8(11, h.channel());

  // The same generation on a different channel is not newer either, so two
  // cars cannot fight over one generation number.
  TEST_ASSERT_FALSE(h.observe(hopPack(1, 1)));
  TEST_ASSERT_EQUAL_UINT8(11, h.channel());
}

void test_a_belief_naming_a_channel_that_does_not_exist_is_refused() {
  // Two bits carry four values and there are three channels. The tag should
  // stop a corrupted byte getting this far, but a bad index would index off
  // the end of the table.
  Hop h;
  h.begin(0);
  TEST_ASSERT_FALSE(h.observe(hopPack(3, 9)));
  TEST_ASSERT_EQUAL_UINT8(1, h.channel());
  TEST_ASSERT_EQUAL_UINT8(0, h.generation());
}

void test_advancing_moves_the_channel_and_the_generation() {
  Hop h;
  h.begin(0);
  uint8_t was = h.channel();
  h.advance();
  TEST_ASSERT_NOT_EQUAL(was, h.channel());
  TEST_ASSERT_EQUAL_UINT8(1, h.generation());

  // Round the houses and back, without ever naming a channel off the list.
  for (int i = 0; i < 20; i++) {
    h.advance();
    bool known = false;
    for (size_t c = 0; c < FAST_CHANNELS; c++)
      if (h.channel() == HOP_CHANNELS[c]) known = true;
    TEST_ASSERT_TRUE(known);
  }
}

void test_a_lost_car_visits_every_candidate() {
  // The floor under everything else. A car that missed a hop, or was switched
  // off during one, or joined the ride late, has three places to look and no
  // dependence on ever having heard the announcement.
  Hop h;
  h.begin(0);
  bool seen[FAST_CHANNELS] = {false, false, false};
  for (int i = 0; i < FAST_CHANNELS; i++) {
    uint8_t ch = h.scanNext();
    for (size_t c = 0; c < FAST_CHANNELS; c++)
      if (ch == HOP_CHANNELS[c]) seen[c] = true;
  }
  for (size_t c = 0; c < FAST_CHANNELS; c++) TEST_ASSERT_TRUE(seen[c]);
}

void test_searching_does_not_change_what_we_believe() {
  // A car hunting for the group must not announce wherever it happened to
  // stop. It takes the group's answer when it finds them; it does not impose
  // the one it guessed on the way.
  Hop h;
  h.begin(1);
  uint8_t gen = h.generation();
  uint8_t idx = h.index();
  h.scanNext();
  h.scanNext();
  TEST_ASSERT_EQUAL_UINT8(gen, h.generation());
  TEST_ASSERT_EQUAL_UINT8(idx, h.index());
}

void test_two_rides_do_not_have_to_start_together() {
  Hop a, b;
  a.begin(0);
  b.begin(1);
  TEST_ASSERT_NOT_EQUAL(a.channel(), b.channel());
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_sha256_vectors);
  RUN_TEST(test_ride_matches_the_app);
  RUN_TEST(test_ride_rejects_short_keys);
  RUN_TEST(test_fast_net_derives_from_the_psk);
  RUN_TEST(test_fast_key_is_not_the_channel_psk);
  RUN_TEST(test_fast_net_changes_with_the_psk);
  RUN_TEST(test_fast_net_rejects_null);
  RUN_TEST(test_wifi_channel_is_always_legal);
  RUN_TEST(test_frame_round_trip);
  RUN_TEST(test_frame_rejects_junk);
  RUN_TEST(test_frame_refuses_to_overflow);
  RUN_TEST(test_position_round_trip);
  RUN_TEST(test_position_heading_quantises_to_two_degrees);
  RUN_TEST(test_position_truncates_a_long_name);
  RUN_TEST(test_position_rejects_a_short_payload);
  RUN_TEST(test_dedupe_forwards_a_packet_once);
  RUN_TEST(test_dedupe_forgets_after_the_window);
  RUN_TEST(test_dedupe_survives_more_traffic_than_it_has_slots);
  RUN_TEST(test_roster_updates_in_place);
  RUN_TEST(test_roster_keeps_a_name_between_name_pings);
  RUN_TEST(test_roster_will_not_bump_a_car_you_are_driving_behind);
  RUN_TEST(test_roster_drops_only_after_a_very_long_silence);
  RUN_TEST(test_packet_ids_never_restart_at_zero);
  RUN_TEST(test_copies_counts_every_arrival);
  RUN_TEST(test_the_weakest_hearer_forwards_first);
  RUN_TEST(test_signal_beyond_the_ends_of_the_range_is_clamped);
  RUN_TEST(test_a_board_that_cannot_measure_signal_waits_longest);
  RUN_TEST(test_two_cars_at_the_same_distance_do_not_transmit_together);
  RUN_TEST(test_the_forwarding_window_widens_with_the_neighbourhood);
  RUN_TEST(test_the_forwarding_window_is_bounded);
  RUN_TEST(test_an_empty_neighbourhood_still_waits);
  RUN_TEST(test_a_position_reaches_the_phone_once_per_interval);
  RUN_TEST(test_each_car_is_throttled_on_its_own_clock);
  RUN_TEST(test_a_car_not_on_the_roster_is_not_withheld);
  RUN_TEST(test_the_phone_throttle_survives_the_millis_wrap);
  RUN_TEST(test_a_forward_waits_for_its_jitter);
  RUN_TEST(test_a_forward_overtaken_by_neighbours_is_dropped);
  RUN_TEST(test_a_forward_nobody_else_made_still_goes);
  RUN_TEST(test_forward_queue_drops_rather_than_delaying_what_is_waiting);
  RUN_TEST(test_a_forward_scheduled_across_the_millis_wrap_still_fires);
  RUN_TEST(test_distance_is_close_enough_to_be_a_gate);
  RUN_TEST(test_distance_does_not_overflow_on_a_full_span_of_longitude);
  RUN_TEST(test_we_take_a_slot_nobody_else_holds);
  RUN_TEST(test_a_claimed_slot_survives_a_car_joining);
  RUN_TEST(test_the_lower_node_number_wins_a_contested_slot);
  RUN_TEST(test_a_higher_node_number_does_not_take_our_slot);
  RUN_TEST(test_a_departed_car_does_not_hold_its_slot_forever);
  RUN_TEST(test_boards_starting_together_do_not_all_take_slot_zero);
  RUN_TEST(test_an_unclaimed_car_free_runs_until_it_has_been_heard);
  RUN_TEST(test_every_car_lands_on_a_slot_of_its_own);
  RUN_TEST(test_a_reference_travels_past_the_cars_that_can_hear_it);
  RUN_TEST(test_the_reference_is_no_hops_from_itself_and_syncs_to_nobody);
  RUN_TEST(test_the_nearest_route_to_the_reference_wins);
  RUN_TEST(test_a_relayed_locked_reference_beats_a_free_running_one_in_earshot);
  RUN_TEST(test_a_route_longer_than_the_cap_is_not_believed);
  RUN_TEST(test_a_convoy_strung_out_converges_on_one_reference);
  RUN_TEST(test_the_position_carries_the_reference_across_the_wire);
  RUN_TEST(test_the_lowest_node_number_is_the_reference);
  RUN_TEST(test_a_car_alone_does_not_wait_for_a_schedule);
  RUN_TEST(test_slot_window_opens_once_per_cycle);
  RUN_TEST(test_slots_keep_running_across_the_millis_wrap);
  RUN_TEST(test_more_cars_than_slots_doubles_up_rather_than_falling_off);
  RUN_TEST(test_a_full_roster_crowds_every_slot);
  RUN_TEST(test_a_duplicate_node_number_does_not_corrupt_the_claim);
  RUN_TEST(test_the_cycle_must_divide_a_second);
  RUN_TEST(test_no_phase_before_the_first_pulse);
  RUN_TEST(test_one_pulse_is_not_enough_to_be_believed);
  RUN_TEST(test_edges_at_the_wrong_spacing_never_lock);
  RUN_TEST(test_a_dropped_pulse_does_not_cost_the_lock);
  RUN_TEST(test_phase_counts_from_the_pulse);
  RUN_TEST(test_a_missed_pulse_does_not_move_the_cycle);
  RUN_TEST(test_a_stopped_clock_reports_no_phase);
  RUN_TEST(test_clock_refuses_a_cycle_it_cannot_keep);
  RUN_TEST(test_two_cars_on_gps_agree_without_ever_meeting);
  RUN_TEST(test_gps_slots_need_no_reference_car);
  RUN_TEST(test_a_locked_car_outranks_a_lower_numbered_free_running_one);
  RUN_TEST(test_the_lowest_locked_car_wins_among_several);
  RUN_TEST(test_our_own_lock_counts_too);
  RUN_TEST(test_nobody_locked_falls_back_to_the_lowest_number);
  RUN_TEST(test_sync_backs_out_the_reference_slot);
  RUN_TEST(test_a_mixed_ride_puts_everyone_on_one_cycle);
  RUN_TEST(test_hmac_matches_rfc_4231);
  RUN_TEST(test_hmac_handles_a_key_longer_than_a_block);
  RUN_TEST(test_a_frame_tag_covers_what_names_the_frame);
  RUN_TEST(test_the_tag_does_not_cover_the_hop_count);
  RUN_TEST(test_tags_match_compares_everything);
  RUN_TEST(test_the_candidates_do_not_overlap_each_other);
  RUN_TEST(test_generation_comparison_survives_the_wrap);
  RUN_TEST(test_hop_packs_into_one_byte);
  RUN_TEST(test_a_newer_belief_wins_and_an_older_one_is_ignored);
  RUN_TEST(test_a_belief_naming_a_channel_that_does_not_exist_is_refused);
  RUN_TEST(test_advancing_moves_the_channel_and_the_generation);
  RUN_TEST(test_a_lost_car_visits_every_candidate);
  RUN_TEST(test_searching_does_not_change_what_we_believe);
  RUN_TEST(test_two_rides_do_not_have_to_start_together);
  return UNITY_END();
}
