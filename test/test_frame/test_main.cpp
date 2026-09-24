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
#include "gnssfix.h"
#include "ownfix.h"

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
  p.slot = 29;
  p.leaseGen = 0xBEEF;
  p.schedGen = 0xF00D;
  p.fix.session = 0xA5C3;
  p.fix.seq = 0x01020304;
  p.fix.fixSec = 1790000000;
  p.fix.fixMs = 987;
  strcpy(p.name, "mattmoto");

  uint8_t buf[POSITION_MIN + 16];
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
  TEST_ASSERT_TRUE(got.clockLocked);
  // Above fifteen, which the version-1 nibble could not carry.
  TEST_ASSERT_EQUAL_UINT8(29, got.slot);
  TEST_ASSERT_EQUAL_UINT16(0xBEEF, got.leaseGen);
  TEST_ASSERT_EQUAL_UINT16(0xF00D, got.schedGen);
  TEST_ASSERT_EQUAL_UINT16(0xA5C3, got.fix.session);
  TEST_ASSERT_EQUAL_UINT32(0x01020304, got.fix.seq);
  TEST_ASSERT_EQUAL_UINT32(1790000000, got.fix.fixSec);
  TEST_ASSERT_EQUAL_UINT16(987, got.fix.fixMs);
  TEST_ASSERT_EQUAL_STRING("mattmoto", got.name);

  // Where the identity sits, big-endian, between the slot map and the name.
  const uint8_t want[FIX_ID_LEN] = {0xA5, 0xC3, 0x01, 0x02, 0x03, 0x04, 0x6A, 0xB1, 0x3B, 0x80, 0x03, 0xDB};
  TEST_ASSERT_EQUAL_MEMORY(want, buf + 23 + SLOT_MAP_LEN, FIX_ID_LEN);
  TEST_ASSERT_EQUAL_UINT8('m', buf[POSITION_MIN]);
}

// ---- Fix identity (SCALE-PLAN 5a) -------------------------------------------
//
// The app's FixId.rank is the same rule, and PositionBatchTest pins the same
// cases.

static FixId fixId(uint16_t session, uint32_t seq, uint32_t fixSec, uint16_t fixMs = 0) {
  FixId f;
  f.session = session;
  f.seq = seq;
  f.fixSec = fixSec;
  f.fixMs = fixMs;
  return f;
}

void test_within_a_session_the_sequence_decides() {
  const FixId held = fixId(7, 100, 1790000100);
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(7, 101, 1790000101), held));
  // The same fix again: its 2.4 GHz copy and its LoRa copy are one position.
  TEST_ASSERT_EQUAL(FixRank::SAME, rankFix(fixId(7, 100, 1790000100), held));
  // A late copy of an older fix never replaces the newer one.
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankFix(fixId(7, 99, 1790000099), held));
  // The sequence wins over a clock stepped back mid-session.
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(7, 101, 1790000090), held));
}

void test_a_long_drive_does_not_wrap_into_looking_older() {
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(7, 2, 1790000002), fixId(7, 0xFFFFFFFF, 1790000001)));
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankFix(fixId(7, 0xFFFFFFFF, 1790000001), fixId(7, 2, 1790000002)));
}

void test_a_rebooted_radio_is_a_new_session() {
  // A reboot starts the sequence at 1 under a new session, measured later.
  const FixId before = fixId(7, 5000, 1790000100, 250);
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(9, 1, 1790000130, 10), before));
  // A late copy from the old boot after the switch is older.
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankFix(before, fixId(9, 1, 1790000130, 10)));
  // A reboot that drew the same session: the sequence went back, the time on.
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(7, 1, 1790000130), before));
  // With no fix time either side, the newcomer is taken.
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(9, 1, 0), before));
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankFix(fixId(7, 1, 0), before));
}

void test_the_same_millisecond_is_not_newer() {
  TEST_ASSERT_EQUAL(FixRank::OLDER, rankFix(fixId(9, 1, 1790000100, 250), fixId(7, 5, 1790000100, 250)));
  TEST_ASSERT_EQUAL(FixRank::NEWER, rankFix(fixId(9, 1, 1790000100, 251), fixId(7, 5, 1790000100, 250)));
}

static Fix reading(int32_t lat, uint32_t fixSec, uint16_t fixMs = 0) {
  Fix f;
  f.lat = lat;
  f.lon = -825000000;
  f.trackE5 = 9000000;
  f.speedKmh = 88;
  f.fixSec = fixSec;
  f.fixMs = fixMs;
  f.external = true;
  return f;
}

void test_each_new_fix_takes_the_next_sequence() {
  OwnFix own;
  TEST_ASSERT_FALSE(own.has());
  own.fromPhone(reading(355000000, 1790000000, 100), 10000, 0x12345678);
  TEST_ASSERT_TRUE(own.has());
  const FixId first = own.id();
  TEST_ASSERT_EQUAL_UINT32(1, first.seq);
  TEST_ASSERT_EQUAL_UINT16(0x1234 ^ 0x5678, first.session);
  TEST_ASSERT_EQUAL_UINT32(1790000000, first.fixSec);
  TEST_ASSERT_EQUAL_UINT16(100, first.fixMs);

  // Written again: the same fix, the same name.
  own.fromPhone(reading(355000000, 1790000000, 100), 11000, 0x99999999);
  TEST_ASSERT_EQUAL_UINT32(1, own.id().seq);
  // A parked car's next fix: same place, measured later.
  own.fromPhone(reading(355000000, 1790000001, 100), 12000, 0x99999999);
  TEST_ASSERT_EQUAL_UINT32(2, own.id().seq);
  // The session is drawn once per boot, not per fix.
  TEST_ASSERT_EQUAL_UINT16(first.session, own.id().session);
}

void test_a_write_with_no_coordinates_changes_nothing() {
  OwnFix own;
  Fix none;
  none.fixSec = 1790000000;
  own.fromPhone(none, 10000, 1);
  TEST_ASSERT_FALSE(own.has());
  own.fromPhone(reading(355000000, 1790000000), 10000, 1);
  own.fromPhone(reading(0, 1790000001), 11000, 1);  // lat 0 and lon set: still a place
  TEST_ASSERT_EQUAL_UINT32(2, own.id().seq);
  // A stock app setting only the clock: the fix stays, and does not count as fed.
  own.fromPhone(none, 20000, 1);
  TEST_ASSERT_TRUE(own.has());
  TEST_ASSERT_EQUAL_UINT32(2, own.id().seq);
  TEST_ASSERT_EQUAL_UINT32(9000, own.fedAgeMs(20000));
}

void test_a_session_is_never_zero() {
  OwnFix own;
  own.fromPhone(reading(355000000, 1790000000), 10000, 0x00010001);
  TEST_ASSERT_NOT_EQUAL(0, own.id().session);
}

// The phone writes at least once a second, a repeat of its last fix when it
// has no new one, so a repeat keeps the fix fresh.
void test_a_fix_nobody_feeds_goes_stale() {
  OwnFix own;
  TEST_ASSERT_FALSE(own.fresh(10000));
  own.fromPhone(reading(355000000, 1790000000), 10000, 1);
  TEST_ASSERT_TRUE(own.fresh(10000 + OwnFix::STALE_MS - 1));
  TEST_ASSERT_FALSE(own.fresh(10000 + OwnFix::STALE_MS));
  own.fromPhone(reading(355000000, 1790000000), 24000, 1);
  TEST_ASSERT_EQUAL_UINT32(1, own.id().seq);
  TEST_ASSERT_TRUE(own.fresh(24000 + OwnFix::STALE_MS - 1));
  // Stale is not gone: the fix is still held, it is only not sent.
  TEST_ASSERT_TRUE(own.has());
  // Across the millis() wrap.
  OwnFix wrapped;
  wrapped.fromPhone(reading(355000000, 1790000000), 0xFFFFF000u, 1);
  TEST_ASSERT_TRUE(wrapped.fresh(0x00000F00u));
}

// 5d and 5e run on UTC from our own fix: the send grid every car shares, and
// how old another car's fix is when it arrives.
void test_utc_runs_on_from_the_fix_as_it_came_in() {
  OwnFix own;
  uint64_t utc = 0;
  TEST_ASSERT_FALSE(own.utcMs(10000, utc));
  own.fromPhone(reading(355000000, 1790000000, 250), 10000, 1);
  TEST_ASSERT_TRUE(own.utcMs(12000, utc));
  TEST_ASSERT_EQUAL_UINT64(1790000000250ull + 2000, utc);
  // A repeat of the same fix does not move it: the fix is no newer.
  own.fromPhone(reading(355000000, 1790000000, 250), 13000, 1);
  TEST_ASSERT_TRUE(own.utcMs(13000, utc));
  TEST_ASSERT_EQUAL_UINT64(1790000000250ull + 3000, utc);
  // A new one does.
  own.fromPhone(reading(355000100, 1790000004, 0), 14000, 1);
  TEST_ASSERT_TRUE(own.utcMs(14500, utc));
  TEST_ASSERT_EQUAL_UINT64(1790000004000ull + 500, utc);
  // A fix with no time says nothing about the clock.
  OwnFix untimed;
  untimed.fromPhone(reading(355000000, 0), 10000, 1);
  TEST_ASSERT_FALSE(untimed.utcMs(10000, utc));
}

static Fix gnssReading(int32_t lat, uint32_t fixSec) {
  Fix f = reading(lat, fixSec);
  f.external = false;
  return f;
}

void test_the_phone_fix_beats_the_boards_receiver_while_it_keeps_writing() {
  OwnFix own;
  own.fromPhone(reading(355000000, 1790000000), 10000, 1);
  // The receiver's fix while the phone is writing is not taken.
  own.fromGnss(gnssReading(355000500, 1790000001), 10500, 1);
  TEST_ASSERT_TRUE(own.fix().external);
  TEST_ASSERT_EQUAL_UINT32(1, own.id().seq);
  own.fromGnss(gnssReading(355000500, 1790000002), 10000 + OwnFix::PHONE_FRESH_MS - 1, 1);
  TEST_ASSERT_TRUE(own.fix().external);
  // The phone has gone quiet: the receiver fills in, as a new fix.
  own.fromGnss(gnssReading(355000600, 1790000003), 10000 + OwnFix::PHONE_FRESH_MS, 1);
  TEST_ASSERT_FALSE(own.fix().external);
  TEST_ASSERT_EQUAL_UINT32(2, own.id().seq);
  // And the phone takes over again the moment it writes.
  own.fromPhone(reading(355000700, 1790000004), 14000, 1);
  TEST_ASSERT_TRUE(own.fix().external);
  TEST_ASSERT_EQUAL_UINT32(3, own.id().seq);
}

void test_a_receiver_that_stops_producing_fixes_goes_stale() {
  OwnFix own;
  own.fromGnss(gnssReading(355000000, 1790000000), 10000, 1);
  TEST_ASSERT_TRUE(own.fresh(10000));
  // The same solution read every pass is not news, so it does not keep the fix fresh.
  own.fromGnss(gnssReading(355000000, 1790000000), 20000, 1);
  TEST_ASSERT_FALSE(own.fresh(10000 + OwnFix::STALE_MS));
  own.fromGnss(gnssReading(355000000, 1790000030), 40000, 1);
  TEST_ASSERT_TRUE(own.fresh(40000));
  TEST_ASSERT_EQUAL_UINT32(2, own.id().seq);
}

void test_the_fix_time_comes_from_the_solution_then_the_write() {
  Fix f;
  setMeasured(f, 1790000000, 250, 1790000009);
  TEST_ASSERT_EQUAL_UINT32(1790000000, f.fixSec);
  TEST_ASSERT_EQUAL_UINT16(250, f.fixMs);
  // An adjustment past a second, or below zero, is folded into the seconds.
  setMeasured(f, 1790000000, 1250, 0);
  TEST_ASSERT_EQUAL_UINT32(1790000001, f.fixSec);
  TEST_ASSERT_EQUAL_UINT16(250, f.fixMs);
  setMeasured(f, 1790000000, -250, 0);
  TEST_ASSERT_EQUAL_UINT32(1790000000 - 1, f.fixSec);
  TEST_ASSERT_EQUAL_UINT16(750, f.fixMs);
  // No solution time: the write's time, whole seconds.
  setMeasured(f, 0, 400, 1790000009);
  TEST_ASSERT_EQUAL_UINT32(1790000009, f.fixSec);
  TEST_ASSERT_EQUAL_UINT16(0, f.fixMs);
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
    uint8_t buf[POSITION_MIN];
    size_t n = encodePosition(p, buf, sizeof(buf));
    Position got;
    TEST_ASSERT_TRUE(decodePosition(buf, n, got));
    TEST_ASSERT_EQUAL_UINT16(cases[i].want, got.headingDeg);
  }
}

void test_position_truncates_a_long_name() {
  // A newer build may send a longer name than this one can hold. Dropping the
  // position would take the car off the map; clipping the name does not.
  uint8_t buf[POSITION_MIN + 40];
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

void test_a_grid_deadline_ahead_of_now_is_left_alone() {
  // A movement-triggered beacon before the 1 s deadline must not move it.
  // Advancing it on every send pushed it a second per extra beacon, and a car
  // that had sent four in quick succession then went quiet for over four.
  uint32_t deadline = 2000;
  const uint32_t sends[] = {1200, 1400, 1600, 1800};
  for (uint32_t at : sends) deadline = nextOnGrid(deadline, 1000, at);
  TEST_ASSERT_EQUAL_UINT32(2000, deadline);
}

void test_a_passed_grid_deadline_moves_to_the_next_point() {
  TEST_ASSERT_EQUAL_UINT32(3000, nextOnGrid(2000, 1000, 2000));
  TEST_ASSERT_EQUAL_UINT32(3000, nextOnGrid(2000, 1000, 2250));
  // A long slot wait skips the missed points rather than owing them.
  TEST_ASSERT_EQUAL_UINT32(6000, nextOnGrid(2000, 1000, 5400));
}

void test_the_grid_survives_the_millis_wrap() {
  const uint32_t deadline = 0xFFFFFF00;
  TEST_ASSERT_EQUAL_UINT32(deadline, nextOnGrid(deadline, 1000, 0xFFFFFE00));
  TEST_ASSERT_EQUAL_UINT32((uint32_t)(deadline + 1000), nextOnGrid(deadline, 1000, 0x10));
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

// ---- Forward slots sized to what they hold (lean on a V3) -------------------
//
// These run under both sizings: env:native has every slot full-size, and
// env:native-lean only FORWARD_FULL_SLOTS of them.

// [len] bytes of wire filled from [seed], so a copy handed back can be checked
// byte for byte. Mesh never parses what it holds.
static void patternFrame(uint8_t* wire, size_t len, uint8_t seed) {
  for (size_t i = 0; i < len; i++) wire[i] = (uint8_t)(seed + i * 7);
}

void test_the_longest_position_frame_fits_a_small_forward_slot() {
  Position p = posNamed(nullptr);
  // Every byte of the name used, as a sender that does not terminate it would.
  memset(p.name, 'x', sizeof(p.name));
  uint8_t body[POSITION_MIN + sizeof(p.name)];
  const size_t n = encodePosition(p, body, sizeof(body));
  TEST_ASSERT_EQUAL_UINT32(POSITION_MIN + sizeof(p.name), n);

  // Sealed, the body carries a tag on the end.
  uint8_t sealed[FRAME_MAX_PAYLOAD];
  memcpy(sealed, body, n);
  memset(sealed + n, 0xAA, TAG_LEN);
  Frame f;
  f.type = FRAME_POSITION;
  f.src = 1;
  f.id = 2;
  f.payload = sealed;
  f.len = (uint16_t)(n + TAG_LEN);
  uint8_t wire[FRAME_MAX];
  const size_t w = encodeFrame(f, wire, sizeof(wire));
  TEST_ASSERT_EQUAL_UINT32(POSITION_FRAME_MAX, w);
  TEST_ASSERT_TRUE(w <= FORWARD_SMALL_BYTES);
}

void test_positions_fill_small_slots_before_borrowing_full_ones() {
  Mesh m;
  m.reset();
  uint8_t pos[POSITION_FRAME_MAX];
  uint8_t voice[FRAME_MAX];
  patternFrame(pos, sizeof(pos), 1);
  patternFrame(voice, sizeof(voice), 2);

  for (size_t i = FORWARD_FULL_SLOTS; i < FORWARD_SLOTS; i++)
    TEST_ASSERT_TRUE(m.defer(pos, sizeof(pos), 1, (uint32_t)i, 10));
  // Small slots gone: one more position borrows a full-size one...
  TEST_ASSERT_TRUE(m.defer(pos, sizeof(pos), 1, 500, 10));
  // ...which leaves one fewer for whole frames.
  for (size_t i = 1; i < FORWARD_FULL_SLOTS; i++)
    TEST_ASSERT_TRUE(m.defer(voice, sizeof(voice), 2, (uint32_t)i, 10));
  TEST_ASSERT_FALSE(m.defer(voice, sizeof(voice), 2, 999, 10));
  TEST_ASSERT_FALSE(m.defer(pos, sizeof(pos), 1, 999, 10));
}

void test_a_whole_frame_is_never_squeezed_into_a_small_slot() {
  Mesh m;
  m.reset();
  uint8_t pos[POSITION_FRAME_MAX];
  uint8_t voice[FRAME_MAX];
  patternFrame(pos, sizeof(pos), 1);
  patternFrame(voice, sizeof(voice), 2);

  for (size_t i = 0; i < FORWARD_FULL_SLOTS; i++)
    TEST_ASSERT_TRUE(m.defer(voice, sizeof(voice), 2, (uint32_t)i, 10));
  // Only position-sized slots left, if any: a whole frame is refused, a
  // position still goes in.
  TEST_ASSERT_FALSE(m.defer(voice, sizeof(voice), 2, 999, 10));
  for (size_t i = FORWARD_FULL_SLOTS; i < FORWARD_SLOTS; i++)
    TEST_ASSERT_TRUE(m.defer(pos, sizeof(pos), 1, (uint32_t)i, 10));
  TEST_ASSERT_FALSE(m.defer(pos, sizeof(pos), 1, 999, 10));
}

void test_forwards_come_back_byte_exact_from_either_kind_of_slot() {
  Mesh m;
  m.reset();
  uint8_t pos[POSITION_FRAME_MAX];
  uint8_t voice[FRAME_MAX];
  patternFrame(pos, sizeof(pos), 3);
  patternFrame(voice, sizeof(voice), 4);
  TEST_ASSERT_TRUE(m.defer(voice, sizeof(voice), 2, 20, 20));
  TEST_ASSERT_TRUE(m.defer(pos, sizeof(pos), 1, 10, 10));

  // Oldest debt first, whichever slot it sits in.
  Forward out;
  TEST_ASSERT_TRUE(m.nextDue(30, out));
  TEST_ASSERT_EQUAL_UINT32(1, out.src);
  TEST_ASSERT_EQUAL_UINT32(10, out.id);
  TEST_ASSERT_EQUAL_UINT16(sizeof(pos), out.len);
  TEST_ASSERT_EQUAL_MEMORY(pos, out.wire, sizeof(pos));

  TEST_ASSERT_TRUE(m.nextDue(30, out));
  TEST_ASSERT_EQUAL_UINT32(2, out.src);
  TEST_ASSERT_EQUAL_UINT16(sizeof(voice), out.len);
  TEST_ASSERT_EQUAL_MEMORY(voice, out.wire, sizeof(voice));
  TEST_ASSERT_FALSE(m.nextDue(30, out));
}

void test_dedupe_holds_its_whole_window_at_peak_traffic() {
  Mesh m;
  m.reset();
  // A full ride's worth of distinct frames, spread over one window.
  const uint32_t frames = (uint32_t)(PEAK_FRAMES_PER_SEC * SEEN_TTL_MS / 1000);
  for (uint32_t i = 0; i < frames; i++)
    TEST_ASSERT_TRUE(m.firstSight(100 + i, 7, i * SEEN_TTL_MS / frames));
  // The first is still known at the end of it, so its late echo is not
  // mistaken for a new frame and forwarded a second time.
  TEST_ASSERT_FALSE(m.firstSight(100, 7, SEEN_TTL_MS));
}

void test_mesh_tables_stay_inside_their_budget() {
  // Build 36 ran a V3 out of internal RAM. The roster, dedupe table and held
  // forwards together, pinned so growing them is a decision and not drift.
  // Build 38's fix identity: 12 bytes a rider, 12 a position forward slot,
  // 5500 to 5944 bytes lean and 7424 to 7760 roomy.
#if TOUGE_LEAN_RAM
  TEST_ASSERT_TRUE(sizeof(Mesh) <= 5950);
#else
  TEST_ASSERT_TRUE(sizeof(Mesh) <= 7800);
#endif
  TEST_ASSERT_EQUAL(12, sizeof(FixId));
  // Still a whole ride, lean or not.
  TEST_ASSERT_TRUE(MAX_RIDERS >= 25);
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

// --- the receiver's own fix, to the phone ---------------------------------

static GnssFix sampleFix() {
  GnssFix f;
  f.latE7 = 351234567;
  f.lonE7 = -839876543;
  f.altitudeM = 512;
  f.speedKmh = 72;
  f.trackE5 = 27012345; // 270.12345 degrees
  f.sats = 11;
  f.hdopE2 = 94;
  f.fixTimeSec = 1790000000;
  return f;
}

void test_gnss_fix_formats_as_the_app_expects() {
  char js[160];
  size_t n = formatGnssFix(sampleFix(), js, sizeof(js));
  TEST_ASSERT_EQUAL_STRING(
      "{\"gf\":{\"la\":351234567,\"lo\":-839876543,\"al\":512,\"kh\":72,"
      "\"tr\":27012,\"sa\":11,\"hd\":94,\"t\":1790000000}}",
      js);
  TEST_ASSERT_EQUAL_size_t(strlen(js), n);
}

void test_gnss_fix_refuses_a_short_buffer() {
  char js[20];
  TEST_ASSERT_EQUAL_size_t(0, formatGnssFix(sampleFix(), js, sizeof(js)));
}

void test_gnss_forward_sends_each_new_fix_once_a_second() {
  GnssForward fwd;
  GnssFix f = sampleFix();
  TEST_ASSERT_TRUE(fwd.due(f, 1000));
  fwd.sent(f, 1000);
  // Same solution read again on the next 5 ms pass.
  TEST_ASSERT_FALSE(fwd.due(f, 1005));
  TEST_ASSERT_FALSE(fwd.due(f, 5000));
  // A new second, but too soon after the last send.
  f.fixTimeSec++;
  TEST_ASSERT_FALSE(fwd.due(f, 1500));
  TEST_ASSERT_TRUE(fwd.due(f, 1950));
}

void test_gnss_forward_skips_no_fix() {
  GnssForward fwd;
  GnssFix f = sampleFix();
  f.latE7 = 0;
  f.lonE7 = 0;
  TEST_ASSERT_FALSE(fwd.due(f, 1000));
  f = sampleFix();
  f.fixTimeSec = 0;
  TEST_ASSERT_FALSE(fwd.due(f, 1000));
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
  RUN_TEST(test_within_a_session_the_sequence_decides);
  RUN_TEST(test_a_long_drive_does_not_wrap_into_looking_older);
  RUN_TEST(test_a_rebooted_radio_is_a_new_session);
  RUN_TEST(test_the_same_millisecond_is_not_newer);
  RUN_TEST(test_each_new_fix_takes_the_next_sequence);
  RUN_TEST(test_a_write_with_no_coordinates_changes_nothing);
  RUN_TEST(test_a_session_is_never_zero);
  RUN_TEST(test_a_fix_nobody_feeds_goes_stale);
  RUN_TEST(test_utc_runs_on_from_the_fix_as_it_came_in);
  RUN_TEST(test_the_phone_fix_beats_the_boards_receiver_while_it_keeps_writing);
  RUN_TEST(test_a_receiver_that_stops_producing_fixes_goes_stale);
  RUN_TEST(test_the_fix_time_comes_from_the_solution_then_the_write);
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
  RUN_TEST(test_a_grid_deadline_ahead_of_now_is_left_alone);
  RUN_TEST(test_a_passed_grid_deadline_moves_to_the_next_point);
  RUN_TEST(test_the_grid_survives_the_millis_wrap);
  RUN_TEST(test_a_forward_waits_for_its_jitter);
  RUN_TEST(test_a_forward_overtaken_by_neighbours_is_dropped);
  RUN_TEST(test_a_forward_nobody_else_made_still_goes);
  RUN_TEST(test_forward_queue_drops_rather_than_delaying_what_is_waiting);
  RUN_TEST(test_a_forward_scheduled_across_the_millis_wrap_still_fires);
  RUN_TEST(test_the_longest_position_frame_fits_a_small_forward_slot);
  RUN_TEST(test_positions_fill_small_slots_before_borrowing_full_ones);
  RUN_TEST(test_a_whole_frame_is_never_squeezed_into_a_small_slot);
  RUN_TEST(test_forwards_come_back_byte_exact_from_either_kind_of_slot);
  RUN_TEST(test_dedupe_holds_its_whole_window_at_peak_traffic);
  RUN_TEST(test_mesh_tables_stay_inside_their_budget);
  RUN_TEST(test_distance_is_close_enough_to_be_a_gate);
  RUN_TEST(test_distance_does_not_overflow_on_a_full_span_of_longitude);
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
  RUN_TEST(test_gnss_fix_formats_as_the_app_expects);
  RUN_TEST(test_gnss_fix_refuses_a_short_buffer);
  RUN_TEST(test_gnss_forward_sends_each_new_fix_once_a_second);
  RUN_TEST(test_gnss_forward_skips_no_fix);
  return UNITY_END();
}
