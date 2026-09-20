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
  TEST_ASSERT_NOT_NULL(m.note(7, posNamed("jackie"), HEARD_FAST, -40, 0, 1000));
  TEST_ASSERT_EQUAL_UINT32(1, m.count());
  TEST_ASSERT_NOT_NULL(m.note(7, posNamed("jackie"), HEARD_LORA, -110, 2, 2000));
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
  m.note(7, posNamed("jackie"), HEARD_FAST, -40, 0, 1000);
  m.note(7, posNamed(nullptr), HEARD_FAST, -41, 0, 2000);
  TEST_ASSERT_EQUAL_STRING("jackie", m.find(7)->pos.name);
}

void test_roster_will_not_bump_a_car_you_are_driving_behind() {
  Mesh m;
  m.reset();
  for (uint32_t i = 0; i < MAX_RIDERS; i++)
    TEST_ASSERT_NOT_NULL(m.note(i + 1, posNamed("x"), HEARD_FAST, -40, 0, 1000));
  TEST_ASSERT_EQUAL_UINT32(MAX_RIDERS, m.count());

  // Everyone is current, so a newcomer is turned away rather than evicting
  // someone whose position is still live on the screen.
  TEST_ASSERT_NULL(m.note(99, posNamed("late"), HEARD_FAST, -40, 0, 1000));

  // Once a seat has gone quiet, the newcomer takes it.
  uint32_t later = 1000 + RIDER_STALE_MS + 1;
  m.note(2, posNamed("x"), HEARD_FAST, -40, 0, later); // keep this one fresh
  TEST_ASSERT_NOT_NULL(m.note(99, posNamed("late"), HEARD_FAST, -40, 0, later));
  TEST_ASSERT_NOT_NULL(m.find(99));
  TEST_ASSERT_NOT_NULL(m.find(2));
}

void test_roster_drops_only_after_a_very_long_silence() {
  Mesh m;
  m.reset();
  m.note(7, posNamed("jackie"), HEARD_LORA, -110, 3, 1000);

  // Stale is not gone. A car that vanishes off the screen every time it dips
  // behind a ridge is worse than one that says it was last seen 90 seconds ago.
  m.age(1000 + RIDER_STALE_MS + 1);
  TEST_ASSERT_NOT_NULL(m.find(7));

  m.age(1000 + RIDER_DROP_MS + 1);
  TEST_ASSERT_NULL(m.find(7));
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
  return UNITY_END();
}
