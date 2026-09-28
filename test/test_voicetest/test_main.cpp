// The test talker: its command, its frames, and the listener's meter.

#include <unity.h>
#include <string.h>
#include "voicetest.h"

using namespace touge;

void test_the_command_round_trips() {
  uint8_t buf[8];
  TestTalkerCommand c;
  c.on = true;
  c.bodyBytes = 203;
  TEST_ASSERT_EQUAL_UINT32(4, encodeTestTalker(c, buf, sizeof(buf)));
  TestTalkerCommand back;
  TEST_ASSERT_TRUE(decodeTestTalker(buf, 4, back));
  TEST_ASSERT_TRUE(back.on);
  TEST_ASSERT_EQUAL_UINT8(203, back.bodyBytes);
}

void test_a_zero_size_means_the_default_and_a_tiny_one_the_header() {
  const uint8_t zero[] = {TEST_TALKER_MAGIC, TEST_VOICE_VERSION, 1, 0};
  TestTalkerCommand c;
  TEST_ASSERT_TRUE(decodeTestTalker(zero, sizeof(zero), c));
  TEST_ASSERT_EQUAL_UINT8(TEST_VOICE_DEFAULT_BYTES, c.bodyBytes);
  const uint8_t tiny[] = {TEST_TALKER_MAGIC, TEST_VOICE_VERSION, 1, 3};
  TEST_ASSERT_TRUE(decodeTestTalker(tiny, sizeof(tiny), c));
  TEST_ASSERT_EQUAL_UINT8(TEST_VOICE_HEADER, c.bodyBytes);
}

void test_other_payloads_are_not_the_command_or_a_test_frame() {
  const uint8_t voice[] = {0x54, 1, 0, 0, 0, 1, 0, 0};
  const uint8_t hello[] = {0xC2, 1, 2, 0, 247};
  TestTalkerCommand c;
  TEST_ASSERT_FALSE(decodeTestTalker(voice, sizeof(voice), c));
  TEST_ASSERT_FALSE(decodeTestTalker(hello, sizeof(hello), c));
  TEST_ASSERT_FALSE(isTestVoice(voice, sizeof(voice)));
  const uint8_t shortFrame[] = {TEST_VOICE_MAGIC, TEST_VOICE_VERSION, 0, 0};
  TEST_ASSERT_FALSE(isTestVoice(shortFrame, sizeof(shortFrame)));
}

void test_a_frame_is_padded_to_the_size_and_decodes() {
  uint8_t buf[250];
  TEST_ASSERT_EQUAL_UINT32(203, encodeTestVoice(0xA7, 0x01020304, 987, 203, buf, sizeof(buf)));
  uint8_t session = 0;
  uint32_t seq = 0;
  uint16_t phase = 0;
  TEST_ASSERT_TRUE(decodeTestVoice(buf, 203, session, seq, phase));
  TEST_ASSERT_EQUAL_UINT8(0xA7, session);
  TEST_ASSERT_EQUAL_UINT32(0x01020304, seq);
  TEST_ASSERT_EQUAL_UINT16(987, phase);
  // Too small a buffer is nothing, not an overrun.
  TEST_ASSERT_EQUAL_UINT32(0, encodeTestVoice(1, 1, 0, 203, buf, 100));
}

void test_the_delay_reads_across_the_second() {
  int32_t d = 0;
  TEST_ASSERT_TRUE(testVoiceDelayMs(990, 20, d));
  TEST_ASSERT_EQUAL_INT32(30, d);
  TEST_ASSERT_TRUE(testVoiceDelayMs(100, 145, d));
  TEST_ASSERT_EQUAL_INT32(45, d);
  TEST_ASSERT_FALSE(testVoiceDelayMs(TEST_VOICE_NO_PHASE, 20, d));
  TEST_ASSERT_FALSE(testVoiceDelayMs(20, TEST_VOICE_NO_PHASE, d));
}

// A listener whose epoch sits a few ms behind the talker's reads a direct frame as
// heard before it was sent. That is skew, not a frame nearly a second late.
void test_a_listener_clock_behind_the_talker_reads_as_skew() {
  int32_t d = 0;
  TEST_ASSERT_TRUE(testVoiceDelayMs(500, 498, d));
  TEST_ASSERT_EQUAL_INT32(-2, d);
  TEST_ASSERT_TRUE(testVoiceDelayMs(1, 999, d));
  TEST_ASSERT_EQUAL_INT32(-2, d);

  VoiceMeter m;
  m.reset();
  m.heard(7, 1, 1, 0, 500, 510, 1000);
  m.heard(7, 1, 2, 0, 560, 558, 1060);
  const VoiceMeter::Talker& t = m.talkers()[0];
  TEST_ASSERT_EQUAL_UINT32(2, t.delayed);
  TEST_ASSERT_EQUAL_UINT32(1, t.skewed);
  TEST_ASSERT_EQUAL_UINT32(10, t.delayMaxMs);
  TEST_ASSERT_EQUAL_UINT32(5, t.delaySumMs / t.delayed);
}

void test_the_meter_counts_what_arrived_against_what_was_sent() {
  VoiceMeter m;
  m.reset();
  // Seq 10 to 19, with 13 and 16 lost; one relayed copy.
  for (uint32_t seq = 10; seq < 20; seq++) {
    if (seq == 13 || seq == 16) continue;
    m.heard(0xB03436AE, 9, seq, seq == 17 ? 1 : 0, 100, 112, 1000 + seq * 60);
  }
  const VoiceMeter::Talker& t = m.talkers()[0];
  TEST_ASSERT_TRUE(t.used);
  TEST_ASSERT_EQUAL_UINT32(8, t.heard);
  TEST_ASSERT_EQUAL_UINT32(10, t.lastSeq - t.firstSeq + 1);
  TEST_ASSERT_EQUAL_UINT32(7, t.hops[0]);
  TEST_ASSERT_EQUAL_UINT32(1, t.hops[1]);
  TEST_ASSERT_EQUAL_UINT32(12, t.delaySumMs / t.delayed);

  char buf[233];
  TEST_ASSERT_TRUE(formatVoiceMeter(t, 1000 + 19 * 60 + 500, buf, sizeof(buf)) > 0);
  TEST_ASSERT_EQUAL_STRING(
      "{\"vt\":{\"s\":\"b03436ae\",\"rx\":8,\"ex\":10,\"h\":[7,1,0,0],\"dm\":12,\"dx\":12,\"sk\":0,\"ag\":500}}", buf);
}

void test_a_restarted_talker_is_counted_afresh() {
  VoiceMeter m;
  m.reset();
  // Ten minutes of talk, then the radio rebooted and the phone switched the talker
  // back on: seq starts at 1 again under a new session.
  for (uint32_t seq = 1; seq <= 10000; seq++) m.heard(7, 40, seq, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, seq * 60);
  m.heard(7, 41, 1, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 700000);
  m.heard(7, 41, 2, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 700060);
  const VoiceMeter::Talker& t = m.talkers()[0];
  TEST_ASSERT_EQUAL_UINT32(2, t.heard);
  TEST_ASSERT_EQUAL_UINT32(1, t.firstSeq);
  TEST_ASSERT_EQUAL_UINT32(2, t.lastSeq);

  // A reboot that happened to draw the same session byte: seq far behind the last.
  m.heard(7, 41, 3, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 700120);
  for (uint32_t seq = 4; seq <= 100; seq++) m.heard(7, 41, seq, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 700000 + seq * 60);
  m.heard(7, 41, 1, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 800000);
  TEST_ASSERT_EQUAL_UINT32(1, m.talkers()[0].heard);
}

// A short run switched off and on again: seq only goes from 25 back to 1, inside
// the reorder window, so only the new session says it is a new run.
void test_a_new_session_after_a_short_run_is_counted_afresh() {
  VoiceMeter m;
  m.reset();
  for (uint32_t seq = 1; seq <= 25; seq++) m.heard(7, 5, seq, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, seq * 60);
  m.heard(7, 6, 1, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 5000);
  const VoiceMeter::Talker& t = m.talkers()[0];
  TEST_ASSERT_EQUAL_UINT32(1, t.heard);
  TEST_ASSERT_EQUAL_UINT32(1, t.firstSeq);
  TEST_ASSERT_EQUAL_UINT32(1, t.lastSeq);

  // The same session stepping back inside the window is a late copy, not a restart.
  m.heard(7, 6, 2, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 5060);
  m.heard(7, 6, 30, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 6800);
  m.heard(7, 6, 10, 1, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 6810);
  TEST_ASSERT_EQUAL_UINT32(4, m.talkers()[0].heard);
  TEST_ASSERT_EQUAL_UINT32(1, m.talkers()[0].firstSeq);
}

// A forward can land ahead of the direct copy of the frame before it. That is the
// same run, not a restart, and widens what was expected.
void test_a_frame_out_of_order_is_the_same_run() {
  VoiceMeter m;
  m.reset();
  m.heard(7, 5, 21, 1, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 1000);
  m.heard(7, 5, 20, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 1002);
  m.heard(7, 5, 22, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 1060);
  const VoiceMeter::Talker& t = m.talkers()[0];
  TEST_ASSERT_EQUAL_UINT32(3, t.heard);
  TEST_ASSERT_EQUAL_UINT32(3, t.lastSeq - t.firstSeq + 1);
}

void test_more_talkers_than_seats_take_the_oldest_seat() {
  VoiceMeter m;
  m.reset();
  for (uint32_t i = 0; i < VoiceMeter::TALKERS; i++)
    m.heard(100 + i, 1, 1, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 1000 + i);
  m.heard(999, 1, 1, 0, TEST_VOICE_NO_PHASE, TEST_VOICE_NO_PHASE, 2000);
  bool newcomer = false, oldestGone = true;
  for (size_t i = 0; i < VoiceMeter::TALKERS; i++) {
    if (m.talkers()[i].src == 999) newcomer = true;
    if (m.talkers()[i].src == 100) oldestGone = false;
  }
  TEST_ASSERT_TRUE(newcomer);
  TEST_ASSERT_TRUE(oldestGone);
}

void test_the_reports_fit_a_payload_at_their_worst() {
  VoiceMeter::Talker t;
  t.used = true;
  t.src = 0xFFFFFFFF;
  t.firstSeq = 0;
  t.lastSeq = 0xFFFFFFFE;
  t.heard = t.delayed = t.skewed = t.delaySumMs = t.delayMaxMs = 0xFFFFFFFF;
  for (uint32_t& h : t.hops) h = 0xFFFFFFFF;
  char buf[233];
  TEST_ASSERT_TRUE(formatVoiceMeter(t, 0, buf, sizeof(buf)) > 0);
  TEST_ASSERT_TRUE(formatTestTalker(true, 255, 0xFFFFFFFF, 0xFFFFFFFF, buf, sizeof(buf)) > 0);
  TEST_ASSERT_EQUAL_STRING("{\"vs\":{\"on\":1,\"b\":255,\"tx\":4294967295,\"tf\":4294967295}}", buf);
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_command_round_trips);
  RUN_TEST(test_a_zero_size_means_the_default_and_a_tiny_one_the_header);
  RUN_TEST(test_other_payloads_are_not_the_command_or_a_test_frame);
  RUN_TEST(test_a_frame_is_padded_to_the_size_and_decodes);
  RUN_TEST(test_the_delay_reads_across_the_second);
  RUN_TEST(test_a_listener_clock_behind_the_talker_reads_as_skew);
  RUN_TEST(test_the_meter_counts_what_arrived_against_what_was_sent);
  RUN_TEST(test_a_restarted_talker_is_counted_afresh);
  RUN_TEST(test_a_new_session_after_a_short_run_is_counted_afresh);
  RUN_TEST(test_a_frame_out_of_order_is_the_same_run);
  RUN_TEST(test_more_talkers_than_seats_take_the_oldest_seat);
  RUN_TEST(test_the_reports_fit_a_payload_at_their_worst);
  return UNITY_END();
}
