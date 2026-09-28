#include "voicetest.h"

#include <stdio.h>
#include <string.h>

namespace touge {

bool decodeTestTalker(const uint8_t* in, size_t len, TestTalkerCommand& out) {
  if (in == nullptr || len < 4 || in[0] != TEST_TALKER_MAGIC || in[1] != TEST_VOICE_VERSION) return false;
  out.on = in[2] != 0;
  out.bodyBytes = in[3] == 0 ? TEST_VOICE_DEFAULT_BYTES : in[3];
  if (out.bodyBytes < TEST_VOICE_HEADER) out.bodyBytes = (uint8_t)TEST_VOICE_HEADER;
  return true;
}

size_t encodeTestTalker(const TestTalkerCommand& c, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < 4) return 0;
  out[0] = TEST_TALKER_MAGIC;
  out[1] = TEST_VOICE_VERSION;
  out[2] = c.on ? 1 : 0;
  out[3] = c.bodyBytes;
  return 4;
}

size_t encodeTestVoice(uint8_t session, uint32_t seq, uint16_t phaseMs, uint8_t bodyBytes, uint8_t* out,
                       size_t cap) {
  const size_t len = bodyBytes < TEST_VOICE_HEADER ? TEST_VOICE_HEADER : bodyBytes;
  if (out == nullptr || cap < len) return 0;
  memset(out, 0, len);
  out[0] = TEST_VOICE_MAGIC;
  out[1] = TEST_VOICE_VERSION;
  out[2] = session;
  out[3] = (uint8_t)(seq >> 24);
  out[4] = (uint8_t)(seq >> 16);
  out[5] = (uint8_t)(seq >> 8);
  out[6] = (uint8_t)seq;
  out[7] = (uint8_t)(phaseMs >> 8);
  out[8] = (uint8_t)phaseMs;
  return len;
}

bool isTestVoice(const uint8_t* in, size_t len) {
  return in != nullptr && len >= TEST_VOICE_HEADER && in[0] == TEST_VOICE_MAGIC && in[1] == TEST_VOICE_VERSION;
}

bool decodeTestVoice(const uint8_t* in, size_t len, uint8_t& session, uint32_t& seq, uint16_t& phaseMs) {
  if (!isTestVoice(in, len)) return false;
  session = in[2];
  seq = ((uint32_t)in[3] << 24) | ((uint32_t)in[4] << 16) | ((uint32_t)in[5] << 8) | in[6];
  phaseMs = (uint16_t)((in[7] << 8) | in[8]);
  return true;
}

bool testVoiceDelayMs(uint16_t sentPhaseMs, uint16_t heardPhaseMs, int32_t& delayMs) {
  if (sentPhaseMs >= 1000 || heardPhaseMs >= 1000) return false;
  // The phase wraps every second: sent at 990 and heard at 20 took 30 ms, and
  // heard 2 ms "before" it was sent is skew, -2, not 998.
  delayMs = ((int32_t)heardPhaseMs - (int32_t)sentPhaseMs + 1500) % 1000 - 500;
  return true;
}

void VoiceMeter::reset() {
  for (Talker& t : talkers_) t = Talker();
}

void VoiceMeter::heard(uint32_t src, uint8_t session, uint32_t seq, uint8_t hopsAway, uint16_t sentPhaseMs,
                       uint16_t heardPhaseMs, uint32_t nowMs) {
  Talker* t = nullptr;
  for (Talker& c : talkers_)
    if (c.used && c.src == src) t = &c;
  // A new session is the talker started again (switched on, or its radio
  // rebooted): counted afresh. Seq alone cannot say, since it restarts at 1.
  // A seq far behind the last covers a reboot that drew the same session byte.
  const bool fresh = t == nullptr || t->session != session ||
                     (int32_t)(seq - t->lastSeq) < -(int32_t)TEST_VOICE_REORDER;
  if (t == nullptr) {
    for (Talker& c : talkers_)
      if (!c.used) {
        t = &c;
        break;
      }
  }
  if (t == nullptr) {
    // Every seat taken: the talker heard longest ago gives way.
    t = &talkers_[0];
    for (Talker& c : talkers_)
      if ((int32_t)(c.heardAtMs - t->heardAtMs) < 0) t = &c;
  }
  if (fresh) {
    *t = Talker();
    t->used = true;
    t->src = src;
    t->session = session;
    t->firstSeq = seq;
    t->lastSeq = seq;
  }
  // A forward can land ahead of the direct copy of an earlier frame.
  if ((int32_t)(seq - t->firstSeq) < 0) t->firstSeq = seq;
  if ((int32_t)(seq - t->lastSeq) > 0) t->lastSeq = seq;
  t->heard++;
  t->heardAtMs = nowMs;
  t->hops[hopsAway > MAX_HOPS ? MAX_HOPS : hopsAway]++;
  int32_t delay = 0;
  if (testVoiceDelayMs(sentPhaseMs, heardPhaseMs, delay)) {
    t->delayed++;
    if (delay < 0) {
      t->skewed++;
      delay = 0;
    }
    t->delaySumMs += (uint32_t)delay;
    if ((uint32_t)delay > t->delayMaxMs) t->delayMaxMs = (uint32_t)delay;
  }
}

size_t formatVoiceMeter(const VoiceMeter::Talker& t, uint32_t nowMs, char* out, size_t cap) {
  if (out == nullptr || cap == 0 || !t.used) return 0;
  const uint32_t expected = t.lastSeq - t.firstSeq + 1;
  const uint32_t mean = t.delayed > 0 ? t.delaySumMs / t.delayed : 0;
  int n = snprintf(out, cap,
                   "{\"vt\":{\"s\":\"%08lx\",\"rx\":%lu,\"ex\":%lu,\"h\":[%lu,%lu,%lu,%lu],\"dm\":%lu,\"dx\":%lu,"
                   "\"sk\":%lu,\"ag\":%lu}}",
                   (unsigned long)t.src, (unsigned long)t.heard, (unsigned long)expected,
                   (unsigned long)t.hops[0], (unsigned long)t.hops[1], (unsigned long)t.hops[2],
                   (unsigned long)t.hops[3], (unsigned long)mean, (unsigned long)t.delayMaxMs,
                   (unsigned long)t.skewed, (unsigned long)(nowMs - t.heardAtMs));
  if (n <= 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

size_t formatTestTalker(bool on, uint8_t bodyBytes, uint32_t sent, uint32_t refused, char* out,
                        size_t cap) {
  if (out == nullptr || cap == 0) return 0;
  int n = snprintf(out, cap, "{\"vs\":{\"on\":%u,\"b\":%u,\"tx\":%lu,\"tf\":%lu}}", on ? 1u : 0u,
                   (unsigned)bodyBytes, (unsigned long)sent, (unsigned long)refused);
  if (n <= 0 || (size_t)n >= cap) return 0;
  return (size_t)n;
}

}  // namespace touge
