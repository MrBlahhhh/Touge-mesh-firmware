#pragma once
//
// A test talker, so relaying, suppression and room for voice can be measured on
// the bench before there is any audio (docs/plans/2026-09-27-voice-coding-plan.md).
//
// The phone switches it on with a command to its own radio. The radio then sends
// voice-sized FRAME_VOICE frames every 60 ms, as a talker would, and every radio
// that hears one counts it instead of handing it to its phone: how many of the
// talker's frames arrived, over how many hops, and how late, read off the ride's
// shared second. Test frames are forwarded like real voice, which is the point.
//
// Two private-port payloads (see the registry in phonebatch.h):
//   0xC4  test talker command, phone to its own radio, never transmitted:
//         0 magic | 1 version | 2 on (0/1) | 3 frame body bytes (0: default)
//   0xC5  test voice frame body, radio to radio over 2.4 GHz:
//         0 magic | 1 version | 2 session, new each time the talker starts |
//         3-6 seq from 1, big-endian | 7-8 the sender's phase in the ride's
//         second when it sent, ms, big-endian (0xFFFF: no clock) | padding
//         to the chosen size
//
// Platform-free: time and the ride phase come in as arguments.

#include <stddef.h>
#include <stdint.h>

namespace touge {

static const uint8_t TEST_TALKER_MAGIC = 0xC4;
static const uint8_t TEST_VOICE_MAGIC = 0xC5;
static const uint8_t TEST_VOICE_VERSION = 1;
static const size_t TEST_VOICE_HEADER = 9;

// One packet of a real talker: 60 ms of AMR, 16.7 a second.
static const uint32_t TEST_VOICE_PERIOD_MS = 60;
// A packet today (107 B) and with the previous packet repeated (203 B).
static const uint8_t TEST_VOICE_DEFAULT_BYTES = 107;
static const uint16_t TEST_VOICE_NO_PHASE = 0xFFFF;
// The app repeats "on" every 30 s; a talker not told again in 2.5 of those stops,
// so a killed app or a phone gone cannot leave a radio flooding the lane.
static const uint32_t TEST_TALKER_LEASE_MS = 75000;
// Talkers heard within this are reported; older ones stay counted but quiet.
static const uint32_t TEST_VOICE_REPORT_FRESH_MS = 10000;
// How far out of order a forward can bring a frame: 2 hops of jitter is well under
// half a second, 8 frames. Further back than this is a talker that restarted.
static const uint32_t TEST_VOICE_REORDER = 32;

struct TestTalkerCommand {
  bool on = false;
  uint8_t bodyBytes = TEST_VOICE_DEFAULT_BYTES;
};

bool decodeTestTalker(const uint8_t* in, size_t len, TestTalkerCommand& out);
size_t encodeTestTalker(const TestTalkerCommand& c, uint8_t* out, size_t cap);

// A test frame's body, padded to [bodyBytes] (at least the header).
size_t encodeTestVoice(uint8_t session, uint32_t seq, uint16_t phaseMs, uint8_t bodyBytes, uint8_t* out,
                       size_t cap);
bool isTestVoice(const uint8_t* in, size_t len);
bool decodeTestVoice(const uint8_t* in, size_t len, uint8_t& session, uint32_t& seq, uint16_t& phaseMs);

// Delay from the sender's phase to ours, ms, -500 to 499, if both clocks are
// known. Negative is clock skew: a child's epoch sits a few ms either side of
// its parent's, and a direct frame flies in 1-2 ms. A frame over half a second
// late also reads negative, and is no use to a listener anyway.
bool testVoiceDelayMs(uint16_t sentPhaseMs, uint16_t heardPhaseMs, int32_t& delayMs);

// What one listener heard from each test talker, since that talker last started.
class VoiceMeter {
 public:
  static const size_t TALKERS = 4;
  static const uint8_t MAX_HOPS = 3; // 0 direct, 1, 2, 3 or more

  struct Talker {
    uint32_t src = 0;
    uint8_t session = 0;
    uint32_t firstSeq = 0;
    uint32_t lastSeq = 0;
    uint32_t heard = 0;
    uint32_t heardAtMs = 0;
    uint32_t hops[MAX_HOPS + 1] = {};
    uint32_t delayed = 0;   // frames with a delay reading
    uint32_t skewed = 0;    // of those, read negative and counted as 0
    uint32_t delaySumMs = 0;
    uint32_t delayMaxMs = 0;
    bool used = false;
  };

  void reset();
  // A test frame arrived: its sender, session and seq, how many hops it came, the
  // sender's phase and ours (TEST_VOICE_NO_PHASE when either clock is unknown).
  void heard(uint32_t src, uint8_t session, uint32_t seq, uint8_t hopsAway, uint16_t sentPhaseMs,
             uint16_t heardPhaseMs, uint32_t nowMs);
  const Talker* talkers() const { return talkers_; }

 private:
  Talker talkers_[TALKERS];
};

// {"vt":{"s":src,"rx":heard,"ex":expected,"h":[h0,h1,h2,h3],"dm":mean,"dx":max,"sk":skewed,"ag":ms}}
// for one talker; 0 if it did not fit.
size_t formatVoiceMeter(const VoiceMeter::Talker& t, uint32_t nowMs, char* out, size_t cap);

// {"vs":{"on":0/1,"b":bytes,"tx":sent,"tf":refused}}, the talker's own side.
size_t formatTestTalker(bool on, uint8_t bodyBytes, uint32_t sent, uint32_t refused, char* out,
                        size_t cap);

}  // namespace touge
