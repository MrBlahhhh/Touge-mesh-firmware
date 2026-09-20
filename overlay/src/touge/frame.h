#pragma once
//
// What goes over the air.
//
// Deliberately not Meshtastic's protobuf. A Meshtastic position packet costs
// about 55 bytes once the envelope, the node info and the protobuf field tags
// are counted, and at SHORT_FAST that is 58 ms of airtime per car per ping.
// Four cars flooding at three hops is sixteen transmissions a round, so the
// packet size is the whole budget. This one is 24 bytes.
//
// No Arduino here, and no crypto here, so the whole format is testable on a
// laptop. Encryption wraps the payload one layer up in cipher.h.

#include <stdint.h>
#include <stddef.h>

namespace touge {

enum FrameType : uint8_t {
  FRAME_POSITION = 1,
  FRAME_TEXT = 2,
  FRAME_VOICE = 3,
  FRAME_PAIR = 4,
  FRAME_ROSTER = 5,
};

static const uint8_t FRAME_MAGIC = 0x54; // 'T', same as the app's VoicePacket
static const uint8_t FRAME_VERSION = 1;
static const size_t FRAME_HEADER = 14;

// ESP-NOW tops out at 250 bytes and is the tightest of the two radios, so it
// sets the ceiling for both. A LoRa frame is much smaller in practice.
static const size_t FRAME_MAX = 250;
static const size_t FRAME_MAX_PAYLOAD = FRAME_MAX - FRAME_HEADER;
// What is left for the plaintext once the authentication tag has taken its
// share of the payload.
static const size_t FRAME_MAX_BODY = FRAME_MAX_PAYLOAD - 8;

struct Frame {
  uint8_t type = 0;
  uint32_t src = 0;
  // Thirty-two bits, not sixteen. This id is half the AES-CTR nonce, and a
  // nonce that repeats under one channel key hands an eavesdropper the XOR of
  // the two plaintexts. Sixteen bits wraps inside a day of pinging. It also
  // has to survive a reboot, so the counter behind it lives in NVS.
  uint32_t id = 0;
  // Counts down, not up. A node forwards only while this is above zero, which
  // is what stops four cars in a bowl from echoing a packet forever.
  uint8_t hops = 0;
  // First byte of the channel hash. Not security, just a cheap way to drop
  // another group's traffic before spending anything on decryption.
  uint8_t chan = 0;
  const uint8_t* payload = nullptr;
  uint16_t len = 0;
};

// Returns bytes written, or 0 if it would not fit.
size_t encodeFrame(const Frame& f, uint8_t* out, size_t cap);

// Returns false on anything malformed. `out.payload` points into `in`, so the
// buffer has to outlive the frame.
bool decodeFrame(const uint8_t* in, size_t len, Frame& out);

// ---- Position --------------------------------------------------------------
//
// Fixed point at 1e7, the same scale the app and Meshtastic both use, so a
// coordinate survives the trip through either without a second rounding.

struct Position {
  int32_t lat = 0; // degrees * 1e7
  int32_t lon = 0;
  uint16_t headingDeg = 0; // 0..359, quantised to 2 degrees on the wire
  uint8_t speedMph = 0;    // capped at 255, which no one on a touge will reach
  uint8_t batteryPct = 255; // 255 means unknown
  bool hasFix = false;
  bool phoneAttached = false;
  // Whether this car's cycle is locked to its own GPS pulse. Everyone needs to
  // know, because the reference car has to be one of the locked ones or the
  // cars with GPS and the cars without end up on two different cycles.
  bool clockLocked = false;
  // Carried only now and then. A name on every ping is pure airtime, and the
  // roster on the other end only needs to learn it once.
  char name[16] = {0};
};

static const size_t POSITION_MIN = 12;

size_t encodePosition(const Position& p, uint8_t* out, size_t cap);
bool decodePosition(const uint8_t* in, size_t len, Position& out);

// Metres between two fixed-point coordinates.
//
// Equirectangular, not haversine. Over the few hundred metres that separate
// cars on one road the error is under a tenth of a percent, and this runs in a
// beacon gate that fires several times a second.
uint32_t distanceM(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

} // namespace touge
