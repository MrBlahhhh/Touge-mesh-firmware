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

// How far the reference's claim is allowed to travel.
//
// Two cars can end up naming each other as the way to a reference that has
// gone away, each adding one to the other's count for ever. The cap turns that
// into a few wasted beacons: past it the route reads as unreachable, the car
// falls back to the best reference it can hear directly, and the ride
// re-converges from the anchor outward.
//
// Five, and not a number picked for feeling generous.
//
// Every hop inherits its parent's epoch from the arrival of a beacon, and a
// beacon leaves up to half a module tick either side of its slot boundary. The
// error is passed down the chain along with the clock, so the deepest car is
// the one closest to transmitting outside its slot. The assert in
// TougeFastModule derives this from the slot width, the frame airtime and the
// tick, and fails the build if any of them move; see the working there.
//
// Eight was the first figure here and it was wrong: at eight hops a car would
// have been transmitting a full slot width late, into whoever was next.
static const uint8_t MAX_REF_HOPS = 5;

// A route to the reference that does not exist, or does not exist yet.
static const uint8_t REF_UNREACHABLE = 0x0F;

static_assert(MAX_REF_HOPS < REF_UNREACHABLE, "the cap has to fit under the sentinel");

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
  // Which transmit slot this car holds, or SLOT_NONE if it has not claimed one
  // yet. Free on the wire: it rides in the spare half of the flags byte, since
  // there are nine slots and four bits to put them in.
  uint8_t slot = 0x0F;
  // Which channel this car believes the ride is on, and how recent that belief
  // is: two bits of index, six of generation. Carried by every car rather than
  // announced by one, so a car that missed a hop hears about it from whoever
  // it hears from next instead of having had one chance at a command.
  uint8_t hop = 0;
  // Which car this one believes is keeping time for the whole ride, and how
  // many hops away it thinks that car is.
  //
  // ## Why the reference has to travel
  //
  // Every car used to elect the lowest node number *it could hear*, which is
  // fine in a car park and wrong on half a mile of mountain road. The head
  // cannot hear the tail, so the head elects itself and the tail elects the
  // lowest car in the tail, and now there are two references - each free to
  // decide the channel is bad and hop, taking its own half of the ride with
  // it. Worse, neither half is ever "lost": each can hear plenty of cars, so
  // neither goes looking, and nothing ever brings them back together.
  //
  // So the belief travels instead of the signal. A car advertises the best
  // reference it knows of, not merely the best one it can hear, and takes the
  // best of what its neighbours advertise. The head's claim reaches the tail
  // through the cars in between, one hop per beacon, and the whole ride
  // converges on one answer without any car having to hear any particular
  // other car.
  //
  // refHops is what makes that safe to act on. A car five hops from the
  // reference cannot sync its clock to a beacon it will never receive, so it
  // syncs to whichever neighbour is closest to the reference instead - and
  // that neighbour's epoch is already the reference's. The count is what picks
  // that parent out, and capping it is what stops two cars pointing at each
  // other and counting to infinity when the reference goes away.
  uint32_t refId = 0;
  uint8_t refHops = REF_UNREACHABLE;
  // Whether that reference is disciplined by its own GPS. Travels with the id
  // because the rule is "locked beats unlocked, then lowest number", and a car
  // relaying the claim has to relay what makes it good.
  bool refLocked = false;
  // Carried only now and then. A name on every ping is pure airtime, and the
  // roster on the other end only needs to learn it once.
  char name[16] = {0};
};

static const size_t POSITION_MIN = 18;

size_t encodePosition(const Position& p, uint8_t* out, size_t cap);
bool decodePosition(const uint8_t* in, size_t len, Position& out);

// Metres between two fixed-point coordinates.
//
// Equirectangular, not haversine. Over the few hundred metres that separate
// cars on one road the error is under a tenth of a percent, and this runs in a
// beacon gate that fires several times a second.
uint32_t distanceM(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

} // namespace touge
