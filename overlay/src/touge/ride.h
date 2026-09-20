#pragma once
//
// How a ride becomes a 2.4 GHz network.
//
// There is only ever one secret, and in a Meshtastic fork it is already on the
// board: the primary channel's PSK. The app derives that from the ride key in
// Invite.channelPsk and pushes it over BLE, and the Meshtastic channel QR does
// the same for anyone joining without the app. So the fast mesh derives from
// the PSK rather than asking for a key of its own, and joining the ride joins
// both radios at once. There is no second pairing flow to get wrong.
//
//   ride key --(Invite.channelPsk, in the app)--> channel PSK --(here)--> ESP-NOW
//
// deriveRide() below reproduces the app's half of that chain. Nothing on the
// board calls it in normal operation; it exists so the host tests can prove
// this firmware and Invite.kt still agree, and so a board can be provisioned
// from a bare ride key over serial with no phone present.

#include <stdint.h>
#include <stddef.h>
#include "sha256.h"

namespace touge {

static const size_t RIDE_KEY_MIN = 8;
static const size_t RIDE_KEY_MAX = 32;
static const size_t PSK_LEN = 32;
// "tg-" plus six hex characters. Eleven is the Meshtastic channel name limit
// and the app already lives inside it, so we do too rather than invent a
// second naming rule for the same ride.
static const size_t CHANNEL_NAME_LEN = 9;

struct Ride {
  char key[RIDE_KEY_MAX + 1] = {0};
  char channelName[CHANNEL_NAME_LEN + 1] = {0};
  uint8_t psk[PSK_LEN] = {0};
  bool valid = false;
};

// False if the key is shorter than the app will let you start a ride with.
bool deriveRide(const char* key, Ride& out);

// The 2.4 GHz side, derived from whatever PSK the primary channel is carrying.
struct FastNet {
  // Encrypts the ESP-NOW payload. Secret, and deliberately not the channel
  // PSK itself: a key reused across two radios with two different nonce
  // schemes is one bookkeeping mistake away from a repeat.
  uint8_t key[PSK_LEN] = {0};
  // ESP-NOW peers only hear each other on the same Wi-Fi channel. Deriving it
  // from the PSK means two groups on the same mountain usually land on
  // different channels instead of both talking over the middle of the band.
  uint8_t wifiChannel = 1;
  // Stamped in clear on every frame so a node can drop another group's traffic
  // without paying to decrypt it. Public, so it comes from a different digest
  // than the key does; taking it from the key would leak a key byte on air.
  uint8_t chanByte = 0;
  bool valid = false;
};

// False only if psk is null. An all-zero PSK is a real (if unwise) Meshtastic
// setting and is treated as one rather than silently refused.
// pskLen because a Meshtastic channel key is 0, 16 or 32 bytes depending on
// how the channel was set up, and all three have to land somewhere sane.
bool deriveFast(const uint8_t* psk, size_t pskLen, FastNet& out);


} // namespace touge
