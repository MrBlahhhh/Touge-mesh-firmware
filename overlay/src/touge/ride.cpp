#include "ride.h"
#include <string.h>

namespace touge {
namespace {

const char* HEX = "0123456789abcdef";

// Domain separation. Every derived value comes from the same 32 bytes, so each
// one is hashed under its own prefix. Without this, two values derived from
// one PSK are two halves of the same digest, and publishing one publishes the
// other.
void derive(const char* label, const uint8_t* in, size_t inLen, uint8_t out[SHA256_LEN]) {
  uint8_t buf[32 + PSK_LEN];
  size_t n = strlen(label);
  memcpy(buf, label, n);
  memcpy(buf + n, in, inLen);
  sha256(buf, n + inLen, out);
}

} // namespace

bool deriveRide(const char* key, Ride& out) {
  if (key == nullptr) return false;
  size_t n = strnlen(key, RIDE_KEY_MAX + 1);
  if (n < RIDE_KEY_MIN || n > RIDE_KEY_MAX) return false;

  memset(&out, 0, sizeof(out));
  memcpy(out.key, key, n);

  uint8_t nameHash[SHA256_LEN];
  sha256((const uint8_t*)key, n, nameHash);

  // "tg-" plus the first three bytes as hex. Matches Invite.channelName.
  out.channelName[0] = 't';
  out.channelName[1] = 'g';
  out.channelName[2] = '-';
  for (int i = 0; i < 3; i++) {
    out.channelName[3 + i * 2] = HEX[(nameHash[i] >> 4) & 0x0F];
    out.channelName[4 + i * 2] = HEX[nameHash[i] & 0x0F];
  }

  // The name is public, so the PSK must not be the rest of that same digest.
  // Matches Invite.channelPsk.
  derive("touge-psk:", (const uint8_t*)key, n, out.psk);

  out.valid = true;
  return true;
}

bool deriveFast(const uint8_t* psk, size_t pskLen, FastNet& out) {
  if (psk == nullptr || pskLen > PSK_LEN) return false;
  memset(&out, 0, sizeof(out));

  derive("touge-fast-key:", psk, pskLen, out.key);

  uint8_t net[SHA256_LEN];
  derive("touge-fast-net:", psk, pskLen, net);
  // 1 to 11, the channels legal in every region we ship to. Twelve to fourteen
  // are not, and a board that picks one simply goes deaf with no error.
  out.wifiChannel = (uint8_t)(net[0] % 11 + 1);
  out.chanByte = net[1];

  out.valid = true;
  return true;
}


} // namespace touge
