#include "frame.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace touge {
namespace {

void put32(uint8_t* b, uint32_t v) {
  b[0] = (uint8_t)(v >> 24);
  b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);
  b[3] = (uint8_t)v;
}

uint32_t get32(const uint8_t* b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

void put16(uint8_t* b, uint16_t v) {
  b[0] = (uint8_t)(v >> 8);
  b[1] = (uint8_t)v;
}

uint16_t get16(const uint8_t* b) { return (uint16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]); }

} // namespace

size_t encodeFrame(const Frame& f, uint8_t* out, size_t cap) {
  if (f.len > FRAME_MAX_PAYLOAD) return 0;
  if (cap < FRAME_HEADER + f.len) return 0;
  if (f.len > 0 && f.payload == nullptr) return 0;
  // The type shares a byte with the version, so a type that does not fit in
  // four bits would silently corrupt the version and be read as a stray packet
  // by every other node. Refuse instead.
  if (f.type > 0x0F) return 0;

  out[0] = FRAME_MAGIC;
  out[1] = (uint8_t)((FRAME_VERSION << 4) | (f.type & 0x0F));
  put32(out + 2, f.src);
  put32(out + 6, f.id);
  out[10] = f.hops;
  out[11] = f.chan;
  put16(out + 12, f.len);
  if (f.len > 0) memcpy(out + FRAME_HEADER, f.payload, f.len);
  return FRAME_HEADER + f.len;
}

bool decodeFrame(const uint8_t* in, size_t len, Frame& out) {
  if (in == nullptr || len < FRAME_HEADER) return false;
  if (in[0] != FRAME_MAGIC) return false;
  if ((in[1] >> 4) != FRAME_VERSION) return false;

  uint16_t plen = get16(in + 12);
  // The length field is attacker- and noise-controlled, so it is checked
  // against what actually arrived rather than trusted. A corrupted length that
  // overran here would hand the caller a pointer past the buffer.
  if ((size_t)plen + FRAME_HEADER != len) return false;

  out.type = (uint8_t)(in[1] & 0x0F);
  out.src = get32(in + 2);
  out.id = get32(in + 6);
  out.hops = in[10];
  out.chan = in[11];
  out.len = plen;
  out.payload = plen > 0 ? in + FRAME_HEADER : nullptr;
  return true;
}

size_t encodePosition(const Position& p, uint8_t* out, size_t cap) {
  size_t nameLen = strnlen(p.name, sizeof(p.name));
  if (cap < POSITION_MIN + nameLen) return 0;

  put32(out + 0, (uint32_t)p.lat);
  put32(out + 4, (uint32_t)p.lon);
  // Heading at two-degree steps. A car's heading is never known better than
  // that from GNSS at road speed, and it buys back a byte on every ping.
  out[8] = (uint8_t)((p.headingDeg % 360) / 2);
  out[9] = p.speedMph;
  out[10] = p.batteryPct;
  out[11] = (uint8_t)((p.hasFix ? 0x01 : 0) | (p.phoneAttached ? 0x02 : 0) |
                      (p.clockLocked ? 0x04 : 0));
  if (nameLen > 0) memcpy(out + POSITION_MIN, p.name, nameLen);
  return POSITION_MIN + nameLen;
}

bool decodePosition(const uint8_t* in, size_t len, Position& out) {
  if (in == nullptr || len < POSITION_MIN) return false;

  out.lat = (int32_t)get32(in + 0);
  out.lon = (int32_t)get32(in + 4);
  out.headingDeg = (uint16_t)(in[8] * 2);
  out.speedMph = in[9];
  out.batteryPct = in[10];
  out.hasFix = (in[11] & 0x01) != 0;
  out.phoneAttached = (in[11] & 0x02) != 0;
  out.clockLocked = (in[11] & 0x04) != 0;

  size_t nameLen = len - POSITION_MIN;
  // A sender on a newer build may carry a longer name than this build knows
  // how to hold. Truncating beats rejecting the position outright: a car on
  // the map with a clipped name is worth more than no car on the map.
  if (nameLen > sizeof(out.name) - 1) nameLen = sizeof(out.name) - 1;
  memset(out.name, 0, sizeof(out.name));
  if (nameLen > 0) memcpy(out.name, in + POSITION_MIN, nameLen);
  return true;
}

uint32_t distanceM(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  // Sixty-four bit, and not for precision. Longitude spans -1.8e9 to +1.8e9 at
  // this scale, so a car either side of the date line produces a difference of
  // 3.6e9, which does not fit in the int32 the operands are stored in.
  const double SCALE = 1e-7;
  double dLat = (double)((int64_t)lat2 - (int64_t)lat1) * SCALE;
  double dLon = (double)((int64_t)lon2 - (int64_t)lon1) * SCALE;

  const double M_PER_DEG = 111320.0;
  double meanLat = ((double)lat1 + (double)lat2) * 0.5 * SCALE * (M_PI / 180.0);
  // Longitude lines converge towards the poles, so a degree of longitude is
  // worth less the further north you are. Without this a car at 60 degrees
  // would read twice as far east as it is.
  double x = dLon * M_PER_DEG * cos(meanLat);
  double y = dLat * M_PER_DEG;

  double d = sqrt(x * x + y * y);
  if (d <= 0) return 0;
  if (d > (double)0xFFFFFFFFu) return 0xFFFFFFFFu;
  return (uint32_t)(d + 0.5);
}

} // namespace touge
