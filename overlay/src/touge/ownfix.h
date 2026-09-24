#pragma once
//
// This car's own fix, and the identity it goes out under (SCALE-PLAN 5a).
//
// The radio names each fix it is handed, by its phone or by its own GNSS: one
// session drawn per boot, and a sequence that counts up with every new fix.
// Both lanes send the fix under that name, so a phone that hears it over
// 2.4 GHz and again over LoRa knows it is one fix.
//
// Platform-free: the module reads Meshtastic's position into a Fix.

#include <stdint.h>
#include "frame.h"

namespace touge {

struct Fix {
  int32_t lat = 0;  // degrees * 1e7
  int32_t lon = 0;
  uint32_t trackE5 = 0;   // Meshtastic's ground_track, degrees * 1e5
  uint32_t speedKmh = 0;  // Meshtastic's ground_speed
  uint32_t fixSec = 0;    // when it was measured, epoch seconds; 0 unknown
  uint16_t fixMs = 0;
  bool external = false;  // the phone's fix, not the board's own GNSS
};

// The fix time out of Meshtastic's Position: the solution's [timestamp] and its
// millisecond adjustment when it has one, else [time], which is all an older
// phone write carries.
void setMeasured(Fix& fix, uint32_t timestamp, int32_t millisAdjust, uint32_t time);

// Both fix times known, and [a] measured after [b].
bool measuredAfter(const Fix& a, const Fix& b);

class OwnFix {
 public:
  // What the radio now holds as its position, read every pass. Anything that
  // differs from the current fix is a new one and takes the next sequence
  // number. No coordinates means no fix, and nothing goes out until there is
  // one. [entropy] is any random number; the first fix draws the session
  // from it.
  void observe(const Fix& reading, uint32_t entropy);

  bool has() const { return has_; }
  const Fix& fix() const { return fix_; }
  FixId id() const;

 private:
  Fix fix_;
  uint32_t seq_ = 0;
  uint16_t session_ = 0;
  bool has_ = false;
};

}  // namespace touge
