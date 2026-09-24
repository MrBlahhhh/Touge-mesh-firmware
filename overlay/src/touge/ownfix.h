#pragma once
//
// This car's own fix, and the identity it goes out under (SCALE-PLAN 5a, 5b).
//
// The radio names each fix it is handed, by its phone or by its own GNSS: one
// session drawn per boot, and a sequence that counts up with every new fix.
// Both lanes send the fix under that name, so a phone that hears it over
// 2.4 GHz and again over LoRa knows it is one fix.
//
// The phone's fix wins while the phone keeps writing it; the board's receiver
// fills in when it stops. Nothing goes out on either lane once no fix has been
// fed for STALE_MS.
//
// Platform-free: the module reads Meshtastic's positions into a Fix.

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

class OwnFix {
 public:
  // How long the last fix is still sent after nothing has fed one. Counted on
  // the radio's own clock from the last write or new fix, not from the fix's
  // measured time: a mute on the measured time once silenced a board whose
  // phone was connected. The phone writes at least once a second (a repeat of
  // its last fix when there is no new one), so fifteen seconds rides out a BLE
  // reconnect and its config download, and stops a board whose phone has gone
  // well inside the 30 s the app takes to call a car lost.
  static const uint32_t STALE_MS = 15000;
  // How long after the phone's last write its fix still outranks the board's
  // own receiver: three missed writes.
  static const uint32_t PHONE_FRESH_MS = 3000;

  // The phone's fix, from its write to this radio. A repeat of the same fix is
  // still the phone saying it is there. [entropy] is any random number; the
  // first fix draws the session from it.
  void fromPhone(const Fix& reading, uint32_t nowMs, uint32_t entropy);
  // The board's own receiver, read every pass. Ignored while the phone's fix
  // is fresh; a reading already held is not news.
  void fromGnss(const Fix& reading, uint32_t nowMs, uint32_t entropy);

  // A fix has been held since boot, fresh or not.
  bool has() const { return has_; }
  // Fed within STALE_MS: worth sending.
  bool fresh(uint32_t nowMs) const { return has_ && (uint32_t)(nowMs - fedMs_) < STALE_MS; }
  // Since the last write or new fix; 0 before the first.
  uint32_t fedAgeMs(uint32_t nowMs) const { return has_ ? nowMs - fedMs_ : 0; }
  const Fix& fix() const { return fix_; }
  FixId id() const;

 private:
  // Takes [reading] as the current fix: the next sequence number if it differs
  // from the one held. False for a reading with no coordinates.
  bool take(const Fix& reading, uint32_t entropy);

  Fix fix_;
  uint32_t seq_ = 0;
  uint32_t fedMs_ = 0;
  uint32_t phoneFedMs_ = 0;
  uint16_t session_ = 0;
  bool has_ = false;
  bool phoneFed_ = false;
};

}  // namespace touge
