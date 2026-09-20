#!/usr/bin/env bash
#
# Drops the Touge fast-mesh module into a Meshtastic checkout.
#
#   ./apply-overlay.sh ~/src/meshtastic-firmware
#
# Copies our sources in and makes two edits to Modules.cpp so the module gets
# constructed. Safe to run again after a git pull: every edit checks for itself
# first, so a second run over an already-patched tree changes nothing.
#
# Nothing else in the Meshtastic tree is touched. That is deliberate. The whole
# reason for a module rather than a fork of the internals is that upstream can
# move underneath us and this keeps applying.

set -euo pipefail

DEST="${1:-}"
if [[ -z "$DEST" ]]; then
  echo "usage: $0 <path to meshtastic firmware checkout>" >&2
  exit 2
fi
if [[ ! -f "$DEST/src/modules/Modules.cpp" ]]; then
  echo "error: $DEST does not look like a Meshtastic checkout" >&2
  echo "       (no src/modules/Modules.cpp)" >&2
  exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> copying sources"
mkdir -p "$DEST/src/touge" "$DEST/src/modules/esp32"
cp "$HERE"/overlay/src/touge/*.h "$HERE"/overlay/src/touge/*.cpp "$DEST/src/touge/"
cp "$HERE"/overlay/src/modules/esp32/TougeFastModule.h \
   "$HERE"/overlay/src/modules/esp32/TougeFastModule.cpp \
   "$DEST/src/modules/esp32/"

MODULES="$DEST/src/modules/Modules.cpp"

# The two edits. Both are anchored on lines that have been stable upstream for
# a long time; if either anchor ever goes missing the script says so loudly
# rather than producing a tree that builds without the module in it.
if grep -q 'TougeFastModule.h' "$MODULES"; then
  echo "==> Modules.cpp already carries the include"
else
  if ! grep -q '#include "modules/esp32/PaxcounterModule.h"' "$MODULES"; then
    echo "error: could not find the include anchor in Modules.cpp" >&2
    echo "       upstream moved; patch it by hand and update this script" >&2
    exit 1
  fi
  sed -i.bak '/#include "modules\/esp32\/PaxcounterModule.h"/i\
#if !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)\
#include "modules/esp32/TougeFastModule.h"\
#endif
' "$MODULES"
  echo "==> added the include"
fi

if grep -q 'new TougeFastModule' "$MODULES"; then
  echo "==> Modules.cpp already constructs the module"
else
  # Ahead of PositionModule, and that ordering is load-bearing.
  #
  # Modules see a packet in the order they were constructed, and it is
  # PositionModule that writes a heard position into NodeDB. Ours has to get
  # there first to be able to say "this car is already on 2.4 GHz, its LoRa
  # position is two seconds old, do not let it overwrite what we have".
  # Constructed after PositionModule, the stale write has already happened.
  if ! grep -q 'positionModule = new PositionModule();' "$MODULES"; then
    echo "error: could not find the setup anchor in Modules.cpp" >&2
    echo "       upstream moved; the module must be constructed BEFORE" >&2
    echo "       PositionModule or 2.4 GHz will not take precedence" >&2
    exit 1
  fi
  sed -i.bak '/positionModule = new PositionModule();/i\
#if defined(ARCH_ESP32) \&\& !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)\
    tougeFastModule = new TougeFastModule();\
#endif
' "$MODULES"
  echo "==> added the constructor, ahead of PositionModule"
fi

rm -f "$MODULES.bak"

echo
echo "done. now, from $DEST:"
echo "    pio run -e heltec-v4"
echo "    pio run -e heltec-v4 -t upload"
