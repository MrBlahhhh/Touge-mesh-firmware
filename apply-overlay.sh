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
  # Immediately after the ESP32 include block opens, not next to another
  # module's include.
  #
  # Anchoring on PaxcounterModule.h put ours *inside* `#if
  # !MESHTASTIC_EXCLUDE_PAXCOUNTER`, so a build with paxcounter switched off
  # lost our header while still constructing the module - a link error that
  # has nothing to do with either feature. Sitting directly under `#ifdef
  # ARCH_ESP32` it is guarded by our own condition and nobody else's.
  if ! grep -q '^#ifdef ARCH_ESP32$' "$MODULES"; then
    echo "error: could not find the include anchor in Modules.cpp" >&2
    echo "       upstream moved; patch it by hand and update this script" >&2
    exit 1
  fi
  # First occurrence only: the same line opens the setup block further down.
  perl -0pi -e 's{(#ifdef ARCH_ESP32\n)}{$1."#if !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)\n#include \"modules/esp32/TougeFastModule.h\"\n#endif\n"}e' "$MODULES"
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
  # Before the guard PositionModule sits in, not inside it.
  #
  # Inserting directly above the constructor put ours inside `#if
  # !MESHTASTIC_EXCLUDE_GPS`, which quietly ties the 2.4 GHz mesh to whether
  # this build has a GPS stack compiled in. Those are unrelated, and the
  # coupling would only show up as the fast lane silently never starting.
  if ! grep -Pzoq '#if !MESHTASTIC_EXCLUDE_GPS\n    positionModule = new PositionModule\(\);' "$MODULES"; then
    echo "error: could not find the setup anchor in Modules.cpp" >&2
    echo "       upstream moved; the module must be constructed BEFORE" >&2
    echo "       PositionModule or 2.4 GHz will not take precedence" >&2
    exit 1
  fi
  perl -0pi -e 's{(#if !MESHTASTIC_EXCLUDE_GPS\n    positionModule = new PositionModule\(\);)}{"#if defined(ARCH_ESP32) && !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)\n    tougeFastModule = new TougeFastModule();\n#endif\n".$1}e' "$MODULES"
  echo "==> added the constructor, ahead of PositionModule"
fi

rm -f "$MODULES.bak"

# The Meshtastic *core* files we patch beyond Modules.cpp, kept as unified diffs
# in core-patches/ so a fresh checkout gets them too - the rate limit that lets
# the fast lane's 1 Hz feed through (PhoneAPI.cpp) and the BLE params that stop
# the radio throttling itself after setup, and the advertising restart a
# disconnect could lose (both NimbleBluetooth.cpp), and from build 30 the phone
# queue's newest-wins positions (MeshService.cpp), the delivered hook
# (PhoneAPI.cpp) and the pre-encoded batch path (NimbleBluetooth.cpp), which
# TougeFastModule links against. From build 33, 0008 makes a 3 s button hold
# shut the board down without waiting for release. Without these
# the fast lane builds but does not carry. Idempotent: a patch that already
# reverse-applies is in, and is skipped.
if ls "$HERE"/core-patches/*.patch >/dev/null 2>&1; then
  echo "==> core patches"
  for patch in "$HERE"/core-patches/*.patch; do
    name="$(basename "$patch")"
    if patch -p1 -d "$DEST" -R --dry-run -f <"$patch" >/dev/null 2>&1; then
      echo "    $name already applied"
    elif patch -p1 -d "$DEST" --dry-run -f <"$patch" >/dev/null 2>&1; then
      patch -p1 -d "$DEST" -f <"$patch" >/dev/null && echo "    applied $name"
    else
      echo "    error: $name does not apply; upstream moved, patch by hand" >&2
      exit 1
    fi
  done
fi

echo
echo "done. now, from $DEST:"
echo "    pio run -e heltec-v4"
echo "    pio run -e heltec-v4 -t upload"
