# Touge fast mesh

A Meshtastic module plus twelve core patches for the Heltec WiFi LoRa 32 V3 and
V4 (ESP32-S3). It adds an ESP-NOW lane on the S3's 2.4 GHz radio for positions
and voice, and from build 39 sends the car's LoRa position itself. The build
number is `TOUGE_BUILD` in `TougeFastModule.cpp`.

## Build

```sh
git clone https://github.com/meshtastic/firmware meshtastic
cd meshtastic && git checkout d971d50 && git submodule update --init --recursive && cd ..
./apply-overlay.sh ./meshtastic
cd meshtastic && pio run -e heltec-v4   # or heltec-v3
```

The patches are made against meshtastic/firmware `d971d50`. `apply-overlay.sh`
is idempotent: it skips what's already applied, and stops if a patch or
`Modules.cpp` anchor no longer matches upstream.

Flash with `flash-all.ps1` (every V3/V4 on USB, chip identified with esptool,
NVS kept) or the Web Serial page in `web-flasher/`.

Every radio on a ride must run the same frame version (4, from build 40).
Boards on different versions drop each other's frames.

## What changes in Meshtastic

`apply-overlay.sh` copies `overlay/src/touge/` and
`overlay/src/modules/esp32/TougeFastModule.*` into the tree and edits
`src/modules/Modules.cpp` twice: the include, and the constructor, placed
ahead of `PositionModule`. The order matters. The module has to see a LoRa
position before `PositionModule` writes it to NodeDB.

Then the patches in `core-patches/`. The hooks in 0006, 0010, 0011 and 0012
stay null unless the module sets them; the other patches change behaviour
directly.

| Patch | Files | Change |
|---|---|---|
| 0001 | `PhoneAPI.cpp` | No 10 s rate limit on a position the phone addresses to its own radio (the 1 Hz fix feed) |
| 0002 | `NimbleBluetooth.cpp` | Keep the high-throughput BLE connection params after config instead of dropping to low power |
| 0003 | `NimbleBluetooth.cpp` | Restart advertising when there's no connection and no advertising (a lost wakeup left the radio dark until reboot) |
| 0004 | `GPS.cpp` | A board that has never answered a GNSS probe sets `gps_mode` NOT_PRESENT, so later boots skip the ~1 min probe |
| 0005 | `MeshService.*`, `StaticPointerQueue.h` | Phone queue: a car's newer position replaces its queued one (by fix identity), and a full queue evicts the oldest position instead of dropping the newest packet |
| 0006 | `PhoneAPI.*` | `phonePacketDeliveredHook`, called as a packet is handed to the phone |
| 0007 | `NimbleBluetooth.*` | `nimbleOfferToPhone` puts a pre-encoded position batch straight in the BLE read queue; from-phone queue 3 to 6; write-drop hook and counters |
| 0008 | `ButtonThread.*`, `Screen.*`, `Power.cpp`, `main-esp32.cpp` | Shut down after a 4 s hold with the button still down, draw the banner, and wait for release before deep sleep |
| 0009 | `mesh-pb-constants.h` | 16 phone-queue slots on an S3 without PSRAM (V3), for the internal RAM BLE needs beside Wi-Fi |
| 0010 | `MeshPacketQueue.*` | `tougeTxPlaceHook`: one position per car in the LoRa TX queue; a newer one takes the older one's place, an older one is refused |
| 0011 | `PositionModule.*` | `positionBroadcastOwnedHook`: periodic and smart position broadcasts stand down while the module sends the LoRa position |
| 0012 | `RadioInterface.*` | `tougeRelayEarlyHook`: relay in the ROUTER's early window for origins this car is shown to deliver further |

## Host tests

```sh
pio test -e native -e native-lean
```

`native-lean` uses the smaller tables a V3 builds with (`overlay/src/touge/ram.h`).
Needs a host C++ compiler on PATH.

## The 2.4 GHz lane

- **Radio.** ESP-NOW broadcast, station mode, Espressif LR PHY on
  (`TOUGE_FAST_LONG_RANGE`), 20 dBm, no modem sleep. Starts no earlier than
  8 s after boot and 2 s after the BLE server exists. On a V3, if bringing
  Wi-Fi up leaves under 16 KB of internal RAM, the lane stays off until reboot.
- **Keys.** Derived from the primary channel PSK (`deriveFast`, `ride.h`):
  ESP-NOW key, Wi-Fi channel (1, 6 or 11) and a clear-text group byte. Payload
  is AES-256-CTR with an 8-byte HMAC-SHA256 tag. The 32-bit packet id is half
  the nonce and its counter lives in NVS.
- **Channel.** Fixed. Hopping is in the code and off (`FAST_LANE_HOP`). A board
  that hears nobody for 6 s goes back to the key's channel.
- **Frame.** 14-byte header, version 4. A position body is 67 bytes plus a
  16-byte name every 30 s: 89 bytes on air, 105 with the name. Layout in
  `frame.h`. Only position and voice frames are sent; text, pair and roster
  types are defined and unused.
- **Beacon.** On moving 20 m or once a second, whichever comes first.
- **Schedule** (`schedule.h`). One second, four 250 ms blocks, each eight 27 ms
  leased slots and a 34 ms shared window. 32 leases, taken by rule from the
  roster, not handed out. With few cars a car also beacons in the free slots of
  its row, up to 4 Hz. Unleased cars use the shared window.
- **Clock.** The GNSS PPS edge when locked (three on-time pulses), otherwise
  the beacons of a reference car. Reference order: GPS-locked, then fit to keep
  time (most links working both ways), then lowest node number.
- **Forwarding.** Two hops. Weakest RSSI forwards first, in a 30 to 120 ms
  window that widens with the neighbour count; a held forward is dropped after
  3 copies are heard.
- **NodeDB.** A heard position is written with `updatePosition` and
  `last_heard`, so the OLED and stock app see it. A LoRa position from a car
  heard on 2.4 GHz in the last 3 s is let through for relaying, then the row
  is put back from the fresher 2.4 GHz one.
- **Phone.** After the app's hello, positions go to the phone in batches of
  the newest per car, not a packet each. The board's own GNSS fix goes to the
  phone at about 1 Hz as `gf` JSON.
- **Voice.** A `PRIVATE_APP` packet the phone addresses to its own radio
  goes out on ESP-NOW and never on LoRa (dropped if the lane is down). Voice
  heard on ESP-NOW goes to the phone on `PRIVATE_APP`. The app's voice
  transport is still loopback only.

Private-port payloads by first byte:

| Byte | Direction | Content |
|---|---|---|
| `{` | radio to phone | JSON: lane reports (`fl`, `fs`, `fq`), LoRa reports (`ll`, `lt`, `le`), own fix (`gf`); the app also uses it for profiles and invites |
| `T` 0x54 | both | voice frame |
| 0xC1 | radio to phone | position batch (`phonebatch.h`) |
| 0xC2 | phone to its radio | hello (`phonebatch.h`) |
| 0xC3 | radio to radio, LoRa | reach summary (`reach.h`) |

## Constraints

- An LR node and a non-LR node can't hear each other. Every board on a ride
  runs the same setting.
- If Meshtastic's Wi-Fi client is associated to an AP, the radio can't leave
  the AP's channel.
- BLE and Wi-Fi time-share one 2.4 GHz radio.

## Licence

GPL-3.0-or-later. See `LICENSE`.
