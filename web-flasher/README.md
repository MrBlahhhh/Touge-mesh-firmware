# Touge radio flasher

A static page that puts the Touge build on a Heltec V3 or V4 from Chrome or
Edge on a computer, over Web Serial, with Espressif's esptool-js (pinned to
0.7.0 on jsDelivr). No build step: `index.html`, `flasher.js` and `style.css`
are the whole page. The layout follows flasher.meshtastic.org (three steps,
a device picker, a flash dialog) with our own name, mark and colours.

## What it does

**Device.** Connect reads the chip, the flash size and the embedded PSRAM, and
applies the same rule as `../flash-all.ps1`:

| Chip | Flash | PSRAM | Board |
|---|---|---|---|
| ESP32-S3 | 8 MB | none | Heltec V3 |
| ESP32-S3 | 16 MB | 2 MB | Heltec V4 |

Anything else gets "not a Heltec V3 or V4, nothing flashed". The USB ID is
never used. "I know what this is" picks the board by hand, like `-Map`; the
chip is still read before writing and must be an ESP32-S3 with enough flash.
A Heltec V4 R8 (8 MB PSRAM) is refused on purpose: it's a different board
with its own Meshtastic env.

**Update** (the default) writes the app at `0x10000`, then the initial
otadata at `0xe000` so the bootloader runs app0. It never writes NVS at
`0x9000` (the Bluetooth bond) or the littlefs partition (the settings and the
ride). Before writing it checks the md5 of the partition table on the radio
against this build's; a different layout is refused with "use Fresh install".
Why otadata: see the comment in `publish-release.ps1`.

**Fresh install** erases the whole chip, then writes the factory image at
`0x0` (bootloader, partition table, otadata, app) and the littlefs image at
the `spiffs` offset when the build has one. The V3 build currently has none:
after the erase the partition is blank and the firmware formats it on first
boot (`FSCom.begin(true)`), and `data/` only holds `static/.gitkeep`, so a
built image would be an empty file system anyway.

Every file is checked against the manifest's sha256 after download and before
anything is written, and against its md5 on the flash after writing.

**Reset and the port.** Reading the chip puts it in the ROM bootloader, where
it has no Bluetooth. The page resets it back into its firmware and closes the
port as soon as the chip is read, after a flash (good or failed), when the
flash dialog closes, and on leaving the page (best effort, never mid-write).
Flashing opens the port again, which puts it back in the bootloader; it reuses
the port picked at Connect when exactly one granted port has the same USB IDs,
otherwise it asks again. The reset is a `CustomReset` with `D0|R1|W200|R0|W200`
(EN low with GPIO0 high), which works on the V3's USB-UART bridge and the V4's
native USB. esptool-js 0.7.0's own `after("hard_reset")` only drops RTS, which
is already low, so it doesn't reset anything.

## Publishing a build

1. Build in the Meshtastic checkout with a plain `pio run -e heltec-v3` and
   `pio run -e heltec-v4`. Not `-t upload`: only the plain run makes the
   littlefs image and the `.mt.json`.
2. Bring Touge-mesh-firmware up to date with this repo's `firmware/` and push
   it. `publish-release.ps1` refuses to publish a binary whose overlay and core
   patches aren't in that checkout at a committed state (GPL-3.0: the exact
   source has to be available), and records its commit as the source link.
3. Run it:

   ```powershell
   .\publish-release.ps1 -Notes "What changed, one line"
   ```

   Defaults: `-Checkout C:\Projects\meshtastic-firmware`,
   `-Source C:\Projects\touge-mesh-firmware`,
   `-Site C:\Projects\touge-mesh-firmware-pages`. It copies the page, adds the
   build to `releases.json` (replacing an entry with the same build number)
   and copies the images to `firmware/<build>/<env>/`.

For a bench test before the source is public, stage into any folder with
`-Bench -Site <folder>` and serve it locally:

```powershell
.\publish-release.ps1 -Bench -Site C:\tmp\flasher
cd C:\tmp\flasher; python -m http.server 8000
```

Then open http://localhost:8000 in Chrome or Edge (Web Serial works on
localhost without HTTPS). A bench build shows a red banner and no source link.

## Hosting on GitHub Pages in Touge-mesh-firmware

The images are served from the Pages site itself, not from GitHub Release
assets. A release download answers a browser `fetch` from another origin
without CORS headers (tested 2026-09-23: `github.com/.../releases/download/...`
302s to `release-assets.githubusercontent.com`, and neither response carries
`Access-Control-Allow-Origin`; the `api.github.com` asset URL has it on the
302 but not on the redirect target), so the page can't read them.

The site lives on a `gh-pages` branch so the binaries stay out of `main`:

```powershell
cd C:\Projects\touge-mesh-firmware
git worktree add --orphan -b gh-pages ..\touge-mesh-firmware-pages
.\web-flasher\publish-release.ps1 -Notes "..."     # or run it from this repo
cd ..\touge-mesh-firmware-pages
git add -A
git commit -m "Build <n>"
git push -u origin gh-pages
```

Then on GitHub: Settings, Pages, "Deploy from a branch", `gh-pages`, `/ (root)`.
The page is at https://mrblahhhh.github.io/Touge-mesh-firmware/.

Each build adds about 13 MB (V3 about 4.6 MB, V4 about 8.2 MB with its 3.5 MB
littlefs image, which is mostly erased flash and compresses well in git).
To drop an old build, delete `firmware/<n>/` and its entry in `releases.json`.

## releases.json

```json
{
  "schema": 1,
  "sourceRepo": "https://github.com/MrBlahhhh/Touge-mesh-firmware",
  "releases": [{
    "build": 25,
    "date": "2026-09-22",
    "bench": false,
    "notes": "",
    "sourceCommit": "<Touge-mesh-firmware commit>",
    "meshtasticVersion": "2.8.1.d971d50",
    "meshtasticCommit": "d971d507c472770eecb8c001467cddd94a6333fe",
    "boards": {
      "heltec-v4": {
        "partitionTable": { "offset": "0x8000", "size": 3072, "md5": "..." },
        "partitions": { "nvs": "0x9000", "otadata": "0xe000", "app0": "0x10000", "app1": "0x650000", "spiffs": "0xc90000", "coredump": "0xff0000" },
        "update": [
          { "what": "app", "path": "firmware/25/heltec-v4/firmware-heltec-v4-2.8.1.d971d50.bin", "offset": "0x10000", "size": 2292240, "md5": "...", "sha256": "..." },
          { "what": "otadata", "path": "firmware/25/heltec-v4/otadata-boot-app0.bin", "offset": "0xe000", "size": 8192, "md5": "...", "sha256": "..." }
        ],
        "fresh": [
          { "what": "factory", "path": "...factory.bin", "offset": "0x0", "...": "..." },
          { "what": "littlefs", "path": "...littlefs-heltec-v4-2.8.1.d971d50.bin", "offset": "0xc90000", "...": "..." }
        ]
      }
    }
  }]
}
```

Files are written in list order. The partition offsets come from the build's
`partitions.bin`, cross-checked against the `.mt.json` when there is one.

## Licence

GPL-3.0-or-later, like the firmware. esptool-js is Apache-2.0 and loaded from
the CDN, not copied here. No Meshtastic code, logo or artwork is used.
