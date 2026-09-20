# Touge fast mesh

A Meshtastic module that adds a second radio path over the board's own 2.4 GHz
side, for the traffic that has no business on LoRa.

Meshtastic keeps doing everything it already does. This adds position at 1 Hz
and push-to-talk audio over ESP-NOW, and falls back to LoRa on its own when a
car drops out of line of sight.

## Why not a firmware of its own

Because the phone app, the OLED, and every Meshtastic node on the mountain
already agree on how to talk to each other, and throwing that away to save
thirty bytes a packet is a bad trade. A module keeps:

- the N30 v3 boards working without reflashing
- the app's existing `MeshtasticLink` working unchanged
- standalone GNSS, which Meshtastic already does
- interoperability with anyone else's Meshtastic node

## Hardware

Heltec WiFi LoRa 32 **V4**. Two things about it are easy to get wrong:

- **Get the 28 dBm high-power version, not the 22 dBm one.** Six decibels is
  roughly double the range and it is the single biggest hardware lever here.
  Listings that do not say which they are shipping are usually the 22.
- **GNSS is not on the bare board.** The SH1.25-8pin header is, but the
  receiver (CM121, 9600 baud) ships on the V4 expansion kit. Without it the
  board has no fix of its own and depends on a phone.

The V4 also carries an external PA (GC1109 on 4.2, KCT8103L on 4.3) that has to
be driven or the LoRa side transmits about a milliwatt while looking healthy.
Stock Meshtastic handles that; `src/touge/board_v4.h` records the pin map and
the reasoning for reference.

The V3 you already own works too, minus one thing: its 2.4 GHz is on a fixed
spring antenna with no u.FL, so ESP-NOW there gets whatever that gives you. The
V4 has a dedicated `2.4G ANT` connector.

## Build

```sh
git clone https://github.com/meshtastic/firmware meshtastic
cd meshtastic && git submodule update --init --recursive && cd ..

./apply-overlay.sh ./meshtastic
cd meshtastic
pio run -e heltec-v4
pio run -e heltec-v4 -t upload
```

`apply-overlay.sh` copies the sources in and makes two anchored edits to
`src/modules/Modules.cpp`. It is safe to run again after a `git pull`.

## Host tests

The wire format, the key derivation and the flood suppression have to agree
between a phone, a board and every other board, and none of them need a radio
to check. They are platform-free and tested on the host:

```sh
pio test -e native
```

This needs a host C++ compiler (MinGW-w64, MSVC, or gcc). It is the only part
of this that runs without hardware, so it is worth having.

## Joining a ride

There is no separate pairing flow, on purpose.

The only secret is the ride key, and it already reaches the board as the
Meshtastic primary channel PSK: the app derives it in `Invite.channelPsk` and
pushes it over BLE, and the Meshtastic channel QR does the same for anyone
joining without the app. The 2.4 GHz network derives from that PSK:

```
ride key --(Invite.channelPsk, app)--> channel PSK --(deriveFast, here)--> Wi-Fi channel + ESP-NOW key
```

So whatever puts a board on the ride puts it on both radios. Change the channel
and both follow within a few seconds; the roster and dedupe table are dropped
so cars from the last ride do not linger on the map.

`deriveRide()` in `src/touge/ride.cpp` reproduces the app's half of that chain.
Nothing on the board calls it in normal operation. It exists so the host tests
can prove this firmware and `Invite.kt` still agree byte for byte, and so a
board can be provisioned from a bare ride key over serial with no phone.

## Push to talk

The board has no microphone and no speaker, so audio stays on the phone, which
already captures and plays it in `Intercom.kt`.

**Outbound:** the app sends a voice packet as an ordinary Meshtastic data packet
on `PRIVATE_APP` addressed **to the local radio's own node number**, not to
broadcast. `Router::sendLocal` delivers a packet aimed at this node straight to
the modules and never transmits it, which is what keeps speech off LoRa. The
module picks it up and sends it over ESP-NOW instead.

Addressing it to broadcast would put 12 kbps of audio on the LoRa mesh and take
it down for everyone. The module refuses to forward voice when the fast lane is
down rather than letting it fall through to LoRa.

**Inbound:** a voice frame heard over ESP-NOW is handed to the phone as a normal
packet on the same port, over the BLE link it already has open.

## What is on the wire

A position is 26 bytes: a 14-byte header and a 12-byte body. Meshtastic's is
about 55 once the protobuf envelope is counted.

```
header  0     magic 'T'
        1     version | type
        2..5  sender (Meshtastic node number, so both radios agree on identity)
        6..9  packet id
        10    hops remaining
        11    channel byte, a cheap reject for another group's traffic
        12,13 payload length
body    0..7  lat, lon as int32 at 1e7
        8     heading, two-degree steps
        9     speed, mph
        10    battery percent
        11    flags
        12..  name, sent every 30 s rather than every ping
```

The payload is AES-256-CTR under a key derived from the channel PSK. The packet
id is half the nonce, which is why it is 32 bits and why the counter is kept in
NVS: counting up from zero after a reboot would replay nonces and hand anyone
listening the XOR of two positions.

## Latency

The target is under a second from one car's GPS to another car's screen, and
the transport is not the part that costs anything: a hop over ESP-NOW is two or
three milliseconds, and two hops with jitter is under fifty.

What decides freshness is `BEACON_MS`, at 250 ms. At one second a position is
already half a second stale on average before it is even sent, which alone eats
the budget. Four cars at 4 Hz over two hops runs at roughly a fifth of the
channel once suppression is working.

There are no dedicated repeaters and there is no need for any. Every node
forwards, which is what `FAST_HOPS = 2` means: it covers a convoy strung out
far enough that the front and back cannot hear each other but the middle can
hear both.

Flooding does need two things that are easy to leave out:

- **Random delay per forward.** Every car that heard a frame reaches the
  forwarding decision in the same microsecond. Sent immediately, they collide
  and the forward reaches nobody, and it gets worse with more cars, not better.
  Each one waits a random slice of `FORWARD_JITTER_MS` instead.
- **Counting the copies.** Having heard `SUPPRESS_AFTER` copies of a packet,
  everyone within earshot has it and one more transmission is interference. A
  held frame that gets overtaken while it waits is dropped. In a four-car
  convoy in line of sight nearly every forward is redundant, and this is what
  keeps them from costing anything. `Mesh::suppressed()` counts them.

Beacons carry their own jitter too, because cars powered up together otherwise
fall into lockstep and collide on every single one.

## Known limits

- **ESP-NOW is line of sight.** A 2.4 GHz receiver bottoms out near -95 dBm
  against roughly -137 for LoRa at SF11. This will not reach over a ridge and
  is not meant to; LoRa still does that.
- **Espressif long-range mode is all or nothing.** `TOUGE_FAST_LONG_RANGE`
  buys about 10 dB, but an LR node and a plain node cannot hear each other at
  all. Every board on the ride has to run the same setting.
- **BLE and Wi-Fi share one 2.4 GHz radio.** They coexist, but they time-share,
  so a BLE link carrying audio and ESP-NOW carrying the same audio will
  contend. How much has not been measured.
- **If Meshtastic's Wi-Fi client joins an access point, the AP owns the
  channel** and the derived channel is ignored. Boards still agree, because
  they all read the interface rather than the derivation, but two groups could
  then land on one channel.

## Not done

- Nothing here has run on hardware yet. It compiles as C++ but no board has
  seen it.
- The app side of push to talk still sends to broadcast; it needs to address
  voice packets to the radio's own node number for the interception above to
  work.
- No UI. Fast-lane state is in the logs only; `fastNeighbours()` is there for
  an OLED frame that has not been written.
