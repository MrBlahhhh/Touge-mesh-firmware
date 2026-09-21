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

What decides freshness is the beacon, and the beacon answers two separate
questions: is there anything worth saying, and is it our turn to say it.

### The gate: 20 metres or 3 seconds

Taken from Blue Force Tracker, which reports every 30 seconds or every 50
metres of travel, whichever comes first.

A distance trigger bounds the thing that actually matters. Not how old a
position is, but how wrong it is. At `GATE_METRES = 20` nobody's icon on your
map is ever more than about four car lengths from where that car really is,
whether it is doing 70 or sitting at a junction. A pure time trigger gives you
the opposite: it spends the most airtime on the car that has not moved.

At 60 mph that works out to a beacon every 750 ms or so. Parked, it drops to
the `GATE_IDLE_MS` heartbeat at 3 s and costs almost nothing.

### The slots

Beacons are the only traffic that is both periodic and constant, which makes
them the traffic that reliably collides, and it gets worse with every car
added. So they get slots instead of a politeness scheme.

`CYCLE_MS` is 250 ms, cut into `MAX_SLOTS` (9) of 27 ms each. Every car works
out the same ordering from the same roster and transmits only in its own:

- **Slot is rank.** How many node numbers on the ride sort below yours. No
  sorting, no allocation, no handing slots out, and every car computing it over
  the same roster gets the same answer.
- **A car alone free-runs.** No schedule worth keeping and nothing to collide
  with, and waiting for a sync that will never arrive would mean never
  transmitting at all.

The roster says *whose* slot is whose. Something else has to say *when* the
slots are, and there are two answers.

### Where the cycle comes from

**GPS, when there is a fix.** The GNSS pulse-per-second output marks the UTC
second to well under a microsecond. Only the position inside the cycle ever
matters, and because the cycle divides a second exactly, every second boundary
is also a cycle boundary — so the time since the last pulse is the phase, and
no date, no UTC seconds and no shared epoch are needed. A missed pulse is
harmless for the same reason: 1200 ms after the last edge is still 200 ms into
a cycle. `static_assert` holds the cycle to a divisor of 1000.

This is what makes the schedule robust rather than merely present:

- **No reference car.** Nobody's turning off down a side road costs the ride
  its clock.
- **Cars that have never met are already in step.** Two groups merging do not
  have to converge first.
- **Microseconds instead of milliseconds.** The old limit was the jitter on
  whatever path a beacon took to reach you.

A receiver that loses its fix freezes the last edge, which would look locked
while drifting a second further out every second, so a pulse older than
`PULSE_STALE_US` counts as no clock at all. The edge timestamp is written in an
interrupt and read in a task, and a 64-bit value is two stores on a 32-bit
core, so the reader retries across a sequence counter rather than risking a
torn read that would put the clock wildly out.

Meshtastic sets `PIN_GPS_PPS` to `INPUT` and never attaches a handler, so the
pin is free.

**The reference car, when there is no fix.** The fallback: one car's beacons
mark the cycle for everyone else. Only a frame heard directly is used, since
one relayed by a neighbour carries that neighbour's forwarding jitter. Good to
a few milliseconds against a 27 ms slot, and it keeps the ride working in a
tunnel, or on a board with no receiver fitted at all.

### Mixed rides, which is the normal case

Some cars will have a GNSS receiver on the board and some will only have a
phone. That is fine, but it constrains who gets to be the reference, and
getting it wrong is worse than having no schedule:

**The reference must be a car whose own clock is GPS-locked.** Cars with a fix
take the cycle from their pulse; cars without take it from the reference's
beacons. If the reference is itself free-running, those two groups end up on
cycles that have nothing to do with each other, and they collide *every time*
rather than occasionally. So a locked car always wins the job and the lowest
node number only breaks the tie. Every beacon carries a flag saying whether the
sender's clock is locked, which is how everyone agrees on the choice.

**The reference is therefore not always slot zero.** It used to be, back when
it was simply the lowest number. Now a locked car can outrank a lower-numbered
free-running one, so its beacon marks its own slot rather than the cycle start,
and `syncTo` subtracts `referenceSlot` to recover the boundary.

If nobody on the ride has a fix, the lowest number takes it and everyone
free-runs together, which is consistent because none of them has anything
better to agree on.

Cardo's DMC does this (US10277748) with a leader election and a designated
synchronizer. This is the same idea with the negotiation removed.

**It is not real TDMA.** ESP-NOW sits on 802.11, whose MAC does its own carrier
sense and backoff underneath and cannot be turned off, and the clock comes from
received beacons rather than GPS, so it is good to a couple of milliseconds and
no better. Against a 27 ms slot that is plenty. What this removes is
self-collision, the loss that grows with the size of the group. It does not
make the channel exclusive.

Reception is timestamped in the ESP-NOW driver callback rather than in the
polling loop, because the 5 ms a poll can sit waiting would be a fifth of a
slot.

### Voice and forwards stay contended

Deliberately. Voice is bursty and latency-critical, and push-to-talk means
there is normally one talker, so slotting it would add up to a full cycle of
delay to buy nothing. Forwards are aperiodic. Both keep the jitter scheme:

- **Random delay per forward.** Every car that heard a frame reaches the
  forwarding decision in the same microsecond. Sent immediately they collide
  and the forward reaches nobody. Each waits a random slice of
  `FORWARD_JITTER_MS`.
- **Counting the copies.** Having heard `SUPPRESS_AFTER` copies, everyone
  within earshot has it and one more transmission is interference, so a held
  frame overtaken while it waits is dropped. In a four-car convoy in line of
  sight nearly every forward is redundant. `Mesh::suppressed()` counts them.

### No repeaters

There are none and none are needed. Every node forwards, which is what
`FAST_HOPS = 2` means: it covers a convoy strung out far enough that the front
and back cannot hear each other but the middle can hear both.

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

## Licence

GPL-3.0-or-later. See `LICENSE`.

Not really a choice, and worth saying why rather than leaving a file sitting
there. This is a module that compiles into Meshtastic's firmware, Meshtastic is
GPL-3.0, and the thing that ends up on a board is the two of them linked into
one binary. Distributing that binary means distributing a derivative work, so
the source that went into it is GPL-3.0 whatever this file says. Naming it is
just being honest about where it already stood.

What that does mean: anyone can take this, change it, and sell it, so long as
they publish their changes under the same terms.

What it does not reach: the Android app. That is a separate program that talks
to the board over BLE, the same way any Meshtastic client does. Nothing here
links into it and nothing here obliges it.
