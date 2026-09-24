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
pio test -e native -e native-lean
```

`native-lean` runs the same suite with the smaller tables a board without PSRAM
(the Heltec V3) builds with; see `overlay/src/touge/ram.h`.

This needs a host C++ compiler (MinGW-w64, MSVC, or gcc). It is the only part
of this that runs without hardware, so it is worth having.

## Joining a ride

There is no separate pairing flow, on purpose.

The only secret is the ride key, and it already reaches the board as the
Meshtastic primary channel PSK: the app derives it in `Invite.channelPsk` and
pushes it over BLE, and the Meshtastic channel QR does the same for anyone
joining without the app. The 2.4 GHz network derives from that PSK:

```
ride key --(Invite.channelPsk, app)--> channel PSK --(deriveFast, here)--> ESP-NOW key + group byte
```

Every ride's lane sits on Wi-Fi channel 1 (`FAST_HOME_INDEX`, from build 42);
the key used to pick one of 1, 6 and 11, and picked a channel shared with a
Starlink router on the bench. Groups sharing the channel ignore each other by
the group byte and the tag.

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

A position frame is 89 bytes: a 14-byte header, a 67-byte body and an 8-byte
tag, plus the name every 30 s. Frame version 4, from build 40, which reads two
flag bits the election now depends on; boards on another version drop it.

```
header  0      magic 'T'
        1      version | type
        2..5   sender (Meshtastic node number, so both radios agree on identity)
        6..9   packet id
        10     hops remaining
        11     channel byte, a cheap reject for another group's traffic
        12,13  payload length
body    0..7   lat, lon as int32 at 1e7
        8      heading, two-degree steps
        9      speed, mph
        10     battery percent
        11     flags: fix, phone's fix, GPS-locked clock, extra beacon, fit to keep time
        12     channel belief (hop index and generation)
        13..17 reference car, its hops, whether it is GPS-locked and fit to keep time
        18     leased slot
        19..22 lease and schedule generations
        23..54 slot map: who was heard in each of the 32 slots
        55..66 fix identity: session (2), sequence (4), fix seconds (4), fix ms (2)
        67..   name, sent every 30 s rather than every ping
```

A reach summary (SCALE-PLAN 5e) goes over LoRa on the private port, first
byte 0xC3, then version, entry count and flags: the list goes on in the next
one (0x01), an early summary (0x02), entries carry the steady flag (0x04, from
build 43). 10 bytes an origin: node, fix sequence (low 16 bits), fix age on
arrival in 250 ms steps, seconds since heard, hops in the low nibble with 0x10
for "heard steadily direct", and the relay byte of the copy that got there
first. See `touge/reach.h` and "Who reaches whom" below.

The fix identity (SCALE-PLAN step 5a) names each of a car's fixes once, on
the radio whose car it is: a session drawn at random per boot, a sequence that
counts up with every new fix, and when the fix was measured. Relays forward the
frame untouched. The car's LoRa position carries the same values in
Meshtastic's `sensor_id`, `seq_number`, `timestamp` and
`timestamp_millis_adjust`, so the phone knows a fix heard on both lanes is one
fix, and an older one arriving late is older (`rankFix` in `touge/frame.h`).

## The LoRa position

From build 39 the radio sends its car's LoRa position itself (SCALE-PLAN 5b);
the phone only writes its fix to its own radio, which never goes on air.

- **Which fix.** The phone's, from its write (at least once a second); the
  board's own GNSS fills in once the phone has not written for 3 s. Both lanes
  stop 15 s after the last write or new GNSS fix, counted on the radio's clock
  (`touge/ownfix.h`).
- **How often.** Every 5 s while the channel is under a quarter busy, the
  share of the last minute Meshtastic measures as busy with everything heard
  and sent (its "polite" limit). Above that, the interval at which the same
  traffic would sit at 25 %, re-judged every 30 s, at most doubling a step,
  whole seconds, capped at 20 s (`LoraLoad` in `touge/loraload.h`, from build
  41; before it, a cars x cars estimate). Busier than 25 % even at 20 s is
  reported as overloaded, not run over quietly.
- **On a grid.** Each car's sends fall on the interval's multiples of UTC, which
  every car has from its own fix, shifted by its rank among the nodes heard
  (interval / cars apart) plus jitter under half that share. Cars that fell
  into step by chance used to stay in step. Meshtastic's contention delay and
  channel sensing still run on top.
- **When.** Only while a Touge app has said hello since boot and the primary
  channel has a real key. A stock app on a Touge radio, or a radio handed back
  to a default channel, keeps Meshtastic's own position broadcasts. While the
  radio sends, PositionModule's periodic and smart broadcasts stand down
  (core-patches/0011); replies to a position request still go.
- **What.** A standard Meshtastic Position on the first channel sharing
  positions: coordinates, speed, track, source, `time`, and the fix identity
  above, unsigned from build 43 (below). Stock nodes and apps read it as an
  ordinary position.

The LoRa TX queue keeps one position per car (5c, core-patches/0010). It holds
packets already encrypted, so the module notes each position with an identity
as it goes to the router, keyed by origin and packet id. A newer one takes the
older one's place and turn, so the queue goes round the cars; a late older copy
is refused. Text, control and positions without an identity keep the stock
rules. The phone queue (0005) ranks queued positions by the same identity.

Our position goes ahead of relays. Relays queue at Meshtastic's DEFAULT
priority, and up to build 42 ours went at BACKGROUND, as PositionModule's,
behind every one of them; on the build 41 bench it waited a median 0.6 s, p90
4.5 s and up to 6.2 s from queued to on the air. From build 43 it goes at
RELIABLE, ahead of relays and behind texts, admin and acks. That alone would
still leave it waiting out whatever was left of the transmit timer a relay had
started, Meshtastic's SNR-weighted delay (up to 2.2 s on SHORT_FAST at a strong
signal), which a new head of the queue does not otherwise cut short. So when
our position arrives at the head, core-patches/0014 pulls that timer in to our
own contention delay (0-56 ms), and nothing else changes. One position an
interval, so relays still go between ours. Priority never goes on the air. If
our last position is still queued when the next falls due it is counted (`os`),
which should now stay at 0.

### Unsigned positions

Meshtastic 2.8 signs every broadcast with a 64-byte XEdDSA signature, 66 bytes
on the air. A position is 64-67 bytes unsigned and 130-133 signed: 63-66 ms on
SHORT_FAST against 112-114 (111 ms measured on the bench). From build 43 a
Touge radio sends its own position unsigned, on a channel with a real key, while
the module sends positions (core-patches/0015, `sendsUnsigned` in
`touge/lorapos.h`). Everything else stays signed: texts, NodeInfo, reach
summaries, admin, routing. A radio in stock mode (no Touge hello) signs as
stock, and so does a licensed one.

On the way in, Meshtastic's Balanced policy drops an unsigned broadcast from a
node it has seen sign, which a Touge car's signed NodeInfo makes every Touge
car. A Touge radio lets an unsigned position through that one check when it
came on a channel with a real key, and nothing else: every path the check sits
on (the routing gate before relaying, the decode for the phone and modules, the
upgraded-copy and MQTT paths, and the cached verdict between them) goes through
`checkXeddsaReceivePolicy`, where the exception is. Strict still drops unsigned
packets, a malformed or failing signature still drops, and a position on a
public channel keeps Balanced's rule. The radio is never switched to
Compatible. Both ends need build 43: an older Touge radio, or a stock one on
Balanced, drops a build 43 car's positions once it has seen that car sign, and
does not relay them either. A stock rider's radio needs its packet signature
policy set to Compatible to see Touge cars.

## What the radio measures on LoRa

Every packet leaving the TX queue passes Meshtastic's `RadioTxHook` (upstream,
no patch); one whose transmission completed has moved Meshtastic's `txGood`
count first. From that the module counts, by kind, what went on the air and
for how long, and for positions how long they waited in the queue (noted as
they enter it). Every five seconds three reports go to serial as
`touge: lora ll ...` and to the phone as JSON with the same keys
(`touge/kvline.h`). The app logs them as `BASELINE lora ...` and shows them in
Group & radio › Advanced › Link diagnostics. They go out whether or not the
2.4 GHz lane runs.

| report | key | what |
|---|---|---|
| `ll`, this window | `li` | interval the load allows, ms; 0 while not sending |
| | `la` | mean of the last four gaps between our positions on the air |
| | `lx` | longest such gap ending in the window |
| | `cu`, `tu` | channel busy over the last minute; our own TX over the last hour; permille |
| | `ov` | 1: over 25 % busy at 20 s. 2: our position missed its turn |
| | `ow`, `rw` | longest TX-queue wait of our position, of a relayed one, ms |
| | `qm` | TX queue high-water mark |
| | `oh`, `pf` | cars heard on LoRa in 4 min; cars we relay early for |
| `lt`, since boot | `ot`, `oa` | our positions sent, their airtime ms |
| | `rt`, `ra`, `re` | relays sent, airtime, how many went early |
| | `sa`, `xa` | airtime of our reach summaries; of our other traffic |
| | `dr`, `cn` | Meshtastic's TX-queue drops; relays cancelled on hearing another copy |
| | `rp`, `rf`, `os` | 5c replaced, refused; our position late |
| `le`, since boot | `st`, `sh` | reach summaries sent (early ones too), heard |
| | `pg`, `pw` | early-relay grants, withdrawals |
| | `se` | of our summaries, those sent early (build 43) |
| | `sk` | position relays skipped: every other car known hears the origin steadily direct |
| | `rn`, `rs`, `rd` | position relays that went anyway: a known car has claimed nothing, a claim is stale, a car does not hear the origin steadily direct |

## Who reaches whom, and relays chosen on it

Hearing a neighbour relay a packet proves only that it went one hop further.
So each car keeps, per origin, the newest position it heard over LoRa: the fix
sequence, how old the fix was on arrival (UTC from its own fix, good to the
phone's delivery delay, under a second), the hops it had come and whose copy
arrived first (Meshtastic's one-byte `relay_node`). With every twelfth of its
own positions, staggered by rank, it broadcasts that list (`touge/reach.h`).
Meshtastic relays it like any broadcast and hands it to every phone, which logs
each one (`lora reach from a1b2: ...`) and lists them in Link diagnostics; the
radio logs every summary it sends and hears the same way.

A summary is a private-port packet, not Position fields: a relay re-encodes a
packet from the fields it knows, so there is no spare room in a position that
survives relaying, and a stock app shows the fields there are. Summaries stay
signed: a full one is 16 origins, 204 ms on SHORT_FAST against an unsigned
position's 66, and with every twelfth position that is 13-26 % on top of a
car's own LoRa airtime.

A car whose relayed copy of an origin's position a summary names as the one
that reached the reporter first, a hop or more out, fresh (fix under 10 s old
on arrival, reporter heard it within three intervals), while this car still
hears that origin itself, relays that origin's positions early: in the window Meshtastic gives a ROUTER
(core-patches/0012, a hook in `shouldRebroadcastEarlyLikeRouter`), 0-15 slots
against the ordinary 16 and up. The others keep their ordinary delay and drop
their copy on hearing the early one, which is managed flooding as it was; if it
never comes, they relay. A grant lasts two and a half summary periods, renewed
by each summary that says the same; a summary showing worse delivery through
this car ends it at once (`touge/relaypref.h`). Preference only changes who
usually goes first.

From build 43 a relay nobody needs is skipped. Each summary entry also says
whether the car hears that origin steadily direct: its last four positions came
first straight from the origin, none missed, the latest within an interval and
a half and under 10 s old on arrival. Every car keeps the other cars' latest
claims. Before Meshtastic relays a Touge position (core-patches/0013, a hook in
`perhapsRebroadcast`), the car goes through every other car it knows, every
node heard on either radio in the last 10 minutes, and skips the relay only if
each one claims to hear the origin steadily direct in a summary no older than
two and a half summary periods (150 s at 5 s). A car that has claimed nothing
(a stock node, an older build, a car just heard), a stale claim, or a car that
says it does not hear the origin directly keeps the relay going as stock
(`relayVerdict` in `touge/relaypref.h`). Only Touge positions are judged:
texts, NodeInfo, summaries, control and stock positions relay as ever. A 2.4
GHz link is never an input, so a good fast lane cannot turn a LoRa relay off.
On the bench (three cars in direct range) every position was relayed once
(`rt`=92 against `ot`=73 on the V3); in the host test, from the first summaries
on, none are.

A claim can go out of date between summaries, a minute apart at 5 s. So a car
that stops hearing an origin it claimed, quiet for two and a half intervals or
its last two positions first through a relay, says so in an early summary
listing just those origins, a random part of a quarter interval later and never
within an interval of its last one, with one repeat two intervals on if the
origin stays quiet. The cars that were skipping relay that origin's next
position. In the host test a car that drifts out of the front car's range
misses two of its positions and gets the third through the car still in range,
within an interval of its early summary. One to three entries, signed, is
91-107 ms on SHORT_FAST.

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

Because only direct copies count, the reference has to be a car the ride hears
well. From build 40 each car says in every beacon whether it is **fit to keep
time**: a link is solid when, in 6 of the last 8 seconds, it heard that
neighbour's lease beacon and the beacon's slot map showed it back, and a car
is fit when at least half the cars it hears are on solid links. Once fit, a
third keeps it, and either change needs 3 s on end. Among cars equally locked
or unlocked a fit one wins, then the lowest node number. Every car ranks the
flags the candidates advertise themselves, so they agree, and among fit cars
the order never moves. Before build 40 the lowest node number won outright,
which on the bench gave the job to the moto's deliberately weak radio.

A count of how many radios hear each car was tried first and failed in the
host simulation: while a ride powers on or two groups meet, every count climbs
several points a second, each car ranked whoever it heard from last highest,
and parents changed with every beacon, so nobody synced.

### Mixed rides, which is the normal case

Some cars will have a GNSS receiver on the board and some will only have a
phone. That is fine, but it constrains who gets to be the reference, and
getting it wrong is worse than having no schedule:

**The reference must be a car whose own clock is GPS-locked.** Cars with a fix
take the cycle from their pulse; cars without take it from the reference's
beacons. If the reference is itself free-running, those two groups end up on
cycles that have nothing to do with each other, and they collide *every time*
rather than occasionally. So a locked car always wins the job, and fitness,
then the lowest node number, only break the tie. Every beacon carries a flag saying
whether the sender's clock is locked, which is how everyone agrees on the
choice.

**The reference is therefore not always slot zero.** It used to be, back when
it was simply the lowest number. Now a locked car can outrank a lower-numbered
free-running one, so its beacon marks its own slot rather than the cycle start,
and `syncTo` subtracts `referenceSlot` to recover the boundary.

If nobody on the ride has a fix, the lowest-numbered fit car takes it (the
lowest number outright while nobody is fit yet, at power-on) and everyone
follows its beacons, which is consistent because none of them has anything
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
