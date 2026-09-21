# 2.4 GHz lane: redesign for 28 cars over half a mile


Stated 2026-09-20 late: rides are **up to 28 cars spread over half a mile in
the mountains**. Everything built and tested that day assumed a bench group of
three and a design ceiling of eight.

## Why the current design cannot get there

- `MAX_RIDERS = 8` (mesh.h) and `MAX_SLOTS = MAX_RIDERS + 1`. The roster is a
  fixed array. Thirty cars do not fit in memory, never mind on air.
- One 250 ms cycle of 9 slots, assigned per cycle. Only 9 cars can ever hold
  distinct slots; the rest free-run and TDMA stops being TDMA at a third of
  the group.
- Clock sync comes only from beacons heard **directly** from one reference car.
  Over half a mile of mountain most cars cannot hear it. The machinery is
  unavailable precisely where hidden nodes make it necessary.
- Blind two-hop flooding. Thirty cars at 1 Hz plus rebroadcast is a large
  share of an ESP-NOW LR channel (250 kbps, ~8 ms per 250 byte frame) before a
  single voice talker.

## Agreed target architecture

Verified against the code where it makes claims about it.

**One epoch across the connected ride, not per-neighbourhood clocks.** GPS PPS
where available. Without PPS, propagate a root clock through a sync tree:
root id, generation, hop distance from root, timing quality. Every node in the
component shares the frame epoch. (Per-neighbourhood clocks were considered
and rejected: adjacent clusters on different time bases cannot reuse slots
against each other, and a relay in both would carry two schedules.)

**Two-hop slot colouring for spatial reuse.** A slot must be unique within a
node's two-hop interference neighbourhood: if A and C both reach B but not
each other, they must not share a slot or they collide at B. Beyond two hops a
slot may be reused, so front and tail transmit simultaneously.

**`FRAME_ROSTER` (type 5, defined in frame.h, currently unused) becomes
neighbour gossip:** direct neighbours, RSSI, link age, claimed slot, selected
relay status. Each node colours its own two-hop graph from that without
knowing all twenty-eight riders.

**Selected multipoint relays instead of flooding** (OLSR-style): pick a small
relay set covering every two-hop neighbour, forward each packet once through
it, keep two candidate relays where possible, recompute on link loss, use
sequence numbers and route age rather than a fixed hop ceiling.

**The timing root is never the data hub.** It supplies the clock only; traffic
follows the relay graph, so losing the root does not disconnect front from
tail. On root loss, hold the current epoch and keep transmitting while
electing a replacement; finish after ~3 missed 1 Hz control packets.

  Correction to the brief as received: the current code already holds the
  epoch and keeps transmitting on root loss (`haveEpoch_` stays true; only the
  vote changes after `REFERENCE_LAPSE_MS` = 8 s). Nobody stops for eight
  seconds. Tightening 8 s to ~3 s is a constant, not a redesign.

**Hybrid voice MAC:** reserved TDMA slots for active voice sources and their
selected relays; a small contention window for joining, topology repair,
emergency control and the 1 Hz positions. Superframe matched to the existing
60 ms packetisation. Capacity assigned only to active talkers, not all 30.
One PTT talker first; two to four overlapping later is realistic. Keep the
previous-packet repeat initially - it doubles voice airtime but recovers loss.

**Per-rider failover** (this was item 3 of the earlier review, deferred):
prefer a fresh 2.4 GHz position; after three missed 1 Hz updates accept LoRa;
if LoRa ages out accept cell; switch back the moment fresher 2.4 GHz traffic
returns. Independently per rider, not the group-wide `usingCellBackup` switch
that exists today.

**Voice stays 2.4 GHz only.** The polling server cannot carry conversation
audio and LoRa cannot carry 12.2 kbit/s speech.

## Order of work

1. **A host build that runs.** 74 firmware tests exist and have never executed:
   no host C++ compiler on this machine. Nothing below can be simulated
   without it. First thing.
2. **`FRAME_ROSTER` neighbour gossip**, then **long-chain simulation tests**
   (28 nodes in a line, partial visibility, hidden relays). This supplies the
   two-hop conflict graph; without it neighbourhood TDMA still collides at
   hidden relay nodes.
3. **Measure real ESP-NOW LR airtime and send-completion latency** on the bench
   with two boards before fixing any slot width. The 27.8 ms slot and the 8 ms
   frame estimate are both unmeasured.
4. Sync tree and epoch propagation.
5. Two-hop slot colouring.
6. Selected relays replacing the flood.
7. Voice superframe.
8. Per-rider failover in the app.

This is weeks of protocol work, not an evening. The nine-slot scheduler was a
useful prototype and proved the lane carries traffic between boards; it is
not the thing that ships for twenty-eight riders.

Related: [[gnss-only-buys-pps-timing]] (PPS makes the sync tree unnecessary
where present), [[hub-election-ignores-position]] (superseded by the sync tree
and relay selection), [[pending-firmware-flash]].
