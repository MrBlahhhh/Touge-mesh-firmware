
## Review findings: nine of ten fixed (2026-09-21)

Everything below was found by a second review and has since been fixed, except
the last one. Kept rather than deleted because the reasoning is the useful part
and each fix is only legible against the fault it was for. The commits are
`bccc0bf`, `0065faf`, `475c190`, `2c8f05b` and `f3489ff`.

**Still open: voice has no app-side transport.** `LoopbackVoiceTransport` is
what is wired. The rest of the stack is built - `Intercom`, `JitterBuffer`,
`VoicePacket`, the `VoiceTransport` interface - and the firmware carries voice
in both directions: out through `handleReceived`, back in through `inject`'s
generic branch on the private port. What is missing is a `MeshVoiceTransport`
implementing `send` as a private-port packet addressed to the radio itself, and
`onReceive` hooked into `MeshtasticLink.drain`. That drain already parses the
private port as JSON, so voice needs a discriminator; JSON always opens with a
brace, so a payload that does not is audio.

One prerequisite for it turned out to be broken and is now fixed:
`MeshService::handleToRadio` zeroes `from` on everything the phone sends, and
only `Router::send` fills it in - which a locally-delivered packet never
reaches. The module compared `mp.from` against its own node number, so the test
failed and push-to-talk audio would never have been transmitted at all. It uses
`isFromUs` now, which is what that helper is for.

Also deliberately not done: a real fix time on the wire. With the phone now
telling the radio where it is once a second, stamping `mp.time` with the receive
time is approximately true rather than twenty seconds of fiction. Still the
honest answer, no longer urgent.

## The findings as they were written



Ranked. Everything above this line in the "order of work" still stands; these
are the specific faults a second review found, with the evidence.

**Positions are up to 20 s old and stamped as fresh.** A phone-fed board's
`localPosition` is only written by the phone's broadcast position packet, sent
at `Convoy.pingIntervalMs`, which at 28 cars is pinned to the 20 s
`MAX_MESH_FLOOR_MS` cap. `beacon()` sees no movement and re-sends the same fix
every second, so the 250 ms cycle is optimising delivery of a 20 s old
position. Worse, the wire format carries no fix time: `inject()` stamps
`mp.time` with the *receive* time, so the app's age column reads "now" and the
LoRa-precedence STOP then blocks the fresher LoRa copy as "older". Fix: have
the app write a local-only position (to = own node, hop_limit 0, via
`sendLocal`) at 1 Hz independent of the LoRa pacing, put a fix time in the
frame, and publish `heard` to the UI from `drain()` rather than only from the
exchange loop.

**The radio-to-phone BLE path cannot carry 28 cars.** `inject()` allocates one
MeshPacket per position, plus one per voice frame, plus the 5 s status. That is
28-38 packets/s idle before voice. `MeshService::sendToPhone` drops non-text
packets when `toPhoneQueue` is full (32 max), and the app drains one GATT read
at a time, sequentially, never requesting a faster connection interval. Once
behind, the newest positions, the fast-lane status and every voice frame are
dropped at the radio - and the app reports a dead lane on a saturated healthy
one.

**The LoRa-precedence STOP also kills LoRa rebroadcast.** `handleReceived`
returns STOP for a LoRa position from any car heard on 2.4 GHz within 3 s, and
`callModules` breaks before RoutingModule, which is where
`perhapsRebroadcast` lives. So a middle car that hears the head over 2.4 GHz
refuses to relay the head's LoRa position, and a tail car beyond both the
head's LoRa range and FAST_HOPS gets nothing on either radio. The existing
comment notices the STOP breaks phone delivery and works around that; it does
not mention the rebroadcast.

**Unslotted forwards saturate the channel.** Every hearer defers a forward with
0-15 ms jitter, but copies only count once drained at the 5 ms cadence, so the
first wave - about a third of hearers - transmits having seen only the
original and SUPPRESS_AFTER cannot stop it. A 28-car car park is ~9 forwards
per frame, roughly 0.85 s of air per second from positions alone. One talker at
16.7 frames/s adds ~1.5 s/s after forwarding. Slots are meaningless while
anyone talks.

**The reference is per observer.** Each car elects the lowest node number *it
can hear*, so over half a mile several cars are simultaneously "the reference"
and each may hop independently. A tail group that loses the head goes lost
together, sweeps in lockstep, meets on the first candidate and stops there -
neither group is ever lost again, so nothing reconverges them.

**Slot ownership outlives the car by ten minutes.** `rebuild` counts a roster
entry's claimed slot regardless of age, and the roster holds a car for
`RIDER_DROP_MS`. Five cars that leave hold five of nine slots for ten minutes.
And `syncChannel` rebuilds against an empty roster, so every board claims slot
0 at boot: 28 cars powering up together take about nine beacon rounds to
resolve, colliding on the low slots throughout.

**Transmit failures are invisible.** `transmit()`'s return is discarded,
`sendDeferred` ignores `send()`, and there is no `esp_now_register_send_cb`, so
`ESP_ERR_ESPNOW_NO_MEM` under saturation is silent. The status line counts
receive drops and nothing else: a board that has stopped getting frames out
looks healthy to itself.

**`rider.chan` is stamped at drain time, not receive time.** `onRecv` records
`rxMs` but not the channel, so a retune inside `drainRadio` leaves frames
received on the old channel stamped with the new one - which then makes
`countOn(new)` count phantom cars.

**Dedupe TTL is fiction at 28 cars.** 64 entries against 40-55 distinct
frames/s evicts the oldest every 1.2-1.6 s, so nothing reaches `SEEN_TTL_MS`.
No harm today, since forwards settle inside ~20 ms, but
`test_dedupe_forgets_after_the_window` documents a window that does not exist
under load.

**Voice has no app-side transport.** `LoopbackVoiceTransport` is what is wired;
nothing implements `VoiceTransport` over the mesh. The firmware voice path is
unexercised, and the two findings above land the moment it is wired.

**Asserts worth adding:** `HOP_WINDOW_MS % GATE_IDLE_MS == 0`, a burst bound on
`FORWARD_SLOTS`, and a measured `FRAME_AIRTIME_MS` (the current 8 ms is
arithmetic and excludes the LR preamble).

**App:** `Convoy.staleAfterMs` at 28 cars is 100 s, so a car that left the fast
lane reads fresh for over a minute and a half. `carsWithinBudget` is logged and
displayed but nothing acts on it.
 supplies the
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
