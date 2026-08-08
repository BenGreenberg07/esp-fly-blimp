# ESP-NOW bridge (XIAO ESP32-C6 or XIAO ESP32-S3)

USB↔ESP-NOW relay. The Mac talks to this board over USB serial; the board
rebroadcasts to the blimp over ESP-NOW. This keeps the Mac's Wi-Fi free for the
mocap (OptiTrack/NatNet) network — no router, no Wi-Fi join, no pairing UI.

**Same sketch, same file (`espnow_bridge.ino`), either board.** The bridge can
be a Seeed XIAO ESP32-**C6** or a Seeed XIAO ESP32-**S3** — pick whichever you
have. The only difference is which `arduino-cli --fqbn` you flash with (see
"Flashing" below); nothing in the sketch needs editing either way. (The one
board-specific bit, the C6's RF-antenna-switch pins, is already handled for
you at compile time via `#if defined(CONFIG_IDF_TARGET_ESP32C6)` — an S3
bridge just skips that block and uses its fixed onboard antenna.)

```
Mac (panel) --USB serial--> XIAO ESP32-C6 or -S3 --ESP-NOW--> ESP32-S3 (blimp)
            <--0xB7 telem-- (bridge relays)      <----------- (motor telemetry)
```

---

## "Where do I put the MAC address?"

**Nowhere — and that's deliberate. There is no MAC address to configure.**

The bridge **broadcasts** to `FF:FF:FF:FF:FF:FF`, so it does not need to know the
drone's MAC, and the drone does not need to know the bridge's:

```c
// espnow_bridge.ino
static uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

If you flashed a *new* drone board, or swapped the C6 for another bridge, **you
do not have to change anything.** They will link up as long as the three things
below match.

> **Why broadcast instead of unicast?** An earlier version unicast to the drone's
> STA MAC. That broke once the drone started running ESP-NOW
> on top of its Wi-Fi SoftAP, because its *active* interface MAC became the **AP**
> MAC, not the STA MAC — so every send failed with `ESP_ERR_ESPNOW_IF (0x306c)`,
> silently (receive still worked, so it looked fine). Broadcast is delivered
> regardless of which interface is up, so it just works. Don't "fix" this back to
> unicast unless you have a reason.

---

## What actually pairs the two boards

Three things must match. If the link is dead, check them in this order.

### 1. Wi-Fi channel — must be the same on both

| Side | File | Setting |
|---|---|---|
| Bridge (C6 or S3) | `espnow_bridge/espnow_bridge.ino` | `static const uint8_t ESPNOW_CHANNEL = 1;` |
| Drone (S3) | `esp-drone/components/espnow_control/espnow_control.c` | pinned by the SoftAP, `WIFI_CH = 1` |

The drone runs ESP-NOW *on top of* its SoftAP, which pins the channel. (This also
fixed an old bug where `esp_wifi_set_channel()` wouldn't stick in pure-STA mode.)

### 2. `FRAME_MAGIC` — the real "pairing key"

This is what actually keeps *your* blimp listening to *your* bridge:

```c
// MUST be identical in BOTH files:
//   espnow_bridge/espnow_bridge.ino
//   esp-drone/components/espnow_control/espnow_control.c
static const uint8_t FRAME_MAGIC[4] = { 0xB1, 0x12, 0x9F, 0x5A };
```

Every frame the bridge broadcasts is prefixed with these 4 bytes. The drone
**rejects anything without them**.

This exists because a shared lab is full of *other* ESP-NOW traffic, and the
drone distinguishes frame types by **length**. Foreign frames were being parsed
as our gain/pose frames — garbage gains → NaN → crash ~3 s after power-up, and
garbage poses → the autonomous controller chasing junk. The magic tag fixed it.

**If you want two blimps flying in the same room at once, give each pair its own
`FRAME_MAGIC`** (change it in both that pair's files and reflash both). That is
the supported way to separate them — not MAC addresses. See
["Multiple drones in the same room"](#multiple-drones-in-the-same-room) below
for the full checklist.

### 3. `TELEM_MAGIC` — the return path

Same idea for telemetry coming *back* from the drone:

```c
static const uint8_t TELEM_MAGIC[4] = { 0xB7, 0x1E, 0x30, 0xA5 };
```

The bridge forwards those frames to the Mac over USB as `0xB7 + 16 bytes`
(4 × LE float32: `fwdL, fwdR, up, down`).

---

## Multiple drones in the same room

Each **Mac + bridge (C6 or S3) + drone (S3)** is one triplet. To fly a second blimp
alongside the first, build a second triplet with its own `FRAME_MAGIC` — not a
different MAC address (broadcast doesn't use MACs, see above) and not a
different Wi-Fi channel (leave both on channel 1; the magic tag is what keeps
them apart, and every extra channel is one more thing that has to match).

1. Pick a second 4-byte value, e.g. `{ 0xC2, 0x44, 0x1A, 0x7E }` — anything
   that isn't `{ 0xB1, 0x12, 0x9F, 0x5A }` (triplet 1) works.
2. Set it identically in **both** files for triplet 2, and reflash both boards:
   - `espnow_bridge/espnow_bridge.ino` → `FRAME_MAGIC[4]` (~line 42)
   - `esp-drone/components/espnow_control/espnow_control.c` → `FRAME_MAGIC[4]` (~line 46)
3. Leave `TELEM_MAGIC` alone unless you also need to tell the two triplets'
   *telemetry* apart on the same USB host — for two separate Macs (one bridge
   each) this doesn't matter.
4. Triplet 1's bridge/drone keep the original magic — don't touch those files.

With mismatched magics, each drone only accepts frames from its own bridge (and
ignores the other bridge's broadcasts as "foreign," same as it already ignores
other lab traffic) — so both blimps can be flown from separate Macs/panels in
the same room without cross-talk or reflashing anything each session.

---

## Frame formats (Mac → bridge → drone)

The bridge is a dumb relay: it reads a framed packet on USB serial and
rebroadcasts the payload with `FRAME_MAGIC` prepended. **The drone tells frame
types apart by payload LENGTH**, so these sizes matter:

| USB header | Payload | Meaning |
|---|---|---|
| `0xA5` | 16 B — 4 × float32 `roll, pitch, yaw, thrust` | manual setpoint |
| `0xA6` | 32 B — 8 × float32 `cx,cy,cz,cyaw, tx,ty,tz,tyaw` | mocap pose + goal |
| `0xA7` | 4 × N B — N × float32 gains | live gain update |

If you add or remove a gain, the `0xA7` length changes and **the drone, the
bridge, and the panel must all be updated together** or frames get misparsed.

---

## Flashing

The two boards use **different toolchains** — this trips people up:

```bash
# Bridge — Arduino, NOT idf.py. Same espnow_bridge.ino, board-specific --fqbn only:

# XIAO ESP32-C6 bridge:
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 --upload -p /dev/cu.usbmodemXXXX espnow_bridge/espnow_bridge.ino

# XIAO ESP32-S3 bridge:
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:CDCOnBoot=cdc --upload -p /dev/cu.usbmodemXXXX espnow_bridge/espnow_bridge.ino

# Drone (ESP32-S3) — ESP-IDF
cd esp-drone && idf.py -p /dev/cu.usbmodemXXXX flash
```

**S3 bridge gotcha:** the `CDCOnBoot=cdc` fqbn option (or, in the Arduino IDE,
Tools → **USB CDC On Boot → Enabled**) is required on S3 — without it, `Serial`
doesn't route over the native USB port and the Mac never sees the bridge as a
serial device at all. The C6 doesn't need this option (its default is already
correct). If `arduino-cli board list` doesn't show `XIAO_ESP32S3` as an option,
your installed "esp32 by Espressif" core is too old — update it.

**Gotcha (both boards):** `arduino-cli upload` on its own flashes the *last
compiled* binary — it does **not** recompile. After editing the `.ino`, always
use `compile --upload` or you will silently flash a stale sketch.

The USB adapter typically exposes only **one** `usbmodem` port at a time, so
flash the boards one at a time.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Drone never responds | Channel mismatch, or `FRAME_MAGIC` differs between the two files |
| Bridge counts `rxAll` climbing but `rxTel` stays 0 | It's hearing other lab traffic, not your drone — check the drone is actually transmitting |
| Motors keep spinning after unplugging the bridge | Old sketch with the idle heartbeat — reflash the bridge (current version goes silent, so the drone's failsafe fires) |
| S3 bridge: no serial port shows up at all | `USB CDC On Boot` wasn't enabled at flash time — reflash with `CDCOnBoot=cdc` (or enable it in the IDE), see "Flashing" |
| Sends fail with `0x306c` | `ESP_ERR_ESPNOW_IF` — interface mismatch; make sure you're broadcasting, not unicasting to a stale MAC |
| Link works but nothing moves | Not a link problem — check ARM state and the panel's KILL/watchdog |

The bridge prints a status line with `rxAll` / `rxTel` counters; open the serial
monitor on the bridge board (C6 or S3) to see whether frames are moving at all.
