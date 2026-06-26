# ESP-FLY board — continuity trace worksheet

Goal: capture the **real** wiring so I can design a pin-identical clone. Fill in
the blanks and send it back. ~20 minutes with a multimeter.

## Setup
- Multimeter in **continuity / beep** mode (the 🔊 or ⤬ symbol).
- **Battery UNPLUGGED.** XIAO can stay soldered on.
- You'll touch one probe to a **XIAO pad** and the other to a **component pin**;
  a beep = they're connected.

## Identifying the XIAO pads
Use the labels printed on the **XIAO module itself**: one edge is
`D0 D1 D2 D3 D4 D5 D6`, the other edge `D7 D8 D9 D10 3V3 GND 5V`, plus two
pads on the **underside** marked `+`/`BAT` and `-` (battery). Refer to pads by
those labels (D0–D10, 3V3, GND, 5V, BAT). You don't need GPIO numbers — I'll
convert.

---

## Part A — Power rails (find which pad is which)
Touch one probe to the named XIAO pad, hunt for what it connects to.

| Question | Answer |
|---|---|
| Does **5V** (or **BAT +**) connect to the **battery JST + pin**? | yes / no |
| Does it also connect to the **motor "+" pads**? | yes / no |
| Which pad powers the **IMU VDD** — `3V3` or `5V`? | ______ |
| Confirm **GND** ↔ battery JST − pin ↔ IMU GND ↔ MOSFET source pins | yes / no |

---

## Part B — Motors (the key mapping)
Each motor has one MOSFET (SOT-23, marked "A2SHB"). One of its 3 pins goes to a
**XIAO pad** (the control/gate pin), one to a **motor pad**, one to **GND**.
For each MOSFET, probe its pins against the XIAO D-pads until one beeps.

| Motor (corner / label on board) | XIAO pad that beeps to its MOSFET control pin |
|---|---|
| Motor A (note silk label e.g. M1) | D____ |
| Motor B | D____ |
| Motor C | D____ |
| Motor D | D____ |

(If a small resistor sits between the MOSFET and the XIAO, probe the XIAO side of it.)

---

## Part C — IMU (MPU-6050, the QFN labeled "IMU")
Probe XIAO pads against the IMU's I²C/INT pins (or against the nearest pullup
resistors, which are easier to touch).

| Signal | XIAO pad |
|---|---|
| IMU **SDA** | D____ |
| IMU **SCL** | D____ |
| IMU **INT** | D____ |
| IMU **AD0** → GND? (sets I²C address) | yes / no |

---

## Part D — LEDs
For each LED, find the XIAO pad (or rail) on the non-resistor side.

| LED (location + color) | Connects to XIAO pad, or "3V3 rail"? |
|---|---|
| LED 1 (____ color) | ______ |
| LED 2 (____ color) | ______ |
| LED 3 (____ color) | ______ |
| LED 4 (____ color) | ______ |

(The white "headlight" LEDs are probably wired to a rail, not a pad — note which.)

---

## Part E — Battery voltage sense
There's a 2-resistor divider from the battery rail to a XIAO pad (the ADC).

| Question | Answer |
|---|---|
| Which XIAO pad connects to the divider midpoint (ADC in)? | D____ |
| The two divider resistor values (read the codes) | ____ / ____ |

---

## Part F — anything else you notice
- Buzzer? extra connector? test points? → note pad + what it is: ______

---

### Tips
- Beep is bidirectional — pad↔pin order doesn't matter.
- If multiple pads beep to one point, they're shorted (likely a rail) — note all.
- Photograph the filled sheet or just type the answers back to me.

Once I have this, I'll lock the schematic to the real map and start the KiCad layout.
