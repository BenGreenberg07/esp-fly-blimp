# ESP-FLY clone — Flight Controller PCB schematic design

_Goal: recreate the ESP-FLY custom flight-controller board (XIAO ESP32-S3 stacked
on top) so it runs the existing stock firmware unchanged. Board outline = XIAO
footprint, **17.9 × 21.2 mm**, 4-layer. Intended for JLCPCB PCBA._

This is the schematic spec (the electrical design). It gets entered into KiCad
next, then laid out. Pin assignments are taken directly from the firmware
`sdkconfig`, so a board built to this will run the stock firmware with no changes.

---

## 1. Identified components (from the photos + firmware)

| Block | Part | Package | Notes |
|---|---|---|---|
| MCU module | Seeed XIAO ESP32-S3 | castellated module | user-solders on top; not in PCBA BOM |
| IMU | **MPU-6050** | QFN-24 4×4 | I²C0, firmware `mpu6050` driver |
| Motor driver ×4 | **SI2302** (mark "A2SHB") | SOT-23 | N-ch low-side, one per motor |
| Battery conn | JST-PH 2.0, 2-pin | TH | 1S LiPo in |
| Bulk caps | ~22 µF ×2 | 0805 | on VBAT rail (motor surge) |
| Decoupling | 0.1 µF | 0402 | IMU + rails |
| I²C pullups | 10 kΩ ×2 | 0402 | SDA/SCL to 3V3 |
| Gate pulldown ×4 | 10 kΩ | 0402 | MOSFET gate→GND |
| Gate series ×4 | ~100 Ω (opt.) | 0402 | GPIO→gate |
| Status LEDs | R/G/B | 0402/0603 | GPIO driven |
| Power LEDs ×2 | white | 0603 | "headlights", on 3V3 |
| LED resistors | 150 Ω / 200 Ω | 0402 | per LED color |
| Batt-sense divider | 2× (see §5) | 0402 | VBAT→ADC GPIO2 |

---

## 2. Power architecture

```
JST-PH + ──┬──────────────────────────► VBAT (motor + rails, bulk caps)
           │
           └──► XIAO B+ pad (onboard charger + system power)
JST-PH − ─────► GND ◄── XIAO B− pad

XIAO 3V3 OUT pad ──► IMU VDD/VLOGIC, I²C pullups, status-LED commons, batt divider top
VBAT ──► [R divider] ──► GPIO2 (ADC, battery sense)
```
- Motors run off **raw VBAT** (battery voltage), low-side switched.
- IMU + logic run off the **XIAO's 3V3** regulator output (no separate LDO needed —
  the XIAO regulates and also charges the LiPo via its USB-C).
- Bigger bulk capacitance on VBAT is the cheap insurance against the brownout
  you saw; spec ≥ 2×22 µF (room for more).

---

## 3. XIAO pin map — FINAL (clean map, board + firmware made to match)

The original board's wiring is undocumented and its firmware config references
GPIO10/11 which are **not broken out** on a stock XIAO ESP32-S3. Since the clone
will be flashed with our own firmware build, we use a verified map of only the
real exposed pads (GPIO 1-9, 43, 44) and set the firmware `sdkconfig` to match.
All 11 user pads are used.

| XIAO pad | GPIO | Net | Connects to |
|---|---|---|---|
| D0 | GPIO1 | MOT1 | SI2302 #1 gate (via series R) |
| D1 | GPIO2 | MOT2 | SI2302 #2 gate |
| D2 | GPIO3 | MOT3 | SI2302 #3 gate |
| D3 | GPIO4 | MOT4 | SI2302 #4 gate |
| D4 | GPIO5 | I2C0_SDA | MPU-6050 SDA + 10k pullup (hardware SDA pad) |
| D5 | GPIO6 | I2C0_SCL | MPU-6050 SCL + 10k pullup (hardware SCL pad) |
| D8 | GPIO7 | MPU_INT | MPU-6050 INT |
| D9 | GPIO8 | VBAT_SENSE | battery divider midpoint (GPIO8 = ADC1_CH7) |
| D10 | GPIO9 | LED_R | red status LED |
| D6 | GPIO43 | LED_G | green status LED (UART TX pad; USB-CDC used for console) |
| D7 | GPIO44 | LED_B | blue status LED (UART RX pad) |
| 3V3 | — | +3V3 | IMU, pullups, status-LED commons |
| 5V / BAT | — | VBAT | from JST-PH +, motor + rails |
| GND | — | GND | common ground |

**Matching firmware config** (set in `sdkconfig.defaults.esp32s3` so it survives
`set-target`; see §9b): MOTOR01..04 = 1,2,3,4 · I2C0 SDA/SCL = 5,6 ·
MPU_PIN_INT = 7 · ADC1_PIN = 8 · LED RED/GREEN/BLUE = 9,43,44. White power LEDs
go on the 3V3 rail (not a GPIO). **This config is for the NEW boards only — do
not flash it to the original stock drone (its pins differ).**

---

## 4. Motor driver (×4, identical) — low-side brushed

Per motor N (gate = GPIOx from table):
```
 VBAT ──► [ Motor + pad ]
              │   (motor connects across here)
 [ Motor − pad ] ──► SI2302 DRAIN
 SI2302 SOURCE ──► GND
 SI2302 GATE  ──► Rg(100Ω) ──► GPIOx
 GATE ──► Rpd(10kΩ) ──► GND        (holds motor off while XIAO boots / Hi-Z)
```
**Decision: match original — NO discrete flyback diode.** The stock board relies
on the SI2302's avalanche rating + the bulk caps for the tiny 615 motors. We
mirror that. (A SOD-123 Schottky per motor could be added later if desired.)

---

## 5. Battery sense divider (GPIO2 ADC)

VBAT max ≈ 4.2 V; ESP32-S3 ADC reads ~0–3.3 V (with attenuation). Use a divider
that keeps the node under range and matches the firmware's scaling:
```
 VBAT ──[ R_top ]──┬── GPIO2
                   │
                [ R_bot ]
                   │
                  GND
```
Start with **R_top = R_bot = 100 kΩ** (÷2 → 2.1 V at 4.2 V; low drain). The
firmware applies the inverse scale; exact ratio to be confirmed against the
firmware's `pm`/ADC calibration during review.

---

## 6. IMU (MPU-6050)

- VDD + VLOGIC → 3V3, each with 0.1 µF decoupling.
- SDA→GPIO11, SCL→GPIO10, both 10 kΩ pull-ups to 3V3.
- INT→GPIO12. AD0→GND (I²C addr 0x68). REGOUT cap 0.1 µF, CPOUT 2.2 nF per
  datasheet. FSYNC→GND.

---

## 7. LEDs

- **Status (GPIO):** R=GPIO8, G=GPIO9, B=GPIO7, each via its series resistor
  (R/G ~150–200 Ω, B ~150 Ω) to 3V3 (firmware sinks/sources per its LED driver).
- **White "headlight" power LEDs ×2:** on 3V3 through ~150 Ω, always-on power
  indicators (these are the ones that flicker on brownout). _Confirm count/colors
  from the physical board during review._

---

## 8. BOM (PCBA candidates — VERIFY LCSC stock before ordering)

| Ref | Qty | Value/Part | Pkg | LCSC (verify) |
|---|---|---|---|---|
| Q1–Q4 | 4 | SI2302 N-MOSFET | SOT-23 | C10487 |
| U1 | 1 | MPU-6050 | QFN-24 | C24112 |
| C_bulk | 2 | 22 µF 10 V | 0805 | C45783 |
| C_dec | ~5 | 0.1 µF | 0402 | C1525 |
| C_cp | 1 | 2.2 nF | 0402 | C1556 |
| R_pu, R_pd | 6 | 10 kΩ | 0402 | C25744 |
| R_g | 4 | 100 Ω | 0402 | C25076 |
| R_led | ~5 | 150/200 Ω | 0402 | C25092 / C25087 |
| D_led | 5 | R/G/B + white | 0402/0603 | TBD |
| J_bat | 1 | JST-PH 2.0 2P | TH | C173752 |
| (module) | 1 | XIAO ESP32-S3 | — | user-soldered, not PCBA |

LCSC numbers are typical matches and **must be confirmed** against current
JLCPCB stock (and Basic vs Extended fee) before placing the order.

---

## 9. Open items to confirm at schematic review
1. Flyback diodes: include per motor (recommended) or match stock (omit)?
2. Exact LED count/colors and whether white LEDs are on 3V3 or VBAT.
3. Battery divider ratio vs. firmware ADC scaling.
4. Whether to place the JST connector via PCBA or hand-solder.
5. XIAO mounting: castellated solder pads vs. surface pads on top layer.

## 10. Next steps
1. (this doc) schematic design → your review.
2. Build the KiCad project: XIAO from your OPL library + these blocks, generate
   netlist.
3. 4-layer layout to the 17.9 × 21.2 mm outline matching the photo placement.
4. Pre-order verification pass (footprints, LCSC stock, DRC) before PCBA.
