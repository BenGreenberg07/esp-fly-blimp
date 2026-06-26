# ESP-FLY clone PCB — state & how to finish

## What's done (in this folder)
- `SCHEMATIC_DESIGN.md` — full electrical design + verified pin map.
- `gen_netlist.py` → `esp_fly_clone.net` — KiCad netlist, **ERC 0 errors**, XIAO
  pad mapping verified.
- `esp_fly_clone_BOM.csv` — parts + LCSC numbers for JLCPCB.
- `clone_firmware_pins.defaults` — firmware config matching the board pin map.
- `kicad/esp_fly_clone.kicad_pcb` — **generated board**: 4-layer stackup, all 36
  footprints loaded with nets applied, 17.9 × 21.2 mm outline. Parts sit in a
  scatter-grid (not yet arranged) — this is a routable starting point.

## How to finish (in the KiCad GUI — needs your eyes)
Open `kicad/esp_fly_clone.kicad_pcb` in KiCad 9, then:
1. **Place** — drag U1 (XIAO) to match the outline, then arrange Q1–Q4 + motor
   pads (J1–J4) at the four corners, U2 (IMU) centered, JST (J5) + bulk caps,
   LEDs at the edges. Use the photos as the placement reference.
2. **Route** — VBAT and GND as thick traces / copper pours (inner layers as
   GND/VBAT planes is the point of going 4-layer); signals on F.Cu/B.Cu. Or
   export a DSN and auto-route with freerouting, then clean up.
3. **DRC** — must be **0 violations** before fab (the 74 now are just the
   unarranged grid + no routing).

## Pre-order checklist (before paying JLCPCB for assembly)
- [ ] DRC clean (0 errors).
- [ ] Every LCSC part in `esp_fly_clone_BOM.csv` is **in stock** on JLCPCB, and
      Basic/Extended fee acceptable.
- [ ] MPU-6050 **pin-1 / rotation** correct vs the assembly layer (CPL).
- [ ] SI2302 pin order (G/S/D) matches the SOT-23 footprint pads.
- [ ] Battery divider ratio sanity-checked vs firmware ADC scaling.
- [ ] XIAO is **DNP** (you solder it on top) — not in the PCBA BOM.
- [ ] Export Gerbers + BOM + CPL (centroid) and review in JLCPCB's previewer.

## Note
Placement + routing were intentionally left for the GUI: they need visual
verification, and this is a board you're paying to assemble — a wrong rotation
or out-of-stock part scraps the whole run. Ask me to walk any step.
