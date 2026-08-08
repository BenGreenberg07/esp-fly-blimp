#!/bin/bash
# =====================================================================
#  FLASH THE SWING BLIMP BUILD (ESP32-S3) — sets BLIMP_SWING=1 in
#  blimp_swing.h, then builds + flashes over USB.
#
#  In this build the drone does NO guidance. It becomes a dumb 4-channel
#  motor amplifier: the four floats in the 0xA5 frame go straight to
#  M1..M4, and the Mac runs the Mellinger controller + airframe mixer
#  (FLY_SWING_MELLINGER.command).
#
#  The C6 BRIDGE DOES NOT NEED REFLASHING — the 0xA5 frame length is
#  unchanged, only the drone's interpretation of it differs.
#
#  While the drone is flashed SWING, the decoupled blimp panels will NOT
#  fly it correctly. Run FLASH_DECOUPLED.command to go back.
#
#  Plug ONLY the drone in via USB first (LiPo can stay off).
# =====================================================================
cd "$(dirname "$0")"
SRC="esp-drone/components/core/crazyflie/modules/interface/blimp_swing.h"
echo "=================================================="
echo "   FLASH BLIMP DRONE  —  SWING build (4 canted motors)"
echo "=================================================="
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ No board found on /dev/cu.usbmodem*. Plug the DRONE into USB, then re-run."
  echo; read -p "Press Return to close." x; exit 1
fi
echo "Port: $PORT"
echo "Selecting SWING mixer (BLIMP_SWING=1)…"
sed -i '' -E 's/#define BLIMP_SWING [01]/#define BLIMP_SWING 1/' "$SRC"
grep -n "define BLIMP_SWING " "$SRC" | head -1
# Safety net: force the LED bench-test flag off, in case FLASH_LED_TEST's monitor
# wasn't exited cleanly (Ctrl+]) and left it set to 1.
sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 0/' esp-drone/main/main.c
echo "Sourcing ESP-IDF (~/esp/esp-idf)…"
if ! . "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1; then
  echo "❌ ESP-IDF not found at ~/esp/esp-idf/export.sh"; echo; read -p "Press Return to close." x; exit 1
fi
cd esp-drone
echo "Building + flashing SWING… (first build can take a few minutes)"
echo
idf.py -p "$PORT" flash
STATUS=$?
echo
if [ $STATUS -eq 0 ]; then
  echo "✅ DRONE FLASHED OK (SWING build) on $PORT.  Safe to unplug."
  echo "   Next: FLY_SWING_MELLINGER.command  →  ARM  →  bench-test M1..M4 PROPS OFF."
else
  echo "❌ Flash FAILED (exit $STATUS)."
  echo "   Try: unplug the LiPo, unplug+replug the USB cable, then flash again."
fi
echo
read -p "Press Return to close." x
