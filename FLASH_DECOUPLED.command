#!/bin/bash
# =====================================================================
#  FLASH BACK TO THE DECOUPLED BLIMP (ESP32-S3) — sets BLIMP_SWING=0 in
#  blimp_swing.h, then builds + flashes over USB.
#
#  Undoes FLASH_SWING.command. With BLIMP_SWING=0 both swing branches
#  compile out entirely, so the drone is back to the proven decoupled
#  firmware: on-board guidance, the role-based mixer (2 forward + up +
#  down), and every existing panel — FLY_MOCAP_PANEL.command,
#  BLIMP_PANEL_ESPNOW.command, the MPC/IPOPT/wander panels — all work as
#  before. Whichever controller flag blimp_guidance.c currently has
#  (simple/complex) is left alone.
#
#  Plug ONLY the drone in via USB first (LiPo can stay off).
# =====================================================================
cd "$(dirname "$0")"
SRC="esp-drone/components/core/crazyflie/modules/interface/blimp_swing.h"
echo "=================================================="
echo "   FLASH BLIMP DRONE  —  DECOUPLED build (restore)"
echo "=================================================="
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ No board found on /dev/cu.usbmodem*. Plug the DRONE into USB, then re-run."
  echo; read -p "Press Return to close." x; exit 1
fi
echo "Port: $PORT"
echo "Deselecting the swing mixer (BLIMP_SWING=0)…"
sed -i '' -E 's/#define BLIMP_SWING [01]/#define BLIMP_SWING 0/' "$SRC"
grep -n "define BLIMP_SWING " "$SRC" | head -1
sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 0/' esp-drone/main/main.c
echo "Sourcing ESP-IDF (~/esp/esp-idf)…"
if ! . "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1; then
  echo "❌ ESP-IDF not found at ~/esp/esp-idf/export.sh"; echo; read -p "Press Return to close." x; exit 1
fi
cd esp-drone
echo "Building + flashing DECOUPLED…"
echo
idf.py -p "$PORT" flash
STATUS=$?
echo
if [ $STATUS -eq 0 ]; then
  echo "✅ DRONE FLASHED OK (DECOUPLED build) on $PORT.  Safe to unplug."
  echo "   Your normal panels work again (FLY_MOCAP_PANEL.command, etc)."
else
  echo "❌ Flash FAILED (exit $STATUS)."
  echo "   Try: unplug the LiPo, unplug+replug the USB cable, then flash again."
fi
echo
read -p "Press Return to close." x
