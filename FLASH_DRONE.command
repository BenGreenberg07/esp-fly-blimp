#!/bin/bash
# =====================================================================
#  FLASH THE BLIMP DRONE (ESP32-S3) — builds the current firmware and
#  writes it over USB. Triggered by the "Flash drone" button in the
#  On-Board panel (or just double-click this file). Plug ONLY the drone
#  in via USB first (LiPo can stay off).
# =====================================================================
cd "$(dirname "$0")"
echo "=========================================="
echo "   FLASH BLIMP DRONE  (ESP32-S3 firmware)"
echo "=========================================="
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ No board found on /dev/cu.usbmodem*. Plug the DRONE into USB, then re-run."
  echo; read -p "Press Return to close." x; exit 1
fi
echo "Port: $PORT"
# Safety net: this must always produce FLYABLE firmware, so force the LED
# bench-test flag off even if FLASH_LED_TEST.command's monitor wasn't exited
# cleanly (Ctrl+]) last time and left it set to 1 in the source tree.
sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 0/' esp-drone/main/main.c
echo "Sourcing ESP-IDF (~/esp/esp-idf)…"
if ! . "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1; then
  echo "❌ ESP-IDF not found at ~/esp/esp-idf/export.sh"; echo; read -p "Press Return to close." x; exit 1
fi
cd esp-drone
echo "Building + flashing… (first build can take a few minutes)"
echo
idf.py -p "$PORT" flash
STATUS=$?
echo
if [ $STATUS -eq 0 ]; then
  echo "✅ DRONE FLASHED OK on $PORT.  Safe to unplug."
else
  echo "❌ Flash FAILED (exit $STATUS)."
  echo "   Try: unplug the LiPo, unplug+replug the USB cable, then flash again."
fi
echo
read -p "Press Return to close." x
