#!/bin/bash
# =====================================================================
#  LED BENCH TEST (ESP32-S3) — sets LED_BENCH_TEST=1 in main.c, builds +
#  flashes a standalone build that skips ALL flight code and just lets
#  you drive the 3 known-controllable LED GPIOs interactively over
#  serial. Opens the monitor automatically — type a command + Enter:
#     0 = BLUE only   1 = RED only   2 = GREEN only
#     a = ALL on      x = ALL off
#  Watch the physical board while you type. Every command also prints
#  the exact GPIO number it drove, so you can tell us which physical
#  LED (if any) responded and to which pin.
#
#  NOTE: this only controls the 3 small LEDs already wired into the
#  firmware (GPIO 9/43/11). If the brighter red/white LEDs near the
#  battery connector don't react to ANY command here, that's the
#  answer — they're most likely wired straight to the battery charge
#  IC, not the ESP32, and no firmware can ever control them.
#
#  Press Ctrl+] to exit the monitor when done. This script then
#  AUTOMATICALLY reverts LED_BENCH_TEST back to 0 in main.c so you
#  don't accidentally fly with the test build — but the drone still has
#  the test firmware on it until you flash again. Run FLASH_DRONE.command
#  afterward before actually flying.
#
#  Plug ONLY the drone in via USB first (LiPo can stay off for this).
# =====================================================================
cd "$(dirname "$0")"
SRC="esp-drone/main/main.c"
echo "=================================================="
echo "   LED BENCH TEST  —  interactive LED control"
echo "=================================================="
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ No board found on /dev/cu.usbmodem*. Plug the DRONE into USB, then re-run."
  echo; read -p "Press Return to close." x; exit 1
fi
echo "Port: $PORT"
echo "Enabling LED bench test (LED_BENCH_TEST=1)…"
sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 1/' "$SRC"
grep -n "define LED_BENCH_TEST " "$SRC" | head -1
echo "Sourcing ESP-IDF (~/esp/esp-idf)…"
if ! . "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1; then
  echo "❌ ESP-IDF not found at ~/esp/esp-idf/export.sh"; echo; read -p "Press Return to close." x; exit 1
fi
cd esp-drone
echo "Building + flashing LED bench test… (first build can take a few minutes)"
echo
idf.py -p "$PORT" flash
STATUS=$?
cd ..
if [ $STATUS -ne 0 ]; then
  echo "❌ Flash FAILED (exit $STATUS)."
  echo "   Try: unplug the LiPo, unplug+replug the USB cable, then flash again."
  echo "Reverting LED_BENCH_TEST back to 0…"
  sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 0/' "$SRC"
  echo; read -p "Press Return to close." x; exit 1
fi
echo "✅ FLASHED OK. Opening serial monitor — type 0 / 1 / 2 / a / x + Enter."
echo "   Watch the physical LEDs on the board as you type each command."
echo "   Press Ctrl+] to quit the monitor when you're done."
echo
cd esp-drone
idf.py -p "$PORT" monitor
cd ..
echo
echo "Reverting LED_BENCH_TEST back to 0 in main.c (so future flashes are normal flight builds)…"
sed -i '' -E 's/#define LED_BENCH_TEST [01]/#define LED_BENCH_TEST 0/' "$SRC"
grep -n "define LED_BENCH_TEST " "$SRC" | head -1
echo
echo "⚠️  The drone still has the TEST firmware on it right now — it will NOT fly."
echo "   Run FLASH_DRONE.command before your next flight."
echo
read -p "Press Return to close." x
