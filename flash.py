#!/usr/bin/env python3
"""Flash firmware onto the boards. Cross-platform (macOS / Windows / Linux).

You only need this when putting NEW FIRMWARE on a board. Day-to-day flying and all
tuning happen in the panel (`python run.py`) and never require a flash.

USAGE
  python flash.py drone      # normal flight firmware  <- the one you almost always want
  python flash.py bridge     # the XIAO ESP32-C6 USB radio bridge
  python flash.py swing      # the experimental 4-motor S-blimp build (UNTESTED)
  python flash.py led-test   # LED bench test — NO flight code, motors will not spin

  --port /dev/cu.usbmodem101   pick the serial port yourself (default: autodetect)

BEFORE YOU RUN IT
  drone / swing / led-test  need ESP-IDF v5.0.x installed.
  bridge                    needs arduino-cli with the esp32 core 3.x.

  If `idf.py` isn't already on your PATH this script looks for ESP-IDF in $IDF_PATH,
  then ~/esp/esp-idf, and sets it up for you.

Plug in ONLY the board you're flashing — most USB adapters expose one port at a time.
"""
import argparse, glob, os, re, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ESP_DRONE = os.path.join(HERE, "esp-drone")
SWING_H = os.path.join(ESP_DRONE, "components", "core", "crazyflie",
                       "modules", "interface", "blimp_swing.h")
MAIN_C = os.path.join(ESP_DRONE, "main", "main.c")
BRIDGE_INO = os.path.join(HERE, "espnow_bridge", "espnow_bridge.ino")
FQBN = "esp32:esp32:XIAO_ESP32C6"


def find_port():
    """First plausible USB serial port, on any OS."""
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
        pref = [p for p in ports if "usbmodem" in p or "usbserial" in p
                or p.upper().startswith("COM") or "ttyACM" in p or "ttyUSB" in p]
        if pref:
            return pref[0]
        if ports:
            return ports[0]
    except ImportError:
        pass
    for pat in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def set_define(path, name, value):
    """Rewrite `#define <name> 0|1` in a C source file."""
    with open(path) as f:
        src = f.read()
    new, n = re.subn(r"#define\s+%s\s+[01]" % re.escape(name),
                     "#define %s %d" % (name, value), src)
    if n == 0:
        sys.exit("Could not find `#define %s` in %s — has the firmware moved?" % (name, path))
    if new != src:
        with open(path, "w") as f:
            f.write(new)
    print("  %s = %d" % (name, value))


def run_idf(port):
    """Build + flash esp-drone, setting up ESP-IDF if it isn't already on PATH."""
    cmd = "idf.py -p %s flash" % port
    if shutil.which("idf.py"):
        return subprocess.call(cmd, shell=True, cwd=ESP_DRONE)

    idf = os.environ.get("IDF_PATH") or os.path.join(os.path.expanduser("~"), "esp", "esp-idf")
    if os.name == "nt":
        export = os.path.join(idf, "export.bat")
        if not os.path.isfile(export):
            sys.exit("ESP-IDF not found. Install it, or run its export.bat first.\nLooked in: %s" % idf)
        return subprocess.call('call "%s" >nul && %s' % (export, cmd), shell=True, cwd=ESP_DRONE)

    export = os.path.join(idf, "export.sh")
    if not os.path.isfile(export):
        sys.exit("ESP-IDF not found. Install it, or `. ~/esp/esp-idf/export.sh` first.\n"
                 "Looked in: %s" % idf)
    return subprocess.call(['bash', '-c', '. "%s" >/dev/null 2>&1 && %s' % (export, cmd)],
                           cwd=ESP_DRONE)


def main():
    ap = argparse.ArgumentParser(description="Flash the blimp's firmware.")
    ap.add_argument("target", choices=["drone", "bridge", "swing", "led-test"])
    ap.add_argument("--port", default=None, help="serial port (default: autodetect)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Plug the board in via USB, then re-run.\n"
                 "If it is plugged in, pass it explicitly: python flash.py %s --port <PORT>"
                 % args.target)
    print("Port: %s" % port)

    if args.target == "bridge":
        if not shutil.which("arduino-cli"):
            sys.exit("arduino-cli not found. Install it (esp32 core 3.x), or use the Arduino IDE\n"
                     "to open %s and upload to board XIAO_ESP32C6." % BRIDGE_INO)
        # compile and upload together on purpose: `arduino-cli upload` alone re-flashes
        # the LAST COMPILED binary, which silently flashes a stale sketch.
        rc = subprocess.call(["arduino-cli", "compile", "--fqbn", FQBN,
                              "--upload", "-p", port, BRIDGE_INO])
    else:
        print("Selecting the firmware build:")
        set_define(SWING_H, "BLIMP_SWING", 1 if args.target == "swing" else 0)
        # Always force this off for flight builds. The LED bench-test firmware skips
        # ALL flight code, so if it is left enabled the motors simply never spin and
        # it looks like dead hardware.
        set_define(MAIN_C, "LED_BENCH_TEST", 1 if args.target == "led-test" else 0)
        print("Building + flashing (the first build takes a few minutes)…\n")
        rc = run_idf(port)

    print()
    if rc == 0:
        print("Flashed OK on %s." % port)
        if args.target == "led-test":
            print("NOTE: this board now has NO flight code on it. Run `python flash.py drone`\n"
                  "      before trying to fly again.")
        elif args.target == "swing":
            print("NOTE: this is the experimental 4-motor build and has never been flown.\n"
                  "      `python flash.py drone` puts the normal blimp back.")
    else:
        print("Flash FAILED (exit %d)." % rc)
        print("Most common fix: unplug the LiPo, unplug and replug the USB cable, then\n"
              "retry immediately. A failed flash never damages the board — esptool gives\n"
              "up before it writes anything.")
    sys.exit(rc)


if __name__ == "__main__":
    main()
