#!/usr/bin/env python3
"""Cross-platform launcher for the blimp ground-station panels (macOS / Windows / Linux).

QUICK START
  1. pip install -r requirements.txt
  2. Edit config.json  -> your OptiTrack/Motive PC IP + the blimp's rigid-body ID.
  3. In VS Code, open the "Run and Debug" panel and press the green ▶ on one of the
     entries, or from a terminal:  python run.py --mode auto

  The panel opens in your browser (http://127.0.0.1:<port>). The DRONE runs its own
  guidance; this ground station streams the mocap pose and your commands over the
  ESP-NOW USB bridge. No firmware reflash is needed to switch between these modes.

MODES
  auto     THE GO-TO PANEL: flies a path, a point, or a circle on its own   (port 8601)
  manual   hand-fly only, no autonomy                                       (port 8611)
  wander   flies to a point, parks, waits for a hand push, moves on         (port 8613)
  swing    4-motor S-blimp + Mellinger controller — UNTESTED, needs the
           swing firmware first (`python flash.py swing`)                   (port 8620)
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PORTS = {"auto": 8601, "manual": 8611, "wander": 8613, "swing": 8620}


def main():
    mode = "auto"
    if "--mode" in sys.argv:
        try:
            mode = sys.argv[sys.argv.index("--mode") + 1]
        except IndexError:
            pass
    if mode not in PORTS:
        print("unknown --mode %r (use: %s)" % (mode, " | ".join(PORTS)))
        sys.exit(2)

    cfg_path = os.path.join(HERE, "config.json")
    try:
        cfg = json.load(open(cfg_path))
    except Exception as e:
        print("Could not read config.json (%s). Set your Motive IP + body id there." % e)
        sys.exit(1)

    port = PORTS[mode]
    sys.path.insert(0, os.path.join(HERE, "control"))
    print("Launching the %s panel -> http://127.0.0.1:%d   (Motive %s, blimp body %s)"
          % (mode.upper(), port, cfg["motive_ip"], cfg["body_id"]))

    if mode == "swing":
        # The swing/S-blimp build is a SEPARATE server (different controller, different
        # motor layout, different wire meaning) -- see control/swing_panel_server.py.
        sys.argv = ["swing_panel_server.py",
                    "--server", str(cfg["motive_ip"]),
                    "--body", str(cfg["body_id"]),
                    "--up", str(cfg.get("up", "Z")),
                    "--port", str(port)]
        import swing_panel_server
        swing_panel_server.main()
        return

    argv = ["panel_server.py",
            "--server", str(cfg["motive_ip"]),
            "--body", str(cfg["body_id"]),
            "--up", str(cfg.get("up", "Z")),
            "--port", str(port)]
    if mode == "manual":
        argv.append("--manual")
    elif mode == "wander":
        argv.append("--wander")
    sys.argv = argv
    import panel_server
    panel_server.main()


if __name__ == "__main__":
    main()
