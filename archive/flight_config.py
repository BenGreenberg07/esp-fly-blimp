"""Shared flight configuration (trim + auto-sequence params) for all ESP-FLY
scripts. Stored as flight_config.json next to this file so the manual script,
the auto script, and the web control panel all read/write the same values.

roll_trim / pitch_trim are degrees ADDED to every setpoint. If the drone
drifts right when you command straight up, set roll_trim slightly negative
(and vice-versa); pitch_trim positive if it drifts backward.
"""

import json
import os

_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(_DIR, "flight_config.json")

DEFAULTS = {
    "uri": "udp://192.168.43.42:2390",
    "roll_trim": 0.0,       # deg, +right
    "pitch_trim": 0.0,      # deg, +forward
    "thrust_climb": 43000,  # auto: ramp-up target
    "thrust_hover": 39000,  # auto: hover throttle (tune first)
    "climb_time": 1.5,      # auto: seconds ramping up
    "hover_time": 3.0,      # auto: seconds hovering
    "land_time": 2.0,       # auto: seconds ramping down
}


def load():
    cfg = dict(DEFAULTS)
    try:
        with open(CONFIG_PATH) as f:
            cfg.update(json.load(f))
    except FileNotFoundError:
        pass
    return cfg


def save(cfg):
    merged = dict(DEFAULTS)
    merged.update(cfg)
    with open(CONFIG_PATH, "w") as f:
        json.dump(merged, f, indent=2)
    return merged
