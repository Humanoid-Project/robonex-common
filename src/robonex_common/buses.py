import json
import os
from functools import lru_cache
from pathlib import Path

BUS_MAP_ENV = "ROBONEX_BUS_MAP"
BUS_MAP_FILE = Path.home() / ".config" / "robonex" / "bus_map.json"
GROUPS = ("left_leg", "right_leg", "left_arm", "right_arm", "head")
DEFAULT_BUS_MAP = {
    "left_leg": "can0",
    "right_leg": "can1",
    "left_arm": "can2",
    "right_arm": "can3",
    "head": "can4",
}


def bus_map_path():
    configured = os.environ.get(BUS_MAP_ENV)
    return Path(configured).expanduser() if configured else BUS_MAP_FILE


@lru_cache(maxsize=None)
def bus_map():
    mapping = dict(DEFAULT_BUS_MAP)
    path = bus_map_path()
    if path.is_file():
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
        unknown = set(loaded) - set(GROUPS)
        if unknown:
            raise ValueError(f"{path}: unknown motor groups {sorted(unknown)}; known: {GROUPS}")
        for group, channel in loaded.items():
            if not isinstance(channel, str) or not channel:
                raise ValueError(f"{path}: channel for {group} must be a non-empty string")
            mapping[group] = channel
    return mapping


def channel_for_group(group, mapping=None):
    mapping = bus_map() if mapping is None else mapping
    try:
        return mapping[group]
    except KeyError:
        raise ValueError(f"No CAN channel for motor group {group!r}") from None
