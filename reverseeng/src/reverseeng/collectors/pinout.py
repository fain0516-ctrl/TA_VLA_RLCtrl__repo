from __future__ import annotations

import re

from ..source import Source

# Raspberry Pi 40-pin header: physical pin -> BCM GPIO.
HEADER_40PIN = {
    3: 2, 5: 3, 7: 4, 8: 14, 10: 15, 11: 17, 12: 18, 13: 27, 15: 22, 16: 23,
    18: 24, 19: 10, 21: 9, 22: 25, 23: 11, 24: 8, 26: 7, 27: 0, 28: 1, 29: 5,
    31: 6, 32: 12, 33: 13, 35: 19, 36: 16, 37: 26, 38: 20, 40: 21,
}
GPIO_TO_HEADER = {g: p for p, g in HEADER_40PIN.items()}

# e.g. "14: a4    pn | hi // TXD0/GPIO14 = TXD0"; outputs add a drive level: "17: op dh pd | ..."
PINCTRL_LINE = re.compile(
    r"^\s*(?P<gpio>\d+):\s+(?P<mode>\S+)(?:\s+d[hl])?\s+(?P<pull>pu|pd|pn)?\s*\|\s*(?P<level>hi|lo|--)\s*//\s*\S+\s*=\s*(?P<func>\S+)"
)
PULL = {"pu": "up", "pd": "down", "pn": "none"}
LEVEL = {"hi": 1, "lo": 0}


def parse_pinctrl(text: str) -> list[dict]:
    pins = []
    for line in text.splitlines():
        m = PINCTRL_LINE.match(line)
        if not m:
            continue
        gpio = int(m.group("gpio"))
        pins.append({
            "gpio": gpio,
            "header_pin": GPIO_TO_HEADER.get(gpio),
            "function": m.group("func"),
            "mode": m.group("mode"),
            "pull": PULL.get(m.group("pull") or ""),
            "level": LEVEL.get(m.group("level")),
        })
    return pins


def collect_pinout(src: Source) -> dict:
    out = src.run("pinctrl_get")
    if out:
        pins = parse_pinctrl(out)
        if pins:
            return {"source": "pinctrl", "pins": pins}
    # No live read (offline mode, or pinctrl missing): header map only.
    return {
        "source": "static",
        "pins": [{"gpio": g, "header_pin": p} for p, g in sorted(HEADER_40PIN.items())],
    }
