"""Live status sampled on the device and streamed over MQTT (not retained)."""

from __future__ import annotations

import re
import time

from .source import Source


def sample(src: Source) -> dict:
    temp = src.read_text("/sys/class/thermal/thermal_zone0/temp")
    load = (src.read_text("/proc/loadavg") or "").split()
    meminfo = src.read_text("/proc/meminfo") or ""
    avail = re.search(r"^MemAvailable:\s+(\d+) kB", meminfo, re.M)
    uptime = (src.read_text("/proc/uptime") or "").split()
    throttled = src.run("get_throttled")  # "throttled=0x0"
    return {
        "ts": time.time(),
        "cpu_temp_c": int(temp) / 1000 if temp and temp.strip().isdigit() else None,
        "load_1m": float(load[0]) if load else None,
        "mem_available_bytes": int(avail.group(1)) * 1024 if avail else None,
        "uptime_s": float(uptime[0]) if uptime else None,
        "throttled": throttled.strip().split("=", 1)[-1] if throttled else None,
    }
