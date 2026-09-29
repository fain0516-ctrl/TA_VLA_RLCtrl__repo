from __future__ import annotations

from ..source import Source
from .boot import parse_config_txt, find_config_txt


def collect_kernel(src: Source) -> dict:
    modules = []
    for line in (src.read_text("/proc/modules") or "").splitlines():
        parts = line.split()
        if len(parts) >= 4:
            used_by = [m for m in parts[3].strip(",").split(",") if m and m != "-"]
            modules.append({"name": parts[0], "size": int(parts[1]), "used_by": used_by})

    config_path = find_config_txt(src)
    overlays = []
    if config_path:
        for e in parse_config_txt(src.read_text(config_path) or ""):
            if e["key"] == "dtoverlay" and e["value"]:
                overlays.append(e["value"].split(",", 1)[0])

    def strip(v):
        return v.strip() if v else None

    return {
        "release": strip(src.read_text("/proc/sys/kernel/osrelease")),
        "version": strip(src.read_text("/proc/version")),
        "cmdline": strip(src.read_text("/proc/cmdline")),
        # Offline mode: kernels present on disk even when nothing is running.
        "installed_releases": src.listdir("/lib/modules"),
        "modules": modules,
        "device_tree": {
            "model": src.read_dt_string("/proc/device-tree/model"),
            "compatible": src.read_dt_strings("/proc/device-tree/compatible"),
            "overlays": overlays,
        },
    }
