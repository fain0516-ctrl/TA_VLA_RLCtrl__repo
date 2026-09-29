from __future__ import annotations

import re

from ..source import Source

# PCIe/USB ids of companion chips worth naming.
KNOWN_PCIE = {("0x1de4", "0x0001"): "RP1"}


def _cpuinfo(src: Source) -> dict[str, str]:
    info: dict[str, str] = {}
    for line in (src.read_text("/proc/cpuinfo") or "").splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            info.setdefault(k.strip(), v.strip())
    return info


def collect_board(src: Source) -> dict:
    cpu = _cpuinfo(src)
    revision = cpu.get("Revision")
    if revision is None:
        rev = src.read_dt_u32("/proc/device-tree/system/linux,revision")
        revision = f"{rev:x}" if rev is not None else None
    serial = src.read_dt_string("/proc/device-tree/serial-number") or cpu.get("Serial")

    mem = None
    m = re.search(r"^MemTotal:\s+(\d+) kB", src.read_text("/proc/meminfo") or "", re.M)
    if m:
        mem = int(m.group(1)) * 1024

    return {
        "model": src.read_dt_string("/proc/device-tree/model") or cpu.get("Model"),
        "revision": revision,
        "serial": serial,
        "compatible": src.read_dt_strings("/proc/device-tree/compatible"),
        "memory_bytes": mem,
    }


def collect_chipset(src: Source) -> dict:
    compatible = src.read_dt_strings("/proc/device-tree/compatible")
    soc = next((c.split(",", 1)[1] for c in compatible if c.startswith("brcm,")), None)

    text = src.read_text("/proc/cpuinfo") or ""
    cores = len(re.findall(r"^processor\s*:", text, re.M)) or None
    part = re.search(r"^CPU part\s*:\s*(\S+)", text, re.M)
    arch = re.search(r"^CPU architecture\s*:\s*(\S+)", text, re.M)

    companions = []
    for addr in src.listdir("/sys/bus/pci/devices"):
        base = f"/sys/bus/pci/devices/{addr}"
        ids = ((src.read_text(f"{base}/vendor") or "").strip(), (src.read_text(f"{base}/device") or "").strip())
        if ids in KNOWN_PCIE:
            companions.append({"name": KNOWN_PCIE[ids], "bus": "pcie", "address": addr})

    return {
        "soc": soc,
        "cpu": {
            "arch": f"armv{arch.group(1)}" if arch else None,
            "cores": cores,
            "part": part.group(1) if part else None,
        },
        "companion_chips": companions,
    }
