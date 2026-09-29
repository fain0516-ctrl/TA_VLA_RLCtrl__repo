from __future__ import annotations

from ..source import Source


def collect_storage(src: Source) -> dict:
    mounts: dict[str, tuple[str, str]] = {}
    for line in (src.read_text("/proc/mounts") or "").splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0].startswith("/dev/"):
            mounts.setdefault(parts[0][5:], (parts[1], parts[2]))

    partitions = []
    for line in (src.read_text("/proc/partitions") or "").splitlines()[2:]:
        parts = line.split()
        if len(parts) != 4 or parts[3].startswith(("ram", "loop", "zram")):
            continue
        mountpoint, fstype = mounts.get(parts[3], (None, None))
        partitions.append({
            "name": parts[3],
            "size_bytes": int(parts[2]) * 1024,  # /proc/partitions counts 1 KiB blocks
            "mountpoint": mountpoint,
            "fstype": fstype,
        })
    return {"partitions": partitions}
