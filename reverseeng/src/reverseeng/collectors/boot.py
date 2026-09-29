from __future__ import annotations

from ..source import Source

CONFIG_TXT_CANDIDATES = ("/boot/firmware/config.txt", "/boot/config.txt")


def find_config_txt(src: Source) -> str | None:
    return next((p for p in CONFIG_TXT_CANDIDATES if src.path(p).is_file()), None)


def parse_config_txt(text: str) -> list[dict]:
    """config.txt entries with the [section] filter they sit under."""
    entries, section = [], "all"
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
            continue
        key, _, value = line.partition("=")
        entries.append({"key": key.strip(), "value": value.strip(), "section": section})
    return entries


def collect_boot_config(src: Source) -> dict:
    path = find_config_txt(src)
    if path is None:
        raise FileNotFoundError("config.txt not found")
    return {"path": path, "entries": parse_config_txt(src.read_text(path) or "")}


def collect_bootloader(src: Source) -> dict:
    version = None
    out = src.run("bootloader_version")
    if out:
        version = out.strip().splitlines()[0]
    else:
        ts = src.read_dt_u32("/proc/device-tree/chosen/bootloader/build-timestamp")
        version = f"build-timestamp {ts}" if ts is not None else None

    config: dict[str, str] = {}
    for line in (src.run("eeprom_config") or "").splitlines():
        line = line.strip()
        if "=" in line and not line.startswith(("#", "[")):
            k, v = line.split("=", 1)
            config[k.strip()] = v.strip()
    return {"version": version, "config": config}
