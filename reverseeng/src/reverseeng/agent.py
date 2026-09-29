"""Build a device profile by running every collector against a Source."""

from __future__ import annotations

import re
import socket
from datetime import datetime, timezone

from . import SCHEMA_VERSION, __version__
from .collectors import COLLECTORS, SECURITY
from .source import Source


def device_id_for(profile: dict, fallback: str | None) -> str:
    raw = (profile.get("board") or {}).get("serial") or fallback or "unknown"
    return re.sub(r"[^A-Za-z0-9_-]", "-", raw)[:64]


def build_profile(src: Source, mode: str = "running") -> dict:
    hostname = (src.read_text("/etc/hostname") or "").strip() or (socket.gethostname() if mode == "running" else None)
    profile: dict = {
        "schema_version": SCHEMA_VERSION,
        "device_id": "",
        "collected_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "collection": {"mode": mode, "agent_version": __version__, "root": str(src.root), "hostname": hostname},
        "errors": [],
    }
    for key, fn, offline_ok in COLLECTORS:
        if mode == "offline" and not offline_ok:
            continue
        try:
            profile[key] = fn(src)
        except Exception as exc:  # one collector failing must not lose the rest
            profile["errors"].append({"collector": key, "message": f"{type(exc).__name__}: {exc}"})
    try:
        profile["security"] = SECURITY(src, profile)
    except Exception as exc:
        profile["errors"].append({"collector": "security", "message": f"{type(exc).__name__}: {exc}"})

    profile["device_id"] = device_id_for(profile, hostname)
    return profile
