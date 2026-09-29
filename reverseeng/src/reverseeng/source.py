"""Read-only access to a device's filesystem and a fixed allowlist of commands.

Every collector goes through `Source`, so the agent cannot open a file for
writing or run anything outside READ_ONLY_COMMANDS. Pointing `root` at a
fixture directory (tests) or a mounted SD card (offline mode) needs no other
changes.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

# Commands that only report state. Nothing here programs OTP, flashes the
# EEPROM or changes a pin: `pinctrl get` reads, `rpi-eeprom-config` with no
# arguments prints the current config.
READ_ONLY_COMMANDS: dict[str, tuple[str, ...]] = {
    "pinctrl_get": ("pinctrl", "get"),
    "bootloader_version": ("vcgencmd", "bootloader_version"),
    "eeprom_config": ("rpi-eeprom-config",),
    "get_throttled": ("vcgencmd", "get_throttled"),
}


class Source:
    def __init__(self, root: str | os.PathLike = "/", allow_commands: bool = True):
        self.root = Path(root)
        # Commands run on the host, so they only make sense when root is the
        # live system.
        self.allow_commands = allow_commands and self.root == Path("/")

    def path(self, p: str) -> Path:
        return self.root / p.lstrip("/")

    def read_text(self, p: str) -> str | None:
        try:
            return self.path(p).read_text(errors="replace")
        except OSError:
            return None

    def read_bytes(self, p: str) -> bytes | None:
        try:
            return self.path(p).read_bytes()
        except OSError:
            return None

    def read_dt_string(self, p: str) -> str | None:
        """Device-tree string property: NUL-terminated."""
        raw = self.read_bytes(p)
        if raw is None:
            return None
        return raw.rstrip(b"\0").decode(errors="replace")

    def read_dt_strings(self, p: str) -> list[str]:
        raw = self.read_bytes(p)
        if not raw:
            return []
        return [s.decode(errors="replace") for s in raw.rstrip(b"\0").split(b"\0") if s]

    def read_dt_u32(self, p: str) -> int | None:
        raw = self.read_bytes(p)
        if raw is None or len(raw) < 4:
            return None
        return int.from_bytes(raw[:4], "big")

    def listdir(self, p: str) -> list[str]:
        try:
            return sorted(os.listdir(self.path(p)))
        except OSError:
            return []

    def run(self, name: str, timeout: float = 5.0) -> str | None:
        """Run an allowlisted read-only command by name; None if unavailable."""
        if not self.allow_commands:
            return None
        argv = READ_ONLY_COMMANDS[name]
        if shutil.which(argv[0]) is None:
            return None
        try:
            res = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return res.stdout if res.returncode == 0 else None
