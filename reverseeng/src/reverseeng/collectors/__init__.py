"""Collectors: each takes a Source and returns one top-level profile section.

A collector raises on failure; the agent records the error and moves on, so
one missing file never loses the rest of the profile.
"""

from . import boot, buses, hardware, kernel, pinout, security, storage

# (profile key, function, runs in offline mode)
COLLECTORS = [
    ("board", hardware.collect_board, False),
    ("chipset", hardware.collect_chipset, False),
    ("bootloader", boot.collect_bootloader, False),
    ("kernel", kernel.collect_kernel, True),
    ("storage", storage.collect_storage, False),
    ("buses", buses.collect_buses, False),
    ("pinout", pinout.collect_pinout, True),
    ("boot_config", boot.collect_boot_config, True),
]

# Security runs last: it reads sections the others produced.
SECURITY = security.collect_security
