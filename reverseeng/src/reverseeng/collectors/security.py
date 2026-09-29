"""Lock / fuse risk diagnosis. Reads only; never programs OTP or changes lock state.

The point is to warn *before* something irreversible happens: on Raspberry Pi
some config.txt options burn one-time-programmable (OTP) bits on the next
boot, which can never be undone.
"""

from __future__ import annotations

from ..source import Source

# config.txt options that program OTP on the next boot.
OTP_PROGRAMMING_OPTIONS = {
    "program_pubkey": "Writes the customer public key hash to OTP; with SIGNED_BOOT this permanently requires signed boot images.",
    "revoke_devkey": "Permanently revokes the ROM development key in OTP.",
    "program_rpiboot_gpio": "Permanently assigns a GPIO for rpiboot mode in OTP.",
    "program_usb_boot_mode": "Permanently enables USB boot mode in OTP (older boards).",
}


def _truthy(value: str) -> bool:
    return value.strip() not in ("", "0")


def collect_security(src: Source, profile: dict) -> dict:
    risks = []
    bootloader_cfg = (profile.get("bootloader") or {}).get("config") or {}

    state, evidence = "unknown", None
    signed = src.read_dt_u32("/proc/device-tree/chosen/bootloader/signed")
    if signed is not None:
        state = "enabled" if signed else "disabled"
        evidence = f"device-tree chosen/bootloader/signed=0x{signed:x}"
    elif "SIGNED_BOOT" in bootloader_cfg:
        state = "enabled" if _truthy(bootloader_cfg["SIGNED_BOOT"]) else "disabled"
        evidence = f"EEPROM config SIGNED_BOOT={bootloader_cfg['SIGNED_BOOT']}"

    boot_cfg = profile.get("boot_config") or {}
    for entry in boot_cfg.get("entries", []):
        key = entry["key"]
        if key in OTP_PROGRAMMING_OPTIONS and _truthy(entry["value"]):
            risks.append({
                "id": f"otp.{key}",
                "severity": "critical",
                "irreversible": True,
                "message": f"{key}={entry['value']} is set [{entry['section']}]. "
                           f"{OTP_PROGRAMMING_OPTIONS[key]} Remove it before rebooting unless this is intended.",
                "source": boot_cfg.get("path"),
            })

    if state == "enabled":
        risks.append({
            "id": "secure_boot.enabled",
            "severity": "warning",
            "irreversible": False,
            "message": "Secure boot is active: only signed boot images will load. Firmware changes must go through the official signing flow.",
            "source": evidence,
        })

    return {"secure_boot": {"state": state, "evidence": evidence}, "lock_risks": risks}
