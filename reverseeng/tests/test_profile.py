import json

from reverseeng.__main__ import main
from reverseeng.agent import build_profile
from reverseeng.collectors.pinout import parse_pinctrl
from reverseeng.schema import validate_profile
from reverseeng.source import Source


def test_running_profile_is_valid_and_complete(pi5_root):
    profile = build_profile(Source(pi5_root), mode="running")
    assert validate_profile(profile) == []
    assert profile["errors"] == []
    assert profile["device_id"] == "abcdef12"
    assert profile["board"]["model"] == "Raspberry Pi 5 Model B Rev 1.0"
    assert profile["board"]["revision"] == "d04170"
    assert profile["board"]["memory_bytes"] == 8245068 * 1024
    assert profile["chipset"]["soc"] == "bcm2712"
    assert profile["chipset"]["cpu"] == {"arch": "armv8", "cores": 4, "part": "0xd0b"}
    assert profile["chipset"]["companion_chips"] == [{"name": "RP1", "bus": "pcie", "address": "0001:01:00.0"}]
    assert profile["kernel"]["release"] == "6.6.51+rpt-rpi-2712"
    assert profile["kernel"]["device_tree"]["overlays"] == ["vc4-kms-v3d", "i2c-rtc"]
    assert profile["kernel"]["modules"][1] == {"name": "rp1_pio", "size": 20480, "used_by": ["pwm_pio"]}
    parts = {p["name"]: p for p in profile["storage"]["partitions"]}
    assert set(parts) == {"mmcblk0", "mmcblk0p1", "mmcblk0p2"}
    assert parts["mmcblk0p1"]["mountpoint"] == "/boot/firmware"
    assert [d["adapter"] for d in profile["buses"]["i2c"]] == ["i2c-1"]
    assert [d["path"] for d in profile["buses"]["usb"]] == ["1-1"]
    assert profile["pinout"]["source"] == "static"  # no pinctrl off-device
    assert len(profile["pinout"]["pins"]) == 28
    assert profile["security"]["secure_boot"]["state"] == "disabled"
    assert profile["security"]["lock_risks"] == []


def test_offline_mode_skips_live_collectors(pi5_root):
    profile = build_profile(Source(pi5_root), mode="offline")
    assert validate_profile(profile) == []
    assert "board" not in profile and "buses" not in profile
    assert profile["kernel"]["installed_releases"] == ["6.6.51+rpt-rpi-2712"]
    assert profile["boot_config"]["path"] == "/boot/firmware/config.txt"


def test_otp_programming_option_is_flagged_irreversible(pi5_root):
    cfg = pi5_root / "boot/firmware/config.txt"
    cfg.write_text(cfg.read_text() + "program_pubkey=1\nrevoke_devkey=0\n")
    risks = build_profile(Source(pi5_root))["security"]["lock_risks"]
    assert [r["id"] for r in risks] == ["otp.program_pubkey"]
    assert risks[0]["irreversible"] is True and risks[0]["severity"] == "critical"


def test_missing_files_become_errors_not_crashes(tmp_path):
    profile = build_profile(Source(tmp_path))
    assert validate_profile(profile) == []
    assert "boot_config" in {e["collector"] for e in profile["errors"]}


def test_parse_pinctrl():
    out = (" 0: no    pu | -- // ID_SDA/GPIO0 = none\n"
           " 4: ip    pu | hi // GPIO4 = input\n"
           "14: a4    pn | hi // TXD0/GPIO14 = TXD0\n"
           "17: op dh pd | lo // GPIO17 = output\n")
    pins = parse_pinctrl(out)
    assert pins[1] == {"gpio": 4, "header_pin": 7, "function": "input", "mode": "ip", "pull": "up", "level": 1}
    assert pins[2]["header_pin"] == 8 and pins[2]["function"] == "TXD0"
    assert [p["gpio"] for p in pins] == [0, 4, 14, 17]


def test_cli_collect_writes_valid_json(pi5_root, tmp_path):
    out = tmp_path / "profile.json"
    assert main(["collect", "--root", str(pi5_root), "--mode", "running", "-o", str(out)]) == 0
    assert validate_profile(json.loads(out.read_text())) == []
