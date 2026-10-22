# -*- coding: utf-8 -*-
"""Pi <-> Pico 2 보드 링크: 패킷 형식, 펌웨어 쪽 코드와의 일치, UDP 루프백."""
import ctypes
import importlib.util
import os
import shutil
import socket
import subprocess

import pytest

from hand_motion_control import pico_link as L
from hand_motion_control import sts_protocol as S
from hand_motion_control.backends import PicoBackend
from hand_motion_control.pca9685 import ServoPulse

FW = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "firmware", "pico2_servo_bridge")


def _load(name):
    spec = importlib.util.spec_from_file_location("fw_" + name, os.path.join(FW, name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_cmd_state_roundtrip():
    b = L.pack_cmd(7, True, [0, 1, 2, 3, 4, 4095], [500, 1500, 2000, 2500])
    assert len(b) == L.CMD_SIZE == 28
    c = L.unpack_cmd(b)
    assert c["torque"] and c["seq"] == 7 and c["sts_raw"][-1] == 4095 and c["pwm_us"][1] == 1500
    s = L.unpack_state(L.pack_state(7, True, True, [2048] * 6, [-5, 0, 1, 2, 3, 4], 0b100))
    assert L.STATE_SIZE == 34 and s["watchdog"] and s["load"][0] == -5 and s["err_mask"] == 4
    with pytest.raises(ValueError):
        L.unpack_cmd(b"X3" + b[2:])


def test_firmware_link_and_sts_match_ros_side():
    fl, fs = _load("link"), _load("sts")
    assert fl.CMD_FMT == L.CMD_FMT and fl.STATE_FMT == L.STATE_FMT and fl.MAGIC == L.MAGIC
    assert fl.pack_cmd(3, False, [1] * 6, [900] * 4) == L.pack_cmd(3, False, [1] * 6, [900] * 4)
    ids, raws = (1, 2, 3, 4, 5, 6), [100, 200, 300, 400, 500, 4095]
    assert fs.sync_goal(ids, raws) == S.sync_write_goal_positions(dict(zip(ids, raws)))
    assert fs.torque(3, True) == S.write_torque_enable(3, True)
    assert fs.read_state(2) == S.read_request(2, S.ADDR_PRESENT_POSITION, 6)
    body = [0x02, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x04]
    resp = bytes([0xFF, 0xFF] + body + [S.checksum(body)])
    assert fs.parse_state(resp, 2) == (2048, -16)
    assert fs.parse_state(resp, 3) is None


def test_pico_backend_udp_loopback():
    pico = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pico.bind(("127.0.0.1", 0))
    pico.settimeout(1.0)
    port = pico.getsockname()[1]
    bk = PicoBackend("127.0.0.1", [2048] * 6, [1] * 6, [ServoPulse()] * 4,
                     cmd_port=port, state_port=0, bind_ip="127.0.0.1")
    state_port = bk.sock.getsockname()[1]
    try:
        bk.write([0.0] * 6 + [0.0, 0.0, 0.0, 4.712389], True)
        buf, _ = pico.recvfrom(64)
        c = L.unpack_cmd(buf)
        assert c["torque"] and c["sts_raw"] == [2048] * 6 and c["pwm_us"] == [500, 500, 500, 2500]
        pico.sendto(L.pack_state(c["seq"], True, False, [3072] + [2048] * 5, [100] * 6, 0b10),
                    ("127.0.0.1", state_port))
        for _ in range(100):
            pos, load = bk.read()
            if bk.state is not None:
                break
        assert pos[0] == pytest.approx(1.5707963, abs=1e-3) and load[0] == pytest.approx(0.1)
        assert load[1] == 0.0                  # 오류 비트가 선 서보는 이전 값 유지
    finally:
        bk.close()
        pico.close()


FW_C = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "firmware", "pico2_servo_bridge_c", "src")


@pytest.mark.skipif(not shutil.which("gcc"), reason="gcc 없음")
def test_c_firmware_link_matches_python(tmp_path):
    """C 펌웨어 link.c 를 호스트 gcc 로 빌드해 Python 패킷과 바이트 단위로 같은지 확인."""
    so = tmp_path / "link.so"
    subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-o", str(so), os.path.join(FW_C, "link.c")], check=True)
    lib = ctypes.CDLL(str(so))

    class Cmd(ctypes.Structure):
        _fields_ = [("torque", ctypes.c_bool), ("seq", ctypes.c_uint32),
                    ("sts_raw", ctypes.c_uint16 * 6), ("pwm_us", ctypes.c_uint16 * 4)]

    b = L.pack_cmd(0x01020304, True, [0, 1, 2, 3, 4, 4095], [0, 1500, 2000, 2500])
    c = Cmd()
    assert lib.link_unpack_cmd(b, len(b), ctypes.byref(c))
    assert c.torque and c.seq == 0x01020304 and list(c.sts_raw) == [0, 1, 2, 3, 4, 4095]
    assert list(c.pwm_us) == [0, 1500, 2000, 2500]
    assert not lib.link_unpack_cmd(b[:-1], len(b) - 1, ctypes.byref(c))

    out = (ctypes.c_uint8 * L.STATE_SIZE)()
    pos = (ctypes.c_uint16 * 6)(*[2048, 1, 2, 3, 4, 4095])
    load = (ctypes.c_int16 * 6)(*[-5, 0, 1, 2, 3, 1000])
    lib.link_pack_state(out, ctypes.c_uint32(9), True, True, pos, load, ctypes.c_uint8(0b100))
    assert bytes(out) == L.pack_state(9, True, True, list(pos), list(load), 0b100)
