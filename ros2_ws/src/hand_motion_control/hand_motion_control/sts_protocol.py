# -*- coding: utf-8 -*-
"""Feetech STS3215 시리얼 버스 패킷 (외부 SDK 없이 pyserial 만으로 사용).

레지스터·통신 값은 C:\\MVP_project\\URT2_STS3215_RASPI_GUIDE.md 와 같습니다.
  1 Mbps, 위치 0~4095 (0.088°/step), 목표 위치 42, 토크 활성화 40,
  현재 위치 56 / 속도 58 / 부하 60 (각 2 byte, 하위 바이트 먼저)
패킷: FF FF ID LEN INSTR PARAM... CHK,  LEN = PARAM 수 + 2,  CHK = ~(ID+LEN+INSTR+ΣPARAM) & 0xFF
"""
import math

BAUDRATE = 1_000_000
BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60

POS_RESOLUTION = 4096
POS_CENTER = 2048


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params=()):
    params = list(params)
    body = [sid & 0xFF, len(params) + 2, instr] + params
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def lo(v):
    return v & 0xFF


def hi(v):
    return (v >> 8) & 0xFF


def write_torque_enable(sid, enable):
    return packet(sid, INST_WRITE, [ADDR_TORQUE_ENABLE, 1 if enable else 0])


def read_request(sid, addr, length):
    return packet(sid, INST_READ, [addr, length])


def sync_write_goal_positions(goals):
    """goals: {id: raw 0~4095} -> SYNC WRITE 한 패킷."""
    params = [ADDR_GOAL_POSITION, 2]
    for sid, raw in goals.items():
        raw = int(raw)
        if not 0 <= raw < POS_RESOLUTION:
            raise ValueError(f"위치 범위 밖: id={sid} raw={raw}")
        params += [sid & 0xFF, lo(raw), hi(raw)]
    return packet(BROADCAST_ID, INST_SYNC_WRITE, params)


def parse_status(buf):
    """응답 패킷 -> (id, error, params). 형식이 틀리면 ValueError."""
    if len(buf) < 6 or buf[0] != 0xFF or buf[1] != 0xFF:
        raise ValueError("헤더 오류")
    sid, length, err = buf[2], buf[3], buf[4]
    if len(buf) < 4 + length:
        raise ValueError("길이 부족")
    params = list(buf[5:3 + length])
    if checksum(list(buf[2:3 + length])) != buf[3 + length]:
        raise ValueError("체크섬 오류")
    return sid, err, params


def word(l, h):
    return (l & 0xFF) | ((h & 0xFF) << 8)


def signed_magnitude(v, sign_bit=15):
    """STS 속도·부하 값은 최상위 비트가 부호입니다."""
    if v & (1 << sign_bit):
        return -(v & ((1 << sign_bit) - 1))
    return v


def rad_to_raw(angle, zero_raw=POS_CENTER, direction=1):
    raw = zero_raw + direction * angle * POS_RESOLUTION / (2 * math.pi)
    return int(round(min(max(raw, 0), POS_RESOLUTION - 1)))


def raw_to_rad(raw, zero_raw=POS_CENTER, direction=1):
    return direction * (raw - zero_raw) * 2 * math.pi / POS_RESOLUTION
