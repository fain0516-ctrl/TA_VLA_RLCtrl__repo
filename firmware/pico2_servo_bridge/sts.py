# -*- coding: utf-8 -*-
"""STS3215 패킷 (MicroPython / CPython 공용). ROS 쪽 sts_protocol.py 의 펌웨어용 축약판.
레지스터: 토크 40, 목표 위치 42, 현재 위치 56 (위치 2 B, 속도 2 B, 부하 2 B)."""

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
BROADCAST_ID = 0xFE
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params):
    body = [sid & 0xFF, len(params) + 2, instr] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def torque(sid, on):
    return packet(sid, INST_WRITE, [ADDR_TORQUE_ENABLE, 1 if on else 0])


def read_state(sid):
    return packet(sid, INST_READ, [ADDR_PRESENT_POSITION, 6])


def sync_goal(ids, raws):
    prm = [ADDR_GOAL_POSITION, 2]
    for sid, raw in zip(ids, raws):
        raw = min(max(int(raw), 0), 4095)
        prm += [sid & 0xFF, raw & 0xFF, (raw >> 8) & 0xFF]
    return packet(BROADCAST_ID, INST_SYNC_WRITE, prm)


def parse_state(buf, sid):
    """응답 12 B -> (pos_raw, load) . 실패 시 None. load 는 부호 포함 0.1 % 단위."""
    if buf is None or len(buf) < 12 or buf[0] != 0xFF or buf[1] != 0xFF or buf[2] != sid or buf[3] != 8:
        return None
    if checksum(list(buf[2:11])) != buf[11]:
        return None
    pos = buf[5] | (buf[6] << 8)
    ld = buf[9] | (buf[10] << 8)
    load = -(ld & 0x3FF) if ld & 0x400 else (ld & 0x3FF)
    return pos, load
