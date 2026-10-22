# -*- coding: utf-8 -*-
"""Raspberry Pi 5 ↔ Pico 2 자작 보드 UDP 패킷 (W5500 이더넷). MicroPython / CPython 공용.

ros2_ws/src/hand_motion_control/hand_motion_control/pico_link.py 와 같은 내용이어야 합니다
(ros2_ws 테스트가 두 파일의 패킷이 같은지 확인).

구성 (사용자 설명): Pi 는 연산·통신만 맡고, Pico 2 자작 보드가 STS3215 버스(URT-2 경유)와 DS3225 서보를 구동.
포트는 테스트 베드 규약을 따릅니다 [인터페이스 문서 IF-03/IF-04]:
  Pi -> Pico  UDP 5556  지령 (100 Hz)
  Pico -> Pi  UDP 5555  상태 (지령 1개당 1개)
테스트 베드는 JSON 이었지만, 여기서는 Pico 쪽 파싱 부담을 줄이려고 고정 길이 이진 패킷을 씁니다.

CMD   "<2sBBI6H4H"   magic 'H3', ver, flags(bit0 토크), seq, STS 목표 raw x6, PWM 펄스 µs x4      (28 B)
STATE "<2sBBI6H6hBB" magic 'H3', ver, flags(bit0 토크, bit1 워치독), seq(마지막 지령),
                     STS 현재 raw x6, STS 부하 x6 (부호 포함, 0.1 %), 통신 오류 비트마스크, 예비     (34 B)
"""
import struct

MAGIC = b"H3"
VERSION = 1
CMD_FMT = "<2sBBI6H4H"
STATE_FMT = "<2sBBI6H6hBB"
CMD_SIZE = struct.calcsize(CMD_FMT)
STATE_SIZE = struct.calcsize(STATE_FMT)
CMD_PORT = 5556
STATE_PORT = 5555

FLAG_TORQUE = 0x01
FLAG_WATCHDOG = 0x02


def pack_cmd(seq, torque, sts_raw, pwm_us):
    if len(sts_raw) != 6 or len(pwm_us) != 4:
        raise ValueError("STS 6개, PWM 4개가 필요합니다")
    flags = FLAG_TORQUE if torque else 0
    return struct.pack(CMD_FMT, MAGIC, VERSION, flags, seq & 0xFFFFFFFF,
                       *[int(v) for v in sts_raw], *[int(round(v)) for v in pwm_us])


def unpack_cmd(buf):
    if len(buf) != CMD_SIZE:
        raise ValueError("지령 길이 오류")
    v = struct.unpack(CMD_FMT, buf)
    if v[0] != MAGIC or v[1] != VERSION:
        raise ValueError("magic/version 오류")
    return dict(torque=bool(v[2] & FLAG_TORQUE), seq=v[3], sts_raw=list(v[4:10]), pwm_us=list(v[10:14]))


def pack_state(seq, torque, watchdog, pos_raw, load, err_mask):
    flags = (FLAG_TORQUE if torque else 0) | (FLAG_WATCHDOG if watchdog else 0)
    return struct.pack(STATE_FMT, MAGIC, VERSION, flags, seq & 0xFFFFFFFF,
                       *[int(v) for v in pos_raw], *[int(v) for v in load], err_mask & 0xFF, 0)


def unpack_state(buf):
    if len(buf) != STATE_SIZE:
        raise ValueError("상태 길이 오류")
    v = struct.unpack(STATE_FMT, buf)
    if v[0] != MAGIC or v[1] != VERSION:
        raise ValueError("magic/version 오류")
    return dict(torque=bool(v[2] & FLAG_TORQUE), watchdog=bool(v[2] & FLAG_WATCHDOG), seq=v[3],
                pos_raw=list(v[4:10]), load=list(v[10:16]), err_mask=v[16])
