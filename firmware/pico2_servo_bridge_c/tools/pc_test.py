#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""노트북에서 Pico 2 보드를 시험하는 도구 (ROS 없이, Python 표준 라이브러리만 사용).

노트북 랜 포트에 고정 IP 192.168.10.10/24 를 주고 보드와 랜선으로 연결한 뒤 실행합니다.
지령을 100 Hz 로 보내고, 보드가 돌려주는 상태(위치·부하·오류)를 화면에 보여 줍니다.

  python pc_test.py monitor                    토크 끔 상태로 위치만 읽기 (서보 안 움직임, 처음엔 이것부터)
  python pc_test.py hold                       현재 위치에서 STS 토크 켜고 유지
  python pc_test.py sweep --servo 2 --amp 100  STS 한 개를 현재 위치 기준 ±amp raw 사인파로 (100 raw ≈ 8.8°)
  python pc_test.py pwm --ch 0 --us 1500       DS3225 한 채널에 펄스 (나머지 채널은 펄스 끔)

Ctrl+C 로 끝내면 토크 끔 지령을 보내고 종료합니다.
패킷 형식은 ros2_ws/.../pico_link.py 와 같습니다.
"""
import argparse
import math
import socket
import struct
import time

CMD_FMT = "<2sBBI6H4H"       # 28 B
STATE_FMT = "<2sBBI6H6hBB"   # 34 B
CMD_PORT, STATE_PORT = 5556, 5555


def pack_cmd(seq, torque, sts_raw, pwm_us):
    return struct.pack(CMD_FMT, b"H3", 1, 1 if torque else 0, seq & 0xFFFFFFFF,
                       *[max(0, min(4095, int(v))) for v in sts_raw], *[int(v) for v in pwm_us])


def unpack_state(buf):
    if len(buf) != struct.calcsize(STATE_FMT):
        return None
    v = struct.unpack(STATE_FMT, buf)
    if v[0] != b"H3" or v[1] != 1:
        return None
    return dict(torque=bool(v[2] & 1), watchdog=bool(v[2] & 2), seq=v[3],
                pos=list(v[4:10]), load=list(v[10:16]), err=v[16])


class Board:
    def __init__(self, ip):
        self.addr = (ip, CMD_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", STATE_PORT))
        self.sock.settimeout(0.05)
        self.seq = 0
        self.last = None

    def exchange(self, torque, sts_raw, pwm_us):
        self.seq += 1
        self.sock.sendto(pack_cmd(self.seq, torque, sts_raw, pwm_us), self.addr)
        try:
            buf, _ = self.sock.recvfrom(64)
            st = unpack_state(buf)
            if st:
                self.last = st
            return st
        except socket.timeout:
            return None


def show(st, extra=""):
    if st is None:
        print("응답 없음 (IP, 랜선, 방화벽 UDP 5555 허용 여부 확인)")
        return
    errs = [str(k) for k in range(6) if st["err"] & (1 << k)]
    print(f"seq {st['seq']:6d}  토크 {'ON ' if st['torque'] else 'OFF'}  워치독 {'동작' if st['watchdog'] else '정상'}  "
          f"위치 {st['pos']}  부하 {st['load']}  STS 응답 없음: {','.join(errs) or '없음'} {extra}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["monitor", "hold", "sweep", "pwm"])
    ap.add_argument("--ip", default="192.168.10.20", help="보드 IP (config.h PICO_IP)")
    ap.add_argument("--servo", type=int, default=0, help="sweep: STS 순번 0~5 (ID 1~6)")
    ap.add_argument("--amp", type=int, default=100, help="sweep: 진폭 raw (4096 = 360°)")
    ap.add_argument("--period", type=float, default=3.0, help="sweep: 주기 s")
    ap.add_argument("--ch", type=int, default=0, help="pwm: 채널 0~3 (GP2~GP5, U4~U7)")
    ap.add_argument("--us", type=int, default=1500, help="pwm: 펄스 µs (500~2500)")
    a = ap.parse_args()

    b = Board(a.ip)
    off = [0, 0, 0, 0]
    print(f"보드 {a.ip}:{CMD_PORT} 로 지령, 상태는 이 PC 의 UDP {STATE_PORT} 에서 받음")

    # 먼저 토크 끔 상태로 현재 위치를 읽어 둡니다 (토크를 켤 때 갑자기 튀지 않게).
    st = None
    for _ in range(50):
        st = b.exchange(False, [2048] * 6, off) or st
        time.sleep(0.01)
    show(st, "(시작 위치)")
    if st is None:
        return
    base = list(st["pos"])
    if a.mode != "monitor" and st["err"]:
        print("경고: 응답 없는 STS 는 위치를 모르므로 2048(중앙)으로 지령됩니다. 연결 후 다시 시작하세요.")

    t0, n = time.time(), 0
    try:
        while True:
            t = time.time() - t0
            goal, pwm, torque = list(base), list(off), a.mode != "monitor"
            if a.mode == "sweep":
                goal[a.servo] = base[a.servo] + a.amp * math.sin(2 * math.pi * t / a.period)
            elif a.mode == "pwm":
                torque = True
                pwm[a.ch] = a.us
            st = b.exchange(torque, goal, pwm)
            n += 1
            if n % 20 == 0:
                show(st, f"지령 {int(goal[a.servo])}" if a.mode == "sweep" else "")
            time.sleep(max(0.0, 0.01 - (time.time() - t0 - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            b.exchange(False, base, off)
        print("토크 끔 지령 보내고 종료")


if __name__ == "__main__":
    main()
