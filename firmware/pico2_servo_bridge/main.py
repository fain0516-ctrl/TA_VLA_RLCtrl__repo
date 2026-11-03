# -*- coding: utf-8 -*-
"""Pico 2 자작 보드 펌웨어 (MicroPython): Pi 지령(UDP) -> STS3215 버스 + DS3225 PWM, 상태 회신.

역할 분담 (사용자 설명): Pi 5 는 연산·통신만, 이 보드가 서보 구동 전부.
  Pi --UDP 5556--> W5500 --SPI0--> Pico 2 --UART0 1 Mbps--> ADuM1201 --> URT-2 --> STS3215 x5
                                           --PWM 50 Hz--> SN74HCT125N ---------------> DS3225 x5 (엄지는 GP6 직결 가정)
  Pi <--UDP 5555-- (지령 1개당 상태 1개)
필요한 MicroPython: network.WIZNET5K 가 들어 있는 빌드 (예: WIZnet W5500-EVB-Pico2 용 펌웨어).
"""
import socket
import time

import network
from machine import PWM, SPI, UART, WDT, Pin, SoftSPI

import config as C
import link
import sts


# ---------------------------------------------------------------- 하드웨어 초기화
def net_up():
    if C.SOFT_SPI:
        spi = SoftSPI(baudrate=2_000_000, sck=Pin(C.W5500_SCK), mosi=Pin(C.W5500_MOSI), miso=Pin(C.W5500_MISO))
    else:
        spi = SPI(C.W5500_SPI, C.SPI_BAUD, sck=Pin(C.W5500_SCK), mosi=Pin(C.W5500_MOSI), miso=Pin(C.W5500_MISO))
    nic = network.WIZNET5K(spi, Pin(C.W5500_CS), Pin(C.W5500_RST))
    nic.active(True)
    nic.ifconfig((C.PICO_IP, C.NETMASK, C.GATEWAY, C.DNS))
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", C.CMD_PORT))
    s.setblocking(False)
    return nic, s


class StsBus:
    """STS 버스 UART. 방향 핀이 None 이면 URT-2 가 방향을 자동 전환한다고 보고 그냥 보내고 받습니다.
    SN74125N 회로일 때는 송신 중 RX 버퍼, 수신 중 TX 버퍼를 닫습니다 (OE active-low)."""

    def __init__(self):
        self.uart = UART(C.STS_UART, baudrate=C.STS_BAUD, tx=Pin(C.STS_TX), rx=Pin(C.STS_RX),
                         timeout=C.READ_TIMEOUT_MS, timeout_char=1)
        self.tx_en = Pin(C.TX_EN_N_PIN, Pin.OUT, value=1) if C.TX_EN_N_PIN is not None else None
        self.rx_en = Pin(C.RX_EN_N_PIN, Pin.OUT, value=0) if C.RX_EN_N_PIN is not None else None

    def send(self, pkt):
        if self.rx_en is not None:
            self.rx_en.value(1)
        if self.tx_en is not None:
            self.tx_en.value(0)
        self.uart.write(pkt)
        self.uart.flush()                       # 마지막 비트까지 송신 완료 대기
        if self.tx_en is not None:
            self.tx_en.value(1)
        if self.rx_en is not None:
            self.rx_en.value(0)

    def query(self, pkt, n):
        while self.uart.any():                  # 남은 바이트 버림
            self.uart.read()
        self.send(pkt)
        return self.uart.read(n)


class Tendons:
    def __init__(self):
        self.pwm = [PWM(Pin(p), freq=C.PWM_FREQ) for p in C.PWM_PINS]
        self.off()

    def set_us(self, us_list):
        for p, us in zip(self.pwm, us_list):
            us = min(max(us, C.PWM_MIN_US), C.PWM_MAX_US)
            p.duty_ns(int(us * 1000))

    def off(self):
        for p in self.pwm:
            p.duty_ns(0)                        # 펄스 없음 -> 서보 무부하


# ---------------------------------------------------------------- 메인 루프
def main():
    nic, sock = net_up()
    bus, ten = StsBus(), Tendons()
    wdt = WDT(timeout=C.HW_WDT_MS)
    n = len(C.STS_IDS)
    pos, load = [2048] * n, [0] * n
    torque_on = False
    last_cmd_ms = None
    seq, peer = 0, None
    period = 1000 // C.LOOP_HZ

    def set_torque(on):
        for sid in C.STS_IDS:
            bus.query(sts.torque(sid, on), 6)
        if not on:
            ten.off()

    while True:
        t0 = time.ticks_ms()
        wdt.feed()

        # 1) 가장 최근 지령만 적용
        cmd = None
        while True:
            try:
                buf, addr = sock.recvfrom(64)
            except OSError:
                break
            try:
                cmd = link.unpack_cmd(buf)
                peer = (addr[0], C.STATE_PORT)
            except ValueError:
                pass
        if cmd is not None:
            last_cmd_ms, seq = t0, cmd["seq"]
            if cmd["torque"] != torque_on:
                set_torque(cmd["torque"])
                torque_on = cmd["torque"]
            if torque_on:
                bus.send(sts.sync_goal(C.STS_IDS, cmd["sts_raw"]))
                ten.set_us(cmd["pwm_us"])

        # 2) 워치독
        tripped = last_cmd_ms is None or time.ticks_diff(t0, last_cmd_ms) > C.WATCHDOG_MS
        if tripped and torque_on and C.WATCHDOG_ACTION == "torque_off":
            set_torque(False)
            torque_on = False

        # 3) 상태 읽기·회신
        err = 0
        for k, sid in enumerate(C.STS_IDS):
            r = sts.parse_state(bus.query(sts.read_state(sid), 12), sid)
            if r is None:
                err |= 1 << k
            else:
                pos[k], load[k] = r
        if peer is not None:
            try:
                sock.sendto(link.pack_state(seq, torque_on, tripped, pos, load, err), peer)
            except OSError:
                pass

        dt = time.ticks_diff(time.ticks_ms(), t0)
        if dt < period:
            time.sleep_ms(period - dt)


main()
