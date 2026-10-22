# -*- coding: utf-8 -*-
"""PCA9685 16-Ch PWM 드라이버로 텐던 서보(DS3225) 구동.

배선은 [AI_Priority_Rules/10] 5절 (Raspberry Pi 5 /dev/i2c-1 -> PCA9685 -> 텐던 서보 4개) 을 따릅니다.
펄스 폭 범위와 회전 범위는 [추정] 값이며, 2팀이 받은 DS3225 의 실제 사양으로 config 에서 바꿉니다.
"""
import time
from dataclasses import dataclass

OSC_HZ = 25_000_000
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
SLEEP = 0x10
RESTART = 0x80
AI = 0x20        # auto increment


@dataclass
class ServoPulse:
    min_us: float = 500.0        # [추정]
    max_us: float = 2500.0       # [추정]
    range_rad: float = 4.712389  # 270° [추정]
    zero_rad: float = 0.0        # 텐던이 느슨한(손가락 편) horn 각
    direction: int = 1

    def angle_to_us(self, angle):
        a = self.zero_rad + self.direction * angle
        a = min(max(a, 0.0), self.range_rad)
        return self.min_us + (self.max_us - self.min_us) * a / self.range_rad


def prescale_for(freq_hz):
    return int(round(OSC_HZ / (4096.0 * freq_hz))) - 1


def us_to_ticks(us, freq_hz):
    period_us = 1e6 / freq_hz
    return int(round(min(max(us, 0.0), period_us) / period_us * 4096)) % 4096


class PCA9685:
    """smbus2 가 있는 Raspberry Pi 에서만 실제로 동작합니다."""

    def __init__(self, bus=1, address=0x40, freq_hz=50.0):
        from smbus2 import SMBus          # 지연 import: 시뮬레이션·테스트 환경에는 없어도 됨
        self.bus = SMBus(bus)
        self.addr = address
        self.freq = freq_hz
        old = self.bus.read_byte_data(self.addr, MODE1)
        self.bus.write_byte_data(self.addr, MODE1, (old & 0x7F) | SLEEP)
        self.bus.write_byte_data(self.addr, PRESCALE, prescale_for(freq_hz))
        self.bus.write_byte_data(self.addr, MODE1, (old & ~SLEEP) | AI)
        time.sleep(0.0005)                 # 발진기 안정화 500 µs (데이터시트)
        self.bus.write_byte_data(self.addr, MODE1, (old & ~SLEEP) | AI | RESTART)

    def set_pulse_us(self, channel, us):
        off = us_to_ticks(us, self.freq)
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, off & 0xFF, off >> 8])

    def off(self, channel):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, 0, 0x10])   # full OFF

    def close(self):
        self.bus.close()
