# -*- coding: utf-8 -*-
"""4조 FSR 신호 처리 (Arduino ADC -> 전압 -> 하중 N -> 접촉·미끄러짐).

근거 [4조 보고서 1.1]: FSR-402 분압 회로(1 kΩ / 10 kΩ), Arduino Uno, 전압(V)을 하중(N)으로 변환,
2차 다항식 추세선으로 캘리브레이션, 최대값 1023 (10 bit ADC).
아래 계수·임계값은 모두 [추정]이며 4조 캘리브레이션 결과로 바꿔야 합니다.
"""
from collections import deque
from dataclasses import dataclass, field


@dataclass
class FsrParams:
    vcc: float = 5.0
    adc_max: int = 1023
    poly: list = field(default_factory=lambda: [0.0, 0.4, 0.15])   # F = c0 + c1·V + c2·V² [추정]
    contact_threshold: float = 0.15     # N, 3조 Tactile Latch 기준과 같게 [추정]
    slip_drop_ratio: float = 0.3        # 창 안 최대값 대비 30% 이상 하락 [추정]
    slip_window: int = 5                # 샘플 수 (100 Hz 기준 50 ms) [추정]


def adc_to_voltage(adc, p: FsrParams):
    return max(0.0, min(float(adc), p.adc_max)) / p.adc_max * p.vcc


def voltage_to_force(v, p: FsrParams):
    f = sum(c * v ** k for k, c in enumerate(p.poly))
    return max(0.0, f)


class SlipDetector:
    """접촉 중 짧은 시간에 법선력이 급히 떨어지면 미끄러짐으로 판단 (단순 기준선)."""

    def __init__(self, p: FsrParams, n=5):
        self.p = p
        self.hist = [deque(maxlen=p.slip_window) for _ in range(n)]

    def update(self, forces):
        contact, slip = [], []
        for i, f in enumerate(forces):
            h = self.hist[i]
            h.append(f)
            peak = max(h)
            c = f >= self.p.contact_threshold
            s = peak >= self.p.contact_threshold and f < (1.0 - self.p.slip_drop_ratio) * peak
            contact.append(c)
            slip.append(bool(s))
        return contact, slip
