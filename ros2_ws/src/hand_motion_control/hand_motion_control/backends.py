# -*- coding: utf-8 -*-
"""서보 백엔드: 시뮬레이션(sim) / 실제 하드웨어(hardware).

3조 중간보고서 코멘트 "다른 팀과 연동 전, 시뮬레이션을 활용해 제어 알고리즘을 먼저 테스트" 에 맞춰,
같은 노드가 backend 파라미터만 바꿔 시뮬레이션과 실기를 오갑니다.

입력은 항상 servo_cmd[10] (rad): 0~5 STS3215 관절각, 6~9 텐던 서보 horn 각.
출력 servo_pos[10] (rad) 은 같은 순서의 측정값(시뮬레이션은 모델값)입니다.
"""
import math

from . import hand_spec as H
from . import sts_protocol as S


class SimBackend:
    """서보를 1차 지연 + 속도 제한으로 근사합니다. 물체 접촉은 모델링하지 않습니다."""

    def __init__(self, tau=0.03, vmax=5.0):
        self.tau = tau          # s [추정]
        self.vmax = vmax        # rad/s [추정]
        self.pos = [0.0] * H.N_ACTUATORS
        self.cmd = [0.0] * H.N_ACTUATORS
        self.enabled = True

    def write(self, servo_cmd, torque_enabled=True):
        self.cmd = list(servo_cmd)
        self.enabled = torque_enabled

    def step(self, dt):
        if not self.enabled:
            return
        a = 1.0 - math.exp(-dt / self.tau)
        for i in range(H.N_ACTUATORS):
            d = a * (self.cmd[i] - self.pos[i])
            lim = self.vmax * dt
            self.pos[i] += max(-lim, min(lim, d))

    def read(self):
        return list(self.pos), [0.0] * H.N_ACTUATORS

    def close(self):
        pass


class HardwareBackend:
    """STS3215 6개(시리얼 버스) + DS3225 4개(PCA9685)."""

    def __init__(self, port, sts_ids, sts_zero_raw, sts_dir, pca_channels, pulses,
                 pca_bus=1, pca_address=0x40, pwm_freq=50.0, read_timeout=0.004):
        import serial                       # pyserial, 실기에서만 필요
        self.ser = serial.Serial(port, S.BAUDRATE, timeout=read_timeout)
        self.ids = list(sts_ids)
        self.zero = list(sts_zero_raw)
        self.dir = list(sts_dir)
        from .pca9685 import PCA9685
        self.pca = PCA9685(pca_bus, pca_address, pwm_freq)
        self.ch = list(pca_channels)
        self.pulses = list(pulses)
        self.enabled = None
        self.last_pos = [0.0] * H.N_ACTUATORS

    def _torque(self, on):
        for sid in self.ids:
            self.ser.write(S.write_torque_enable(sid, on))
            self.ser.read(6)                  # 상태 응답 버림
        if not on:
            for ch in self.ch:
                self.pca.off(ch)
        self.enabled = on

    def write(self, servo_cmd, torque_enabled=True):
        if torque_enabled != self.enabled:
            self._torque(torque_enabled)
        if not torque_enabled:
            return
        goals = {sid: S.rad_to_raw(servo_cmd[k], self.zero[k], self.dir[k])
                 for k, sid in enumerate(self.ids)}
        self.ser.write(S.sync_write_goal_positions(goals))   # SYNC WRITE 는 응답 없음
        for k, ch in enumerate(self.ch):
            self.pca.set_pulse_us(ch, self.pulses[k].angle_to_us(servo_cmd[6 + k]))
            self.last_pos[6 + k] = servo_cmd[6 + k]          # PWM 서보는 위치 피드백 없음

    def step(self, dt):
        pass

    def read(self):
        """STS 위치·부하를 읽습니다. 텐던 서보는 마지막 지령값을 그대로 돌려줍니다."""
        load = [0.0] * H.N_ACTUATORS
        for k, sid in enumerate(self.ids):
            self.ser.reset_input_buffer()
            self.ser.write(S.read_request(sid, S.ADDR_PRESENT_POSITION, 6))
            buf = self.ser.read(12)
            try:
                _, _, prm = S.parse_status(buf)
                raw = S.word(prm[0], prm[1])
                self.last_pos[k] = S.raw_to_rad(raw, self.zero[k], self.dir[k])
                load[k] = S.signed_magnitude(S.word(prm[4], prm[5]), 10) / 1000.0
            except (ValueError, IndexError):
                pass                          # 이전 값 유지 (통신 실패 1회는 무시)
        return list(self.last_pos), load

    def close(self):
        try:
            self._torque(False)
        finally:
            self.ser.close()
            self.pca.close()
