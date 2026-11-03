# -*- coding: utf-8 -*-
import pytest

from team_interface_mocks.fsr import FsrParams, SlipDetector, adc_to_voltage, voltage_to_force
from team_interface_mocks.tactile_serial_bridge_node import parse_line


def test_adc_voltage_force():
    p = FsrParams()
    assert adc_to_voltage(1023, p) == pytest.approx(5.0)
    assert adc_to_voltage(2000, p) == pytest.approx(5.0)
    assert voltage_to_force(0.0, p) == 0.0
    assert voltage_to_force(2.0, p) == pytest.approx(0.4 * 2 + 0.15 * 4)


def test_slip_detector_on_force_drop():
    det = SlipDetector(FsrParams())
    for _ in range(3):
        c, s = det.update([1.0, 0, 0, 0, 0])
    assert c[0] and not s[0]
    c, s = det.update([0.5, 0, 0, 0, 0])
    assert s[0]


def test_parse_line_pads_single_sensor():
    assert parse_line("512\r\n") == [512, 0, 0, 0, 0]
    with pytest.raises(ValueError):
        parse_line("   ")
