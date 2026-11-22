import itertools
import math
import struct

import numpy as np
import pytest

import robonex_common.can as can_module
from robonex_common.can import Motor
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import COMM_PARAMETER_WRITE, clamp, float_to_uint

NONFINITE = [math.nan, math.inf, -math.inf]
KINDS = [float, np.float64, np.float32, np.array]
FIELDS = ("pos", "vel", "kp", "kd", "torque")
FINITE_COMMAND = dict(pos=0.1, vel=-0.2, kp=40.0, kd=2.0, torque=0.5)


class _Bus:
    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(message)


@pytest.mark.parametrize("value", NONFINITE)
@pytest.mark.parametrize("kind", KINDS)
def test_clamp_refuses_a_non_finite_value_for_any_bounds(value, kind):
    bounds = [(-12.57, 12.57), (0.0, 500.0), (5.0, -3.0), (-math.inf, math.inf), (math.nan, math.nan), (-1.0, math.nan)]
    for lower, upper in bounds:
        with pytest.raises(ValueError, match="non-finite"):
            clamp(kind(value), lower, upper)
        for bits in (16, 8, 1):
            with pytest.raises(ValueError, match="non-finite"):
                float_to_uint(kind(value), lower, upper, bits)


def test_finite_values_never_raise_even_with_odd_bounds():
    values = [0.0, -0.0, 1e308, -1e308, 5e-324, 10**400, -(10**400), 3, True, np.float32(2.5), np.float64(-7.0)]
    odd_bounds = [(-12.57, 12.57), (5.0, -3.0), (math.nan, 1.0), (-1.0, math.nan), (-math.inf, math.inf)]
    for value, (lower, upper) in itertools.product(values, odd_bounds):
        clamp(value, lower, upper)
    for value in values:
        float_to_uint(value, -12.57, 12.57, 16)
        float_to_uint(value, 0.0, 500.0, 16)


@pytest.mark.skipif(can_module.python_can is None, reason="python-can is not installed")
@pytest.mark.parametrize("field", FIELDS)
@pytest.mark.parametrize("value", NONFINITE)
@pytest.mark.parametrize("kind", [float, np.float64, np.float32])
def test_every_control_field_refuses_non_finite_and_sends_nothing(field, value, kind):
    for name in MOTOR_SPECS:
        bus = _Bus()
        motor = Motor(bus, 3, name)
        command = dict(FINITE_COMMAND, **{field: kind(value)})
        with pytest.raises(ValueError, match="non-finite"):
            motor.control(**command)
        assert bus.sent == []
        motor.control(**FINITE_COMMAND)
        assert len(bus.sent) == 1


@pytest.mark.skipif(can_module.python_can is None, reason="python-can is not installed")
def test_float_parameter_write_refuses_non_finite_and_sends_nothing():
    bus = _Bus()
    motor = Motor(bus, 3, "rs02")
    for value in NONFINITE:
        with pytest.raises(ValueError, match="non-finite"):
            motor.write_parameter_f32(0x7017, value)
    assert bus.sent == []
    motor.write_parameter_f32(0x7017, 2.5)
    message = bus.sent.pop()
    assert message.arbitration_id >> 24 == COMM_PARAMETER_WRITE
    assert bytes(message.data) == struct.pack("<H", 0x7017) + bytes(2) + struct.pack("<f", 2.5)
