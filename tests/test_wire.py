import math
from types import SimpleNamespace

import numpy as np
import pytest

import robonex_common.can as can_module
from robonex_common.can import Motor
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import (
    COMM_ENABLE,
    COMM_FEEDBACK,
    COMM_OPERATION,
    COMM_PARAMETER_READ,
    COMM_PARAMETER_WRITE,
    COMM_SAVE,
    COMM_SET_ZERO,
    COMM_STOP,
    FAULT_BIT_NAMES,
    FAULT_STATUS_INDEX,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    RUN_MODE_INDEX,
    RUN_MODE_OPERATION,
    ZERO_STATUS_INDEX,
    parse_arbitration_id,
)
from robonex_common.runtime import GAIT_PERIOD_S, ObservationHistory, gait_phase_at


class RecordingBus:
    def __init__(self, replies=()):
        self.sent = []
        self.replies = list(replies)

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout=0.0):
        return self.replies.pop(0) if self.replies else None


@pytest.fixture
def bus(monkeypatch):
    monkeypatch.setattr(
        can_module,
        "python_can",
        SimpleNamespace(Message=lambda **fields: SimpleNamespace(**fields)),
    )
    return RecordingBus()


def unpack(data):
    return tuple((data[i] << 8) | data[i + 1] for i in range(0, 8, 2))


def test_motor_encoding_ranges_are_the_published_mit_ranges():
    rs02, rs03 = MOTOR_SPECS["rs02"], MOTOR_SPECS["rs03"]
    assert (rs02.p_min, rs02.p_max, rs02.v_min, rs02.v_max) == (-12.57, 12.57, -44.0, 44.0)
    assert (rs02.t_min, rs02.t_max, rs02.kp_max, rs02.kd_max) == (-17.0, 17.0, 500.0, 5.0)
    assert (rs03.p_min, rs03.p_max, rs03.v_min, rs03.v_max) == (-12.57, 12.57, -20.0, 20.0)
    assert (rs03.t_min, rs03.t_max, rs03.kp_max, rs03.kd_max) == (-60.0, 60.0, 5000.0, 100.0)


def test_protocol_constants_match_the_robstride_manual():
    assert HOST_ID == 0xFD
    assert (COMM_OPERATION, COMM_FEEDBACK, COMM_ENABLE, COMM_STOP) == (0x01, 0x02, 0x03, 0x04)
    assert (COMM_SET_ZERO, COMM_PARAMETER_READ, COMM_PARAMETER_WRITE, COMM_SAVE) == (0x06, 0x11, 0x12, 0x16)
    assert (RUN_MODE_INDEX, RUN_MODE_OPERATION) == (0x7005, 0)
    assert (MECHANICAL_POSITION_INDEX, ZERO_STATUS_INDEX, FAULT_STATUS_INDEX) == (0x7019, 0x7029, 0x3022)


@pytest.mark.parametrize(
    "model, kp, kd, torque, expected",
    [
        ("rs03", 150.0, 4.0, 0.0, (1966, 2621, 32767)),
        ("rs03", 1250.0, 75.0, 30.0, (16383, 49151, 49151)),
        ("rs02", 40.0, 2.0, 0.0, (5242, 26214, 32767)),
        ("rs02", 125.0, 3.75, -8.5, (16383, 49151, 16383)),
    ],
)
def test_control_frame_puts_each_field_in_its_slot(bus, model, kp, kd, torque, expected):
    motor = Motor(bus, 4, model)
    motor.control(pos=0.0, vel=0.0, kp=kp, kd=kd, torque=torque)
    message = bus.sent[-1]
    comm_type, data16, motor_id = parse_arbitration_id(message.arbitration_id)
    raw_pos, raw_vel, raw_kp, raw_kd = unpack(message.data)
    assert (comm_type, motor_id, message.is_extended_id) == (0x01, 4, True)
    assert (raw_pos, raw_vel) == (32767, 32767)
    assert (raw_kp, raw_kd, data16) == expected


def test_control_frame_encodes_position_and_velocity_on_the_model_scale(bus):
    Motor(bus, 2, "rs03").control(pos=6.285, vel=-10.0, kp=0.0, kd=0.0)
    Motor(bus, 1, "rs02").control(pos=6.285, vel=-22.0, kp=0.0, kd=0.0)
    rs03_pos, rs03_vel, _, _ = unpack(bus.sent[0].data)
    rs02_pos, rs02_vel, _, _ = unpack(bus.sent[1].data)
    assert rs03_pos == rs02_pos == 49151
    assert rs03_vel == rs02_vel == 16383


def test_enable_stop_and_run_mode_frames(bus):
    motor = Motor(bus, 9, "rs03")
    motor.enable()
    motor.stop(clear_fault=True)
    motor.write_run_mode_operation()
    enable, stop, run_mode = bus.sent
    assert parse_arbitration_id(enable.arbitration_id) == (0x03, 0xFD, 9)
    assert parse_arbitration_id(stop.arbitration_id) == (0x04, 0xFD, 9)
    assert stop.data[0] == 1
    assert parse_arbitration_id(run_mode.arbitration_id) == (0x12, 0xFD, 9)
    assert run_mode.data[0:2] == bytes((0x05, 0x70))
    assert run_mode.data[4] == 0


def test_mechanical_position_read_round_trip(bus):
    payload = bytes((0x19, 0x70, 0, 0)) + np.float32(-0.3857).tobytes()
    bus.replies.append(SimpleNamespace(arbitration_id=(0x11 << 24) | (5 << 8) | 0xFD, data=payload, is_extended_id=True))
    position = Motor(bus, 5, "rs02").read_mech_position(timeout=0.05)
    request = bus.sent[-1]
    assert parse_arbitration_id(request.arbitration_id) == (0x11, 0xFD, 5)
    assert request.data[0:2] == bytes((0x19, 0x70))
    assert position == pytest.approx(-0.3857, abs=1e-6)


def test_gait_clock_is_an_0_8_s_sine_cosine_pair():
    assert GAIT_PERIOD_S == 0.8
    expected = {0: (0.0, 1.0), 10: (1.0, 0.0), 20: (0.0, -1.0), 30: (-1.0, 0.0), 40: (0.0, 1.0)}
    for step, (sin_value, cos_value) in expected.items():
        np.testing.assert_allclose(gait_phase_at(step), (sin_value, cos_value), atol=1e-6)
    np.testing.assert_allclose(gait_phase_at(5), (math.sin(math.pi / 4), math.cos(math.pi / 4)), atol=1e-6)


def test_observation_history_reset_prefills_again():
    history = ObservationHistory((("a", 1),), history_length=3)
    history.append(np.array([1.0]))
    history.append(np.array([2.0]))
    history.reset()
    history.append(np.array([7.0]))
    np.testing.assert_array_equal(history.observation(), [7.0, 7.0, 7.0])


def test_fault_status_register_bits_follow_the_manual():
    assert "overtemperature" in FAULT_BIT_NAMES[0].lower()
    assert "driver" in FAULT_BIT_NAMES[1].lower()
    assert "undervoltage" in FAULT_BIT_NAMES[2].lower()
    assert "overvoltage" in FAULT_BIT_NAMES[3].lower()
    assert "calibrat" in FAULT_BIT_NAMES[7].lower()
    assert "stall" in FAULT_BIT_NAMES[14].lower()
