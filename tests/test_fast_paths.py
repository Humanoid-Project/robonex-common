import itertools
import math
import struct
import warnings

import numpy as np
import pytest

import robonex_common.can as can_module
from robonex_common.can import FeedbackHub, Motor
from robonex_common.foot_roll import _clip_foot_roll_numpy, _clip_foot_roll_scalar, clip_foot_roll, foot_roll
from robonex_common.joints import ALL_MOTORS, POLICY_JOINT_ORDER
from robonex_common.limits import RUNNER_ACTION_CLIP, action_normalization, exceeds_joint_limit, joint_limit_for
from robonex_common.models import VER2_EDU, robot_model
from robonex_common.motors import MOTOR_SPECS
from robonex_common.policy import PolicyContract
from robonex_common.protocol import (
    COMM_FEEDBACK,
    COMM_OPERATION,
    HOST_ID,
    build_arbitration_id,
    clamp,
    float_to_uint,
    uint_to_float,
)
from robonex_common.runtime import OBSERVATION_TERM_SIZES, ActionPipeline, ObservationHistory

ROLL = VER2_EDU.foot_roll
NAN, INF = float("nan"), float("inf")


def _bits(value):
    if isinstance(value, np.ndarray):
        return ("ndarray", value.dtype.str, value.shape, value.tobytes())
    if isinstance(value, np.generic):
        return (type(value).__name__, value.tobytes())
    if isinstance(value, float):
        return ("float", struct.pack("<d", value))
    if isinstance(value, (tuple, list)):
        return (type(value).__name__, tuple(_bits(item) for item in value))
    return (type(value).__name__, repr(value))


def _outcome(function, *args):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            return ("ok", _bits(function(*args)))
        except Exception as error:
            return ("error", type(error).__name__)


def _pipeline():
    offsets, scales, clips = action_normalization(model="ver2_edu")
    order = POLICY_JOINT_ORDER
    contract = PolicyContract(
        3, "test", "policy.onnx", "0" * 64, "0" * 64, "0" * 64, "0" * 64, "scene.xml", tuple(order),
        tuple(f"{name}:{size}x5" for name, size in OBSERVATION_TERM_SIZES),
        tuple(offsets[name] for name in order), tuple(scales[name] for name in order),
        tuple(clips[name] for name in order), RUNNER_ACTION_CLIP, 235, 12, 50.0, "c", "c", "c",
        "ver2_edu", ROLL.limit, tuple(ROLL.coeffs), tuple(ROLL.pairs),
    )
    return ActionPipeline(contract)


def _roll_matches(*args):
    return _outcome(clip_foot_roll, *args) == _outcome(_clip_foot_roll_numpy, *(args if len(args) == 7 else (*args, 8)))


def test_foot_roll_fast_path_is_bit_identical_on_the_ankle_ranges():
    rng = np.random.default_rng(3)
    bounds = [(pair[3], pair[4]) for pair in _pipeline().roll_pairs]
    taken = 0
    for index in range(20000):
        upper_range, lower_range = bounds[index & 1]
        width = 3.0 if index % 3 == 0 else 1.0
        upper = float(rng.uniform(upper_range[0] * width - 0.05, upper_range[1] * width + 0.05))
        lower = float(rng.uniform(lower_range[0] * width - 0.05, lower_range[1] * width + 0.05))
        if index % 2:
            upper, lower = np.float64(upper), np.float64(lower)
        if index % 5 == 0:
            upper_range, lower_range = list(upper_range), list(lower_range)
        args = (upper, lower, upper_range, lower_range, ROLL.coeffs, ROLL.limit)
        assert _roll_matches(*args), args
        taken += _clip_foot_roll_scalar(*args, 8) is not None
    assert taken == 20000


def test_foot_roll_fast_path_is_bit_identical_for_any_coefficients():
    rng = np.random.default_rng(4)
    zeros = (0.0, -0.0)
    for index in range(20000):
        coeffs = tuple(float(value) for value in rng.uniform(-1.0, 1.0, 6))
        limit = float(rng.choice([0.05, 0.2, 1.0, -0.1]))
        upper_range = tuple(sorted(float(value) for value in rng.uniform(-2.0, 2.0, 2)))
        lower_range = tuple(float(value) for value in rng.uniform(-2.0, 2.0, 2))
        upper, lower = (float(value) for value in rng.uniform(-3.0, 3.0, 2))
        if index % 4 == 0:
            upper = zeros[index // 4 % 2]
        if index % 6 == 0:
            lower = zeros[index // 6 % 2]
        args = (upper, lower, upper_range, lower_range, coeffs, limit, int(rng.integers(0, 12)))
        assert _roll_matches(*args), args


def test_foot_roll_fast_path_handles_special_values_like_numpy():
    upper_range, lower_range = (-0.269253, 0.862665), (-0.862665, 0.513599)
    special = [
        0.0, -0.0, 5e-324, -5e-324, 1e-300, 1e150, -1e150, 1e300, 1.7976931348623157e308, NAN, INF, -INF,
        upper_range[0], upper_range[1], math.nextafter(upper_range[1], 9.0), math.nextafter(upper_range[1], -9.0),
        lower_range[0], lower_range[1], 0.24, -0.23,
    ]
    steep = (0.0, 0.0, 0.0, 1e200, 0.0, 1e-200)
    for upper, lower in itertools.product(special, repeat=2):
        for cast in (float, np.float64):
            assert _roll_matches(cast(upper), cast(lower), upper_range, lower_range, ROLL.coeffs, ROLL.limit)
            assert _roll_matches(cast(upper), cast(lower), (0.0, 1.0), (-1.0, -0.0), ROLL.coeffs, ROLL.limit)
            assert _roll_matches(cast(upper), cast(lower), (-1.0, 1.0), (-1.0, 1.0), steep, 0.1)
            assert _roll_matches(cast(upper), cast(lower), (-1.0, 1.0), (-1.0, 1.0), (0.0, -0.5, -0.5, 0.1, 0.0, 0.1), 0.2)
    for value in special:
        assert _roll_matches(0.3, 0.1, (value, 1.0), lower_range, ROLL.coeffs, ROLL.limit)
        assert _roll_matches(0.3, 0.1, upper_range, (-1.0, value), ROLL.coeffs, ROLL.limit)
        assert _roll_matches(0.3, 0.1, upper_range, lower_range, ROLL.coeffs, value)
        assert _roll_matches(0.3, 0.1, upper_range, lower_range, (ROLL.coeffs[0], value, *ROLL.coeffs[2:]), ROLL.limit)


def test_foot_roll_on_the_limit_is_bit_identical():
    rng = np.random.default_rng(5)
    upper_range, lower_range = (-0.269253, 0.862665), (-0.862665, 0.513599)
    for _ in range(5000):
        upper, lower = float(rng.uniform(*upper_range)), float(rng.uniform(*lower_range))
        limit = abs(foot_roll(upper, lower, ROLL.coeffs))
        assert _roll_matches(upper, lower, upper_range, lower_range, ROLL.coeffs, limit)
        assert _roll_matches(upper, lower, upper_range, lower_range, ROLL.coeffs, math.nextafter(limit, 0.0))


def test_foot_roll_leaves_other_input_kinds_to_numpy():
    upper_range, lower_range = (-0.269253, 0.862665), (-0.862665, 0.513599)
    kinds = [1, True, np.float32(0.3), np.int64(1), np.array(0.3), np.array([0.3]), np.array([0.3, -0.2]), [0.3], "0.3", None]
    for kind in kinds:
        for args in (
            (kind, 0.1, upper_range, lower_range, ROLL.coeffs, ROLL.limit),
            (0.9, kind, upper_range, lower_range, ROLL.coeffs, ROLL.limit),
            (0.9, 0.9, upper_range, lower_range, ROLL.coeffs, kind),
        ):
            assert _clip_foot_roll_scalar(*args, 8) is None, args
            assert _roll_matches(*args), args
    for ranges in (np.array(upper_range), [0.1], (*upper_range, 1.0), (np.float32(-0.2), 0.8), (1, 2), "ab", None):
        assert _clip_foot_roll_scalar(0.9, -0.9, ranges, lower_range, ROLL.coeffs, ROLL.limit, 8) is None
        assert _roll_matches(0.9, -0.9, ranges, lower_range, ROLL.coeffs, ROLL.limit)
        assert _roll_matches(0.9, -0.9, upper_range, ranges, ROLL.coeffs, ROLL.limit)
    for coeffs in (np.array(ROLL.coeffs), ROLL.coeffs[:5], (*ROLL.coeffs, 1.0), (0, 1, 1, 0, 0, 0), None):
        assert _clip_foot_roll_scalar(0.9, 0.9, upper_range, lower_range, coeffs, ROLL.limit, 8) is None
        assert _roll_matches(0.9, 0.9, upper_range, lower_range, coeffs, ROLL.limit)
    for iterations in (0, -1, 2.0, np.int64(3), True, None, "8"):
        assert _clip_foot_roll_scalar(0.9, 0.9, upper_range, lower_range, ROLL.coeffs, ROLL.limit, iterations) is None
        assert _roll_matches(0.9, 0.9, upper_range, lower_range, ROLL.coeffs, ROLL.limit, iterations)


def test_pipeline_roll_clip_uses_the_fast_path(monkeypatch):
    import robonex_common.foot_roll as module

    calls = []
    original = module._clip_foot_roll_numpy
    monkeypatch.setattr(module, "_clip_foot_roll_numpy", lambda *args: calls.append(args) or original(*args))
    pipeline = _pipeline()
    rng = np.random.default_rng(6)
    for _ in range(200):
        pipeline.apply(rng.normal(0.0, 3.0, 12))
        pipeline.clip_roll([float(value) for value in rng.uniform(-1.0, 1.0, 12)])
    assert calls == [] and pipeline.roll_clip_count > 0


class _ReferenceHistory:
    def __init__(self, term_sizes, history_length):
        self.term_sizes = tuple(term_sizes)
        self.history_length = history_length
        self.frame_size = sum(size for _, size in self.term_sizes)
        self._frames = None

    def append(self, frame):
        frame = np.asarray(frame, dtype=np.float32).reshape(-1)
        if frame.shape != (self.frame_size,):
            raise ValueError
        if self._frames is None:
            self._frames = np.repeat(frame[None, :], self.history_length, axis=0)
        else:
            self._frames = np.roll(self._frames, -1, axis=0)
            self._frames[-1] = frame

    def observation(self):
        if self._frames is None:
            raise RuntimeError
        pieces = []
        offset = 0
        for _, size in self.term_sizes:
            pieces.append(self._frames[:, offset : offset + size].reshape(-1))
            offset += size
        observation = np.concatenate(pieces).astype(np.float32)
        if not np.isfinite(observation).all():
            raise ValueError
        return observation


@pytest.mark.parametrize(
    "term_sizes,history_length",
    [
        (OBSERVATION_TERM_SIZES, 5),
        ((("a", 3), ("b", 1), ("c", 4)), 1),
        ((("a", 2), ("b", 5)), 3),
        ((("a", 4),), 7),
        ((("a", 0), ("b", 3)), 2),
    ],
)
def test_observation_history_matches_the_reference_layout(term_sizes, history_length):
    rng = np.random.default_rng(7)
    history = ObservationHistory(term_sizes, history_length)
    reference = _ReferenceHistory(term_sizes, history_length)
    assert _outcome(history.observation) == _outcome(reference.observation)
    for index in range(600):
        if index % 97 == 0:
            history.reset()
            reference._frames = None
        frame = rng.uniform(-9.0, 9.0, history.frame_size).astype(np.float32 if index % 2 else np.float64)
        if index % 53 == 0:
            frame[0] = np.inf
        if index % 71 == 0:
            frame = frame[:-1]
        if index % 5 == 0:
            frame = frame.tolist()
        assert _outcome(history.append, frame)[0] == _outcome(reference.append, frame)[0]
        assert _outcome(history.observation) == _outcome(reference.observation)
        if index == 300:
            history.term_sizes = reference.term_sizes = tuple(reversed(history.term_sizes))


def test_observation_is_a_fresh_array_each_call():
    history = ObservationHistory()
    history.append(np.arange(47, dtype=np.float32))
    first = history.observation()
    kept = first.copy()
    history.append(np.arange(47, dtype=np.float32) + 100.0)
    second = history.observation()
    assert first is not second and np.array_equal(first, kept) and not np.array_equal(first, second)
    second[:] = 0.0
    assert np.array_equal(history.observation()[48:60], np.arange(12, dtype=np.float32) + 100.0)


def _reference_clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _reference_float_to_uint(value, lower, upper, bits):
    value = _reference_clamp(value, lower, upper)
    return int((value - lower) / (upper - lower) * ((1 << bits) - 1))


def test_clamp_and_float_to_uint_match_the_min_max_form():
    values = [
        0.0, -0.0, 0.3, -12.57, 12.57, 13.0, -99.0, 1e300, NAN, INF, -INF, 1, -3, True, np.float32(0.3),
        np.float64(-0.7), np.int64(2), np.array(0.4), np.array([0.4]), np.array([0.1, 0.2]), "a", None,
    ]
    lowers = [-12.57, 0.0, 5, np.float32(-1.0), 2.0, NAN]
    uppers = [12.57, 500.0, 0.0, -3.0, np.float64(1.5), NAN]
    for value, lower, upper in itertools.product(values, lowers, uppers):
        assert _outcome(clamp, value, lower, upper) == _outcome(_reference_clamp, value, lower, upper)
        for bits in (16, 8, 1):
            assert _outcome(float_to_uint, value, lower, upper, bits) == _outcome(
                _reference_float_to_uint, value, lower, upper, bits
            )
    rng = np.random.default_rng(8)
    for value in rng.uniform(-15.0, 15.0, 20000):
        assert float_to_uint(value, -12.57, 12.57, 16) == _reference_float_to_uint(value, -12.57, 12.57, 16)
        assert float_to_uint(float(value), -12.57, 12.57, 16) == _reference_float_to_uint(float(value), -12.57, 12.57, 16)


class _Bus:
    def __init__(self):
        self.sent = []
        self.queue = []

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout=0.0):
        return self.queue.pop(0) if self.queue else None


def _reference_control_frame(motor_id, spec, pos, vel, kp, kd, torque):
    raw = [
        _reference_float_to_uint(pos, spec.p_min, spec.p_max, 16),
        _reference_float_to_uint(vel, spec.v_min, spec.v_max, 16),
        _reference_float_to_uint(kp, 0.0, spec.kp_max, 16),
        _reference_float_to_uint(kd, 0.0, spec.kd_max, 16),
    ]
    data = bytes(byte for value in raw for byte in ((value >> 8) & 0xFF, value & 0xFF))
    data16 = _reference_float_to_uint(torque, spec.t_min, spec.t_max, 16)
    return build_arbitration_id(COMM_OPERATION, data16, motor_id), data


@pytest.mark.skipif(can_module.python_can is None, reason="python-can is not installed")
def test_control_frames_match_the_byte_by_byte_encoding():
    rng = np.random.default_rng(9)
    bus = _Bus()
    odd = [NAN, INF, -INF, 1e9, -1e9, 0, 3, np.float32(1.5), np.float64(-2.5), True]
    for index in range(20000):
        name = list(MOTOR_SPECS)[index % len(MOTOR_SPECS)]
        spec = MOTOR_SPECS[name]
        motor = Motor(bus, 1 + index % 27, name)
        args = [float(value) for value in rng.uniform((-14, -60, -10, -1, -70), (14, 60, 5200, 110, 70))]
        if index % 3 == 0:
            args = [np.float64(value) for value in args]
        if index % 7 == 0:
            args[index % 5] = odd[index // 7 % len(odd)]
        motor.control(*args)
        message = bus.sent.pop()
        assert (message.arbitration_id, bytes(message.data)) == _reference_control_frame(motor.motor_id, spec, *args)
        assert message.is_extended_id and message.dlc == 8


def _reference_feedback(spec, data, data16):
    return (
        uint_to_float((data[0] << 8) | data[1], spec.p_min, spec.p_max, 16),
        uint_to_float((data[2] << 8) | data[3], spec.v_min, spec.v_max, 16),
        uint_to_float((data[4] << 8) | data[5], spec.t_min, spec.t_max, 16),
        ((data[6] << 8) | data[7]) / 10.0,
        (data16 >> 8) & 0x3F,
    )


def test_feedback_decoding_matches_uint_to_float_for_every_buffer_kind():
    rng = np.random.default_rng(10)
    kinds = (bytes, bytearray, list, tuple, lambda raw: memoryview(bytes(raw)))
    for index in range(20000):
        name = list(MOTOR_SPECS)[index % len(MOTOR_SPECS)]
        motor = Motor(None, 4, name)
        raw = [int(value) for value in rng.integers(0, 256, (8, 8, 8, 9, 12)[index % 5])]
        data16 = int(rng.integers(0, 1 << 16))
        result = motor.ingest_feedback(kinds[index % len(kinds)](raw), data16=data16, now=float(index))
        expected = _reference_feedback(MOTOR_SPECS[name], raw, data16)
        assert _bits(result) == _bits((float(index), *expected))
        assert motor.last_mode_status == (data16 >> 14) & 0x03
    motor = Motor(None, 4, "rs02")
    for short in (b"", bytes(7), [1, 2, 3]):
        assert motor.ingest_feedback(short) is None and motor.last_position is None


@pytest.mark.skipif(can_module.python_can is None, reason="python-can is not installed")
def test_hub_routes_only_feedback_addressed_to_the_host():
    rng = np.random.default_rng(11)
    message_type = can_module.python_can.Message
    motors = [Motor(None, motor_id, "rs02") for motor_id in range(2, 8)]
    hub = FeedbackHub(_Bus(), motors)
    for index in range(20000):
        comm_type = int(rng.choice([COMM_FEEDBACK, COMM_FEEDBACK, 0x01, 0x11, 0x18, 0x1F]))
        data16 = int(rng.integers(0, 1 << 16)) & ~0xFF | int(rng.integers(0, 12))
        destination = int(rng.choice([HOST_ID, HOST_ID, 0xFE, 3]))
        extended = index % 17 != 0
        arbitration_id = build_arbitration_id(comm_type, data16, destination)
        data = bytes(int(value) for value in rng.integers(0, 256, 8 if index % 9 else 6))
        message = message_type(arbitration_id=arbitration_id if extended else arbitration_id & 0x7FF,
                               data=data, is_extended_id=extended)
        before = {motor.motor_id: motor.last_feedback_time for motor in motors}
        routed = hub.route(message, now=float(index + 1))
        wanted = hub.motors.get(data16 & 0xFF) if extended and comm_type == COMM_FEEDBACK and destination == HOST_ID else None
        assert routed is wanted
        for motor in motors:
            updated = motor is wanted and len(data) >= 8
            assert motor.last_feedback_time == (float(index + 1) if updated else before[motor.motor_id])
        if wanted is not None and len(data) >= 8:
            assert _bits((wanted.last_position, wanted.last_velocity, wanted.last_torque, wanted.last_temp,
                          wanted.last_fault)) == _bits(_reference_feedback(wanted.spec, data, data16))


def _reference_joint_limit_for(motor_id, margin, model):
    limits = robot_model(model).joint_limits_by_id()
    lower, upper = limits[motor_id] if motor_id in limits else {
        joint.motor_id: (joint.lower, joint.upper) for joint in ALL_MOTORS
    }[motor_id]
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError
    return lower + margin, upper - margin


def test_joint_limit_lookup_matches_the_per_model_table():
    motor_ids = [*range(-1, 32), np.int64(5), 5.0, 4.5, "5", None, True]
    for motor_id, margin, model in itertools.product(motor_ids, (0.05, 0.0, 0.01, -0.1, 0.5, 2.0), ("ver2", "ver2_edu", "nope")):
        assert _outcome(joint_limit_for, motor_id, margin, model) == _outcome(
            _reference_joint_limit_for, motor_id, margin, model
        )
        for position in (0.0, -0.75, 0.82, NAN):
            got = _outcome(exceeds_joint_limit, position, motor_id, margin, model)
            want = _outcome(_reference_joint_limit_for, motor_id, margin, model)
            if want[0] == "error":
                assert got == want
            else:
                lower, upper = joint_limit_for(motor_id, margin, model)
                assert got == ("ok", _bits(position <= lower or position >= upper))


def test_observation_history_copies_do_not_share_frames():
    import copy

    history = ObservationHistory((("a", 1), ("b", 2)), 3)
    history.append([1.0, 2.0, 3.0])
    held = history._frames
    clone = copy.copy(history)
    clone.append([7.0, 8.0, 9.0])
    assert history.observation().tolist() == [1.0, 1.0, 1.0, 2.0, 3.0, 2.0, 3.0, 2.0, 3.0]
    history.append([4.0, 5.0, 6.0])
    assert held.tolist() == [[1.0, 2.0, 3.0]] * 3


def test_observation_history_follows_layout_changes_made_after_the_first_read():
    history = ObservationHistory((("a", 1), ("b", 2)), 2)
    reference = _ReferenceHistory((("a", 1), ("b", 2)), 2)
    for target in (history, reference):
        target.term_sizes = [("a", 1), ("b", 2)]
        target.append([1.0, 2.0, 3.0])
        target.append([4.0, 5.0, 6.0])
    assert _outcome(history.observation) == _outcome(reference.observation)
    for target in (history, reference):
        target.term_sizes.reverse()
    assert _outcome(history.observation) == _outcome(reference.observation)
    for target in (history, reference):
        target.history_length = 3
    history.expected_size = 9
    history.reset()
    reference._frames = None
    for target in (history, reference):
        target.append([1.0, 2.0, 3.0])
    assert _outcome(history.observation) == _outcome(reference.observation)
    bare = ObservationHistory.__new__(ObservationHistory)
    bare.__dict__.update(term_sizes=(("a", 3),), history_length=1, frame_size=3, expected_size=3, _frames=None)
    assert bare.append([1.0, 2.0, 3.0]).observation().tolist() == [1.0, 2.0, 3.0]


def test_foot_roll_keeps_the_numpy_error_state():
    args = (1e-200, 0.1, (-1.0, 1.0), (-1.0, 1.0), ROLL.coeffs, ROLL.limit)
    with np.errstate(under="raise"):
        assert _clip_foot_roll_scalar(*args, 8) is None
        with pytest.raises(FloatingPointError):
            clip_foot_roll(*args)
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        for upper, lower in ((1e300, 1e300), (0.3, 0.1)):
            wide = (upper, lower, (-1e308, 1e308), (-1e308, 1e308), (0.0, 0.5, 0.5, 1e200, 0.1, 1e200), 0.2)
            assert _roll_matches(*wide)
    assert _clip_foot_roll_scalar(*args, 8) is not None


@pytest.mark.skipif(can_module.python_can is None, reason="python-can is not installed")
def test_control_fallback_raises_without_a_chained_struct_error(monkeypatch):
    monkeypatch.setattr(can_module, "float_to_uint", lambda *args: 1.5)
    with pytest.raises(TypeError) as caught:
        Motor(_Bus(), 1, "rs02").control(0.0, 0.0, 1.0, 1.0)
    assert caught.value.__context__ is None
