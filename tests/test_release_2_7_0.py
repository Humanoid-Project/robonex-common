import json
import math
import struct
import subprocess
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from robonex_common import can as can_module
from robonex_common import buses, identity, limits, models, protocol
from robonex_common.can import Motor
from robonex_common.joints import ALL_MOTORS, AUXILIARY_JOINTS, JOINT_BY_ID, MOTOR_BY_ID
from robonex_common.paths import git_dirty
from robonex_common.policy import PolicyContract, parse_observation_terms
from robonex_common.protocol import FAULT_STATUS_INDEX, MECHANICAL_POSITION_INDEX, build_arbitration_id
from robonex_common.runtime import GAIT_COMMAND_DEADBAND, ObservationHistory, gait_phase_at

from test_ver2_profile import _payload


class _ReplyBus:
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout=None):
        return self.replies.pop(0) if self.replies else None


def _reply(motor_id, index, status, payload_bytes, host=protocol.HOST_ID):
    data = struct.pack("<H", index) + b"\x00\x00" + payload_bytes
    return SimpleNamespace(
        arbitration_id=build_arbitration_id(protocol.COMM_PARAMETER_READ, (status << 8) | motor_id, host),
        data=data,
        is_extended_id=True,
    )


@pytest.fixture
def fake_can(monkeypatch):
    if can_module.python_can is None:
        monkeypatch.setattr(can_module, "python_can", SimpleNamespace(Message=lambda **kw: SimpleNamespace(**kw)))


@pytest.mark.parametrize("index,fmt,raw", [
    (FAULT_STATUS_INDEX, "<I", struct.pack("<I", 0)),
    (MECHANICAL_POSITION_INDEX, "<f", struct.pack("<f", 0.0)),
])
def test_read_parameter_fail_status_returns_none(fake_can, index, fmt, raw):
    motor = Motor(_ReplyBus([_reply(4, index, 1, raw)]), 4, "rs03")
    assert motor.read_parameter(index, fmt, timeout=0.05) is None


def test_read_parameter_success_status_returns_value(fake_can):
    motor = Motor(_ReplyBus([_reply(4, MECHANICAL_POSITION_INDEX, 0, struct.pack("<f", 0.25))]), 4, "rs03")
    assert motor.read_parameter(MECHANICAL_POSITION_INDEX, timeout=0.05) == pytest.approx(0.25)


def test_read_parameter_ignores_other_motor_then_reads_own(fake_can):
    replies = [
        _reply(5, MECHANICAL_POSITION_INDEX, 0, struct.pack("<f", 9.0)),
        _reply(4, MECHANICAL_POSITION_INDEX, 0, struct.pack("<f", -0.5)),
    ]
    motor = Motor(_ReplyBus(replies), 4, "rs03")
    assert motor.read_mech_position(timeout=0.05) == pytest.approx(-0.5)


def test_read_mech_position_fail_reply_is_not_zero(fake_can):
    motor = Motor(_ReplyBus([_reply(4, MECHANICAL_POSITION_INDEX, 1, struct.pack("<f", 0.0))]), 4, "rs03")
    assert motor.read_mech_position(timeout=0.05) is None


def _set(payload, field, index, value):
    payload = json.loads(json.dumps(payload))
    if index is None:
        payload[field] = value
    elif field == "target_clips":
        payload[field][0][index] = value
    else:
        payload[field][index] = value
    return payload


@pytest.mark.parametrize("field,index", [
    ("action_offsets", 0), ("action_scales", 0), ("target_clips", 0), ("target_clips", 1),
    ("runner_action_clip", None), ("policy_hz", None), ("foot_roll_limit", None), ("foot_roll_coeffs", 0),
])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_validate_rejects_non_finite(field, index, value):
    with pytest.raises(ValueError, match="non-finite|foot-roll"):
        PolicyContract.from_dict(_set(_payload("ver2_edu"), field, index, value))


@pytest.mark.parametrize("terms,size,match", [
    (["garbage"], 42, "malformed"),
    (["a:bad"], 42, "malformed"),
    (["a:0"], 0, "malformed|observation_size"),
    ([":42"], 42, "malformed"),
    (["a:3x0"], 0, "malformed|observation_size"),
    (["a:3x5", "b:3x2"], 25, "one history length"),
    (["joint_pos:42"], 41, "observation_size"),
    (["joint_pos_rel:12x5"], 42, "observation_size"),
])
def test_validate_rejects_bad_observation_terms(terms, size, match):
    payload = dict(_payload("ver2_edu"), observation_terms=terms, observation_size=size)
    with pytest.raises(ValueError, match=match):
        PolicyContract.from_dict(payload)


def test_walking_layout_still_validates():
    terms = ["joint_pos_rel:12x5", "joint_vel_rel:12x5", "imu_ang_vel:3x5", "projected_gravity:3x5",
             "velocity_commands:3x5", "gait_phase:2x5", "last_action:12x5"]
    contract = PolicyContract.from_dict(dict(_payload("ver2_edu"), observation_terms=terms, observation_size=235))
    assert ObservationHistory.from_contract(contract).expected_size == 235
    assert parse_observation_terms(terms)[1] == 5


@pytest.mark.parametrize("model,name", [("ver2_edu", "l_knee_joint"), ("ver2_edu", "r_ankle_roll_joint")])
def test_validate_names_passive_joint(model, name):
    payload = _payload(model)
    payload["joint_order"][0] = name
    with pytest.raises(ValueError, match="passive closed-loop"):
        PolicyContract.from_dict(payload)


def test_gait_phase_masks_standing_command():
    for step in range(200):
        unmasked = gait_phase_at(step)
        assert np.array_equal(gait_phase_at(step, command=None), unmasked)
        assert np.array_equal(gait_phase_at(step, command=(0.06, 0.0, 0.0)), unmasked)
        assert np.array_equal(gait_phase_at(step, command=(0.03, 0.0, 0.0)), np.zeros(2, dtype=np.float32))
    assert np.array_equal(gait_phase_at(7, command=(GAIT_COMMAND_DEADBAND, 0.0, 0.0)), np.zeros(2, np.float32))
    assert GAIT_COMMAND_DEADBAND == 0.05
    assert np.array_equal(gait_phase_at(7, command=(math.nan, 0.0, 0.0)), np.zeros(2, np.float32))


def test_overtemp_labels_follow_the_manuals():
    assert protocol.decode_fault_bits(1, "rs02") == ["Overtemperature (>135C)"]
    assert protocol.decode_fault_bits(1, "rs05") == ["Overtemperature (>135C)"]
    assert protocol.decode_fault_bits(1, "rs03") == ["Overtemperature (>145C)"]
    assert {spec.motor_model for spec in ALL_MOTORS} <= set(protocol.OVERTEMP_FAULT_C)
    assert "135C" in protocol.decode_fault_bits(1)[0] and "145C" in protocol.decode_fault_bits(1)[0]
    assert protocol.decode_fault_bits(1 << 14, "rs02") == ["Stall overload"]


@pytest.mark.parametrize("content", ["{", "[]", "5", "null", "\xff\xfe", '{"head": " "}', '{"head": ""}', '{"can0": "can0"}'])
def test_bus_map_errors_name_the_path(tmp_path, monkeypatch, content):
    path = tmp_path / "bus_map.json"
    path.write_bytes(content.encode("latin-1"))
    monkeypatch.setenv(buses.BUS_MAP_ENV, str(path))
    buses.bus_map.cache_clear()
    try:
        with pytest.raises(buses.BusMapError, match=str(path)):
            buses.bus_map()
    finally:
        buses.bus_map.cache_clear()


def test_bus_map_error_is_a_value_error():
    assert issubclass(buses.BusMapError, ValueError)


def test_bad_bus_map_import_message_names_the_path(tmp_path):
    path = tmp_path / "bus_map.json"
    path.write_text("{", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, "-c", "import robonex_common"],
        env={"ROBONEX_BUS_MAP": str(path), "PYTHONPATH": str(models.__file__).rsplit("/robonex_common/", 1)[0]},
        capture_output=True, text=True,
    )
    assert result.returncode != 0 and "BusMapError" in result.stderr and str(path) in result.stderr


def test_git_dirty(tmp_path):
    run = lambda *args: subprocess.run(["git", "-C", str(tmp_path), *args], check=True, capture_output=True)
    run("init", "-q")
    assert git_dirty(tmp_path) is False
    (tmp_path / "a.txt").write_text("1")
    assert git_dirty(tmp_path) is False
    run("add", "a.txt")
    assert git_dirty(tmp_path) is True


def test_identity_reader(tmp_path, monkeypatch):
    path = tmp_path / "robot_model"
    assert identity.read_identity(path) is None
    path.write_text("")
    assert identity.read_identity(path) is None
    path.write_text(" ver2\n")
    assert identity.read_identity(path) == "ver2"
    assert identity.require_identity(path) == "ver2"
    monkeypatch.setenv(identity.ROBOT_IDENTITY_ENV, str(path))
    assert identity.identity_path() == path and identity.read_identity() == "ver2"


def test_identity_reader_errors(tmp_path):
    with pytest.raises(identity.IdentityError, match=str(tmp_path)):
        identity.read_identity(tmp_path)
    binary = tmp_path / "robot_model"
    binary.write_bytes(b"\xff\xfe\x00")
    with pytest.raises(identity.IdentityError, match=str(binary)):
        identity.read_identity(binary)
    with pytest.raises(identity.IdentityError, match="motors will not be moved"):
        identity.require_identity(tmp_path / "missing")
    assert issubclass(identity.IdentityError, ValueError)


def test_identity_defaults_match_the_callers():
    assert identity.ROBOT_IDENTITY_ENV == "ROBONEX_ROBOT_MODEL_FILE"
    assert identity.ROBOT_IDENTITY_FILE.parts[-3:] == (".config", "robonex", "robot_model")


def test_limit_status_and_placeholders():
    assert limits.PLACEHOLDER_LIMIT_MOTOR_IDS == tuple(range(16, 28))
    assert {limits.limit_status(i) for i in JOINT_BY_ID} == {"measured"}
    assert {limits.limit_status(j.motor_id) for j in AUXILIARY_JOINTS} == {"placeholder"}
    assert set(JOINT_BY_ID) | set(limits.PLACEHOLDER_LIMIT_MOTOR_IDS) == set(MOTOR_BY_ID)
    for motor_id in (16, 21, 27):
        assert limits.limit_status(motor_id) == "placeholder"
    with pytest.raises(ValueError):
        limits.limit_status(14)


def test_placeholder_consent_rule():
    assert limits.PLACEHOLDER_CONSENT_FLAG == "--allow-placeholder-limits"
    assert limits.require_placeholder_consent(range(1, 13), allowed=False) == ()
    assert limits.require_placeholder_consent([27, 3, 16, 16], allowed=True) == (16, 27)
    with pytest.raises(limits.PlaceholderLimitError, match="16, 27.*--allow-placeholder-limits"):
        limits.require_placeholder_consent([27, 3, 16], allowed=False)
    with pytest.raises(ValueError):
        limits.require_placeholder_consent([99], allowed=True)


def _broken(**changes):
    return replace(models.VER3, **changes)


def test_check_rejects_partial_hold():
    hold = dict(models.VER3.hold_joint_pos)
    hold.pop(next(iter(hold)))
    with pytest.raises(ValueError, match="hold targets"):
        models._check(_broken(hold_joint_pos=hold))


def test_check_rejects_missing_or_repeated_passive_joints():
    passive = models.VER3.passive_joints
    for bad in (passive[:-1], (*passive[:-1], passive[0])):
        with pytest.raises(ValueError, match="passive joints"):
            models._check(_broken(passive_joints=bad))


def test_check_rejects_swapped_or_flipped_foot_roll_pairs():
    roll = models.VER3.foot_roll
    (u1, l1, s1), (u2, l2, s2) = roll.pairs
    for pairs in (((l1, u1, s1), (u2, l2, s2)), ((u1, l1, -s1), (u2, l2, s2)), ((u1, l1, s1), (u1, l1, s1))):
        with pytest.raises(ValueError, match="foot-roll pairs"):
            models._check(_broken(foot_roll=replace(roll, pairs=pairs)))


def test_check_rejects_partial_stop_limits():
    stops = dict(models.VER3.stop_limits)
    stops.pop(next(iter(stops)))
    with pytest.raises(ValueError, match="stop limits do not cover"):
        models._check(_broken(stop_limits=stops))


def test_current_profiles_pass_the_checks():
    for model in (models.VER2_EDU, models.VER2, models.VER3):
        models._check(model)


def test_ver3_leg_values_equal_ver2_by_id():
    assert models.VER3.joint_limits_by_id() == models.VER2_EDU.joint_limits_by_id()
    assert models.VER3.stop_limits_by_id() == models.VER2_EDU.stop_limits_by_id()
