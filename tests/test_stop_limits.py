import math
from dataclasses import replace

import pytest

import robonex_common
from robonex_common.joints import (
    ACTUATED_JOINTS,
    AUXILIARY_JOINTS,
    DEFAULT_JOINT_POS,
    DEFAULT_JOINT_POS_LEAN25,
    JOINT_BY_MODEL_NAME,
    JOINT_LIMITS_BY_NAME,
    MOTOR_LIMITS_BY_ID,
    STOP_LIMITS_BY_NAME,
    STOP_LIMITS_DEG,
)
from robonex_common.limits import DEFAULT_ACTION_MARGIN_RAD, action_normalization, exceeds_stop_limit, joint_limit_for, stop_limit_for
from robonex_common.models import ACTION_POSES, ROBOT_MODELS, VARIANTS, VER2, VER2_EDU, _check, robot_model

MEASURED_2026_11_17 = {
    1: (-46.8, 44.5), 2: (-89.2, 113.8), 3: (-145.0, 13.9), 4: (-63.6, 4.2),
    7: (-44.5, 46.8), 8: (-113.8, 89.2), 9: (-13.9, 145.0), 10: (-4.2, 63.6),
}
ANKLE_STOPS = {5: (-38.6, 31.2), 6: (-36.2, 28.1), 11: (-31.2, 38.6), 12: (-28.1, 36.2)}
COMMAND_DEG = {
    1: (-45.8, 43.5), 2: (-88.2, 112.8), 3: (-144.0, 12.9), 4: (-62.6, 0.0), 5: (-37.6, 29.0), 6: (-34.0, 27.1),
    7: (-43.5, 45.8), 8: (-112.8, 88.2), 9: (-12.9, 144.0), 10: (0.0, 62.6), 11: (-29.0, 37.6), 12: (-27.1, 34.0),
}


def test_stop_table_is_the_2026_11_17_measurement():
    expected = {**MEASURED_2026_11_17, **ANKLE_STOPS}
    assert {JOINT_BY_MODEL_NAME[name].motor_id for name in STOP_LIMITS_DEG} == set(expected)
    for joint in ACTUATED_JOINTS:
        assert STOP_LIMITS_DEG[joint.model_name] == expected[joint.motor_id]
        lower, upper = expected[joint.motor_id]
        assert STOP_LIMITS_BY_NAME[joint.model_name] == (math.radians(lower), math.radians(upper))


def test_right_leg_mirrors_the_left_leg():
    for name, (lower, upper) in STOP_LIMITS_DEG.items():
        if name.startswith("l_"):
            assert STOP_LIMITS_DEG["r_" + name[2:]] == (-upper, -lower)


def test_every_model_carries_the_same_stop_table():
    for model in (VER2, VER2_EDU, *VARIANTS.values()):
        assert model.stop_limits == STOP_LIMITS_BY_NAME
        assert model.stop_limits_by_id() == {
            joint.motor_id: STOP_LIMITS_BY_NAME.get(joint.model_name, JOINT_LIMITS_BY_NAME[joint.model_name])
            for joint in ACTUATED_JOINTS
        }


@pytest.mark.parametrize("pose", [DEFAULT_JOINT_POS, DEFAULT_JOINT_POS_LEAN25, dict.fromkeys(DEFAULT_JOINT_POS, 0.0)])
def test_start_poses_lie_inside_the_stops(pose):
    for name, value in pose.items():
        lower, upper = STOP_LIMITS_BY_NAME.get(name, JOINT_LIMITS_BY_NAME[name])
        assert lower < value < upper, name


@pytest.mark.parametrize("model", ["ver2", "ver2_edu"])
def test_stop_limit_for_leg_motors(model):
    for joint in ACTUATED_JOINTS:
        lower, upper = STOP_LIMITS_BY_NAME.get(joint.model_name, JOINT_LIMITS_BY_NAME[joint.model_name])
        assert stop_limit_for(joint.motor_id, model=model) == (lower, upper)
        assert stop_limit_for(joint.motor_id, 0.01, model) == (lower + 0.01, upper - 0.01)


def test_every_leg_joint_has_its_own_stop_limit():
    assert set(STOP_LIMITS_BY_NAME) == {joint.model_name for joint in ACTUATED_JOINTS}


def test_auxiliary_motors_fall_back_to_their_motor_limits():
    for joint in AUXILIARY_JOINTS:
        assert stop_limit_for(joint.motor_id) == MOTOR_LIMITS_BY_ID[joint.motor_id]
        assert stop_limit_for(joint.motor_id) == joint_limit_for(joint.motor_id, 0.0)


def test_stop_limit_for_rejects_bad_input():
    with pytest.raises(KeyError):
        stop_limit_for(99)
    with pytest.raises(ValueError):
        stop_limit_for(4, -0.01)
    with pytest.raises(ValueError):
        stop_limit_for(4, 2.0)
    with pytest.raises(ValueError):
        stop_limit_for(4, model="nope")


def test_exceeds_stop_limit_edges():
    lower, upper = stop_limit_for(4)
    assert not exceeds_stop_limit(0.0, 4)
    assert exceeds_stop_limit(lower, 4) and exceeds_stop_limit(upper, 4)
    assert not exceeds_stop_limit(math.nextafter(upper, 0.0), 4)
    assert exceeds_stop_limit(upper - 0.005, 4, 0.01)
    assert not exceeds_stop_limit(float("nan"), 4)


def test_command_limits_are_the_2_4_0_table():
    assert robot_model("ver2").joint_limits == JOINT_LIMITS_BY_NAME
    for joint in ACTUATED_JOINTS:
        lower, upper = COMMAND_DEG[joint.motor_id]
        assert (joint.lower, joint.upper) == pytest.approx((math.radians(lower), math.radians(upper)), abs=1e-6)


def test_right_command_limits_mirror_the_left():
    for joint in ACTUATED_JOINTS:
        if joint.model_name.startswith("l_"):
            lower, upper = JOINT_LIMITS_BY_NAME[joint.model_name]
            assert JOINT_LIMITS_BY_NAME["r_" + joint.model_name[2:]] == (-upper, -lower)


@pytest.mark.parametrize("model", sorted({*ROBOT_MODELS, *VARIANTS}))
def test_every_command_limit_lies_inside_its_stop_limit(model):
    profile = robot_model(model)
    for joint in ACTUATED_JOINTS:
        lower, upper = profile.joint_limits[joint.model_name]
        stop_lower, stop_upper = stop_limit_for(joint.motor_id, model=model)
        assert stop_lower <= lower < upper <= stop_upper, joint.model_name


@pytest.mark.parametrize("pose", ["home", *sorted(ACTION_POSES)])
def test_target_clips_and_offsets_lie_inside_the_stop_limits(pose):
    offsets, _, clips = action_normalization(DEFAULT_ACTION_MARGIN_RAD, pose=pose)
    for joint in ACTUATED_JOINTS:
        stop_lower, stop_upper = stop_limit_for(joint.motor_id)
        clip_lower, clip_upper = clips[joint.model_name]
        assert stop_lower < clip_lower <= offsets[joint.model_name] <= clip_upper < stop_upper, joint.model_name


def test_a_command_limit_past_its_stop_is_refused_at_import():
    name = "l_hip_yaw_joint"
    lower, upper = VER2_EDU.stop_limits[name]
    limits = {**VER2_EDU.joint_limits, name: (lower, upper + 0.01)}
    with pytest.raises(ValueError, match="command limit of l_hip_yaw_joint is not inside its stop limit"):
        _check(replace(VER2_EDU, name="bad", joint_limits=limits))


def test_new_names_are_exported():
    for name in ("stop_limit_for", "exceeds_stop_limit", "STOP_LIMITS_BY_NAME", "STOP_LIMITS_DEG"):
        assert name in robonex_common.__all__ and hasattr(robonex_common, name)
