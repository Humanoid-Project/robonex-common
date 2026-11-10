import json

import pytest

from robonex_common.actuators import ACTUATOR_PARAMETERS, CONTROL_GAINS_BY_JOINT
from robonex_common.joints import (
    ACTUATED_JOINTS,
    ALL_MOTORS,
    CHANNEL_MOTOR_IDS,
    DEFAULT_JOINT_POS,
    GROUP_ID_RANGES,
    LEGACY_VARIANT_NAMES,
    JOINT_BY_ID,
    MOTOR_BY_ID,
    MOTOR_LIMITS_BY_ID,
    POLICY_JOINT_ORDER,
    VARIANT_MOTOR_IDS,
    motors_for_variant,
)
from robonex_common.limits import action_normalization, exceeds_joint_limit, joint_limit_for
from robonex_common.models import LEG_PROFILES, ROBOT_MODELS, VARIANTS, VER2, VER2_EDU, leg_profile, robot_model
from robonex_common.motors import JOINT_CONTROL_GAINS, MOTOR_PHYSICS, MOTOR_SPECS
from robonex_common.policy import PolicyContract

from test_ver2_profile import _load, _payload

LEGS = tuple(range(1, 13))
HEAD = (26, 27)
LEFT_ARM, RIGHT_ARM = (16, 17, 18, 19, 20), (21, 22, 23, 24, 25)


def test_variant_motor_sets_match_the_layout_table():
    assert VARIANT_MOTOR_IDS == {"ver2": LEGS + LEFT_ARM + RIGHT_ARM + HEAD}
    for name, ids in VARIANT_MOTOR_IDS.items():
        assert tuple(joint.motor_id for joint in motors_for_variant(name)) == ids
        assert robot_model(name).motor_ids == ids
    assert motors_for_variant("ver2_edu") == motors_for_variant("ver2")
    for retired in ("ver2_pro", "ver2_max", "ver2_ultra"):
        with pytest.raises(ValueError, match="unknown robot variant"):
            motors_for_variant(retired)


def test_new_motor_names_and_models():
    expected = {
        16: ("l_shoulder_pitch_joint", "left_shoulder_pitch", "rs02", "left_arm"),
        17: ("l_shoulder_roll_joint", "left_shoulder_roll", "rs10p", "left_arm"),
        18: ("l_shoulder_yaw_joint", "left_shoulder_yaw", "rs00", "left_arm"),
        19: ("l_elbow_joint", "left_elbow", "rs00", "left_arm"),
        20: ("l_wrist_yaw_joint", "left_wrist_yaw", "rs05", "left_arm"),
        21: ("r_shoulder_pitch_joint", "right_shoulder_pitch", "rs02", "right_arm"),
        22: ("r_shoulder_roll_joint", "right_shoulder_roll", "rs10p", "right_arm"),
        23: ("r_shoulder_yaw_joint", "right_shoulder_yaw", "rs00", "right_arm"),
        24: ("r_elbow_joint", "right_elbow", "rs00", "right_arm"),
        25: ("r_wrist_yaw_joint", "right_wrist_yaw", "rs05", "right_arm"),
        26: ("neck_pitch_joint", "neck_pitch", "rs05", "head"),
        27: ("neck_yaw_joint", "neck_yaw", "rs05", "head"),
    }
    for motor_id, fields in expected.items():
        joint = MOTOR_BY_ID[motor_id]
        assert (joint.model_name, joint.hardware_name, joint.motor_model, joint.group) == fields
        assert motor_id not in JOINT_BY_ID


def test_group_ranges_are_disjoint_and_reserved_ids_are_unused():
    assert GROUP_ID_RANGES == {
        "left_leg": range(1, 7),
        "right_leg": range(7, 13),
        "waist": range(13, 16),
        "left_arm": range(16, 21),
        "right_arm": range(21, 26),
        "head": range(26, 29),
    }
    ranges = list(GROUP_ID_RANGES.values())
    for i, first in enumerate(ranges):
        for second in ranges[i + 1:]:
            assert not set(first) & set(second)
    reserved = {13, 14, 15, 28}
    assert not reserved & set(MOTOR_BY_ID)
    assert all(not reserved & set(ids) for ids in VARIANT_MOTOR_IDS.values())
    assert not any(joint.group == "waist" for joint in ALL_MOTORS)


def test_every_motor_has_a_gain_a_limit_and_a_group_in_range():
    assert set(CONTROL_GAINS_BY_JOINT) == {joint.model_name for joint in ALL_MOTORS}
    assert set(MOTOR_LIMITS_BY_ID) == set(MOTOR_BY_ID) == set(LEGS) | set(LEFT_ARM) | set(RIGHT_ARM) | set(HEAD)
    for joint in ALL_MOTORS:
        kp, kd = CONTROL_GAINS_BY_JOINT[joint.model_name]
        spec = MOTOR_SPECS[joint.motor_model]
        assert 0.0 < kp <= spec.kp_max and 0.0 < kd <= spec.kd_max, joint.model_name
        lower, upper = MOTOR_LIMITS_BY_ID[joint.motor_id]
        assert lower < 0.0 < upper
        assert joint.motor_id in GROUP_ID_RANGES[joint.group]
        for model in VARIANTS:
            assert joint_limit_for(joint.motor_id, margin=0.0, model=model) == pytest.approx((lower, upper))


def test_placeholder_gains_and_limits():
    for role in ("neck_pitch", "neck_yaw"):
        assert JOINT_CONTROL_GAINS[role] == (20.0, 1.0)
    assert JOINT_CONTROL_GAINS["wrist_yaw"] == (10.0, 0.5)
    for role in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"):
        assert JOINT_CONTROL_GAINS[role] == (40.0, 2.0)
    assert MOTOR_LIMITS_BY_ID[26] == pytest.approx((-1.27409, 1.27409))
    assert MOTOR_LIMITS_BY_ID[27] == pytest.approx((-0.523599, 0.523599))
    for motor_id in (16, 21):
        assert MOTOR_LIMITS_BY_ID[motor_id] == pytest.approx((-1.745329, 1.745329))
    for motor_id in (17, 18, 19, 20, 22, 23, 24, 25):
        assert MOTOR_LIMITS_BY_ID[motor_id] == pytest.approx((-0.785398, 0.785398))


def test_arms_are_mirror_symmetric():
    for left, right in zip(LEFT_ARM, RIGHT_ARM):
        l_joint, r_joint = MOTOR_BY_ID[left], MOTOR_BY_ID[right]
        assert r_joint.model_name == "r" + l_joint.model_name[1:]
        assert r_joint.hardware_name == "right" + l_joint.hardware_name[4:]
        assert r_joint.motor_model == l_joint.motor_model
        assert MOTOR_LIMITS_BY_ID[right] == MOTOR_LIMITS_BY_ID[left]
        assert CONTROL_GAINS_BY_JOINT[r_joint.model_name] == CONTROL_GAINS_BY_JOINT[l_joint.model_name]


def test_actuator_parameters_stay_leg_only():
    assert set(ACTUATOR_PARAMETERS) == set(MOTOR_PHYSICS) == {"rs02", "rs03"}
    sim_joints = set()
    for params in ACTUATOR_PARAMETERS.values():
        assert set(params["stiffness"]) == set(params["damping"])
        sim_joints |= set(params["stiffness"])
    assert sim_joints == {joint.model_name for joint in ACTUATED_JOINTS}


def test_the_twelve_joint_contract_is_unchanged():
    assert tuple(joint.motor_id for joint in ACTUATED_JOINTS) == LEGS
    assert set(JOINT_BY_ID) == set(LEGS)
    assert sorted(sum(CHANNEL_MOTOR_IDS.values(), ())) == list(LEGS)
    assert len(POLICY_JOINT_ORDER) == 12
    assert set(DEFAULT_JOINT_POS) == set(POLICY_JOINT_ORDER)


def test_ver2_is_the_only_robot_and_ver2_edu_its_leg_profile():
    assert set(VARIANTS) == set(VARIANT_MOTOR_IDS) == {"ver2"}
    assert set(ROBOT_MODELS) == set(LEG_PROFILES) == {"ver2_edu"}
    assert LEGACY_VARIANT_NAMES == {"ver2_edu": "ver2"}
    assert robot_model("ver2") is VER2 and robot_model("ver2_edu") is VER2_EDU
    assert VER2.leg_profile == "ver2_edu" and VER2.motor_ids == VER2_EDU.motor_ids == VARIANT_MOTOR_IDS["ver2"]
    assert leg_profile("ver2") is leg_profile("ver2_edu") is VER2_EDU
    assert (VER2.joint_limits, VER2.default_joint_pos, VER2.foot_roll) == (
        VER2_EDU.joint_limits, VER2_EDU.default_joint_pos, VER2_EDU.foot_roll)
    assert action_normalization(0.01, model="ver2_edu") == action_normalization(0.01, model="ver2")
    for motor_id in LEGS:
        assert joint_limit_for(motor_id, model="ver2_edu") == joint_limit_for(motor_id, model="ver2")
    assert exceeds_joint_limit(0.0, 21) is False
    assert exceeds_joint_limit(1.8, 21) is True
    for name in ("ver2_pro", "ver2_max", "ver2_ultra"):
        with pytest.raises(ValueError, match="unknown robot model"):
            robot_model(name)


def test_legacy_edu_manifests_still_validate(tmp_path):
    for name in ("ver2_edu", "ver2"):
        contract = _load(tmp_path, _payload(name))
        assert contract.robot_model == name
        assert _load(tmp_path, contract.to_dict()) == contract


def _infer_like_export_policy_manifest(offsets, scales, clips, roll, tolerance=1.0e-9):
    matches = []
    for name, profile in ROBOT_MODELS.items():
        want_offsets, want_scales, want_clips = action_normalization(0.01, model=name)
        same_actions = all(
            abs(offsets[j] - want_offsets[j]) <= tolerance
            and abs(scales[j] - want_scales[j]) <= tolerance
            and all(abs(a - b) <= tolerance for a, b in zip(clips[j], want_clips[j]))
            for j in POLICY_JOINT_ORDER
        )
        want_roll = (profile.foot_roll.limit, tuple(profile.foot_roll.coeffs),
                     tuple(tuple(p) for p in profile.foot_roll.pairs))
        if same_actions and roll == want_roll:
            matches.append(name)
    return matches


def _match_like_verify_model_limits(got):
    return [
        profile.name for profile in ROBOT_MODELS.values()
        if all(got[mid] == pytest.approx(profile.joint_limits_by_id()[mid], abs=5e-6) for mid in got)
    ]


def test_inference_over_robot_models_still_finds_exactly_one_profile():
    roll = VER2_EDU.foot_roll
    saved_roll = (roll.limit, tuple(roll.coeffs), tuple(tuple(p) for p in roll.pairs))
    for name in VARIANTS:
        offsets, scales, clips = action_normalization(0.01, model=name)
        assert _infer_like_export_policy_manifest(offsets, scales, clips, saved_roll) == ["ver2_edu"]
        got = {mid: robot_model(name).joint_limits_by_id()[mid] for mid in LEGS}
        assert _match_like_verify_model_limits(got) == ["ver2_edu"]


def test_arm_motor_specs_match_the_can20_manuals():
    rs00, rs10p = MOTOR_SPECS["rs00"], MOTOR_SPECS["rs10p"]
    assert (rs00.p_min, rs00.p_max, rs00.v_min, rs00.v_max, rs00.t_min, rs00.t_max, rs00.kp_max, rs00.kd_max) == (
        -12.57, 12.57, -33.0, 33.0, -14.0, 14.0, 500.0, 5.0)
    assert (rs10p.p_min, rs10p.p_max, rs10p.v_min, rs10p.v_max, rs10p.t_min, rs10p.t_max, rs10p.kp_max, rs10p.kd_max) == (
        -12.57, 12.57, -13.0, 13.0, -42.0, 42.0, 500.0, 5.0)
    for joint in ALL_MOTORS:
        assert joint.motor_model in MOTOR_SPECS, joint.model_name
