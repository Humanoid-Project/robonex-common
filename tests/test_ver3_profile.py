import math

import pytest

from robonex_common import __version__
from robonex_common.actuators import CONTROL_GAINS_BY_JOINT, control_gains
from robonex_common.joints import (
    ACTUATED_JOINTS,
    AUXILIARY_JOINTS,
    JOINT_BY_MODEL_NAME,
    MOTOR_BY_ID,
    MOTOR_LIMITS_BY_ID,
    PASSIVE_CLOSED_LOOP_JOINTS,
    POLICY_JOINT_ORDER,
    VARIANT_MOTOR_IDS,
    VER3_NAME_BY_ID,
    VER3_PASSIVE_CLOSED_LOOP_JOINTS,
    VER3_POLICY_JOINT_ORDER,
)
from robonex_common.limits import (
    action_limit_reach,
    action_normalization,
    exceeds_joint_limit,
    exceeds_stop_limit,
    joint_limit_for,
    stop_limit_for,
)
from robonex_common.models import (
    ACTION_POSES,
    LEG_PROFILES,
    ROBOT_MODELS,
    VARIANTS,
    VER2,
    VER2_EDU,
    VER3,
    VER3_MODELS,
    action_pose,
    leg_profile,
    robot_model,
)
from robonex_common.runtime import ActionPipeline
from test_ver2_profile import _load, _payload

DESCRIPTION_VER3_MOTOR_ID_BY_JOINT = {
    "left_hip_yaw_joint": 1, "left_hip_pitch_joint": 2, "left_hip_roll_joint": 3, "left_knee_pitch_joint": 4,
    "left_ankle_upper_joint": 5, "left_ankle_lower_joint": 6, "right_hip_yaw_joint": 7, "right_hip_pitch_joint": 8,
    "right_hip_roll_joint": 9, "right_knee_pitch_joint": 10, "right_ankle_upper_joint": 11,
    "right_ankle_lower_joint": 12, "left_shoulder_pitch_joint": 16, "left_shoulder_roll_joint": 17,
    "left_shoulder_yaw_joint": 18, "left_elbow_joint": 19, "left_wrist_roll_joint": 20,
    "right_shoulder_pitch_joint": 21, "right_shoulder_roll_joint": 22, "right_shoulder_yaw_joint": 23,
    "right_elbow_joint": 24, "right_wrist_roll_joint": 25, "neck_yaw_joint": 26, "neck_pitch_joint": 27,
}
DESCRIPTION_VER3_POLICY_JOINT_ORDER = (
    "left_hip_yaw_joint", "right_hip_yaw_joint", "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "left_knee_pitch_joint", "right_knee_pitch_joint",
    "left_ankle_lower_joint", "left_ankle_upper_joint", "right_ankle_lower_joint", "right_ankle_upper_joint",
)
DESCRIPTION_VER3_PASSIVE = {
    "left_knee_joint", "right_knee_joint", "left_knee_coupler_joint", "right_knee_coupler_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint",
}
LEGS = tuple(range(1, 13))
ALL_IDS = tuple(sorted(MOTOR_BY_ID))


def _v2(motor_id):
    return MOTOR_BY_ID[motor_id].model_name


def _v3(motor_id):
    return VER3_NAME_BY_ID[motor_id]


def _to_v3(values):
    return {_v3(JOINT_BY_MODEL_NAME[name].motor_id): value for name, value in values.items()}


def _v3_payload():
    payload = _payload("ver2_edu")
    offsets, scales, clips = action_normalization(0.01, model="ver3")
    payload.update(
        robot_model="ver3",
        foot_roll_pairs=[list(pair) for pair in VER3.foot_roll.pairs],
        description_model="ver3/mujoco/robot/full/scene.xml",
        joint_order=list(VER3_POLICY_JOINT_ORDER),
        action_offsets=[offsets[name] for name in VER3_POLICY_JOINT_ORDER],
        action_scales=[scales[name] for name in VER3_POLICY_JOINT_ORDER],
        target_clips=[list(clips[name]) for name in VER3_POLICY_JOINT_ORDER],
    )
    return payload


def test_version():
    assert __version__ == "2.7.0"


def test_ver3_names_match_the_description_constants():
    assert {name: motor_id for motor_id, name in VER3_NAME_BY_ID.items()} == DESCRIPTION_VER3_MOTOR_ID_BY_JOINT
    assert VER3.names_by_id == VER3_NAME_BY_ID
    assert VER3_POLICY_JOINT_ORDER == VER3.policy_joint_order == DESCRIPTION_VER3_POLICY_JOINT_ORDER
    assert set(VER3_PASSIVE_CLOSED_LOOP_JOINTS) == set(VER3.passive_joints) == DESCRIPTION_VER3_PASSIVE


def test_ver3_names_are_the_ver2_names_with_g1_prefixes():
    for motor_id in ALL_IDS:
        old = _v2(motor_id)
        new = old.replace("l_", "left_", 1) if old.startswith("l_") else old
        new = new.replace("r_", "right_", 1) if new.startswith("r_") else new
        new = new.replace("wrist_yaw", "wrist_roll")
        assert _v3(motor_id) == new


def test_ver3_policy_order_is_the_ver2_motor_order():
    assert [VER3.motor_id_for(name) for name in VER3_POLICY_JOINT_ORDER] == [
        JOINT_BY_MODEL_NAME[name].motor_id for name in POLICY_JOINT_ORDER
    ]


def test_ver3_legs_equal_ver2_through_the_id_map():
    assert VER3.joint_limits == _to_v3(VER2_EDU.joint_limits)
    assert VER3.default_joint_pos == _to_v3(VER2_EDU.default_joint_pos)
    assert VER3.stop_limits == _to_v3(VER2_EDU.stop_limits)
    assert VER3.joint_limits_by_id() == VER2.joint_limits_by_id()
    assert VER3.stop_limits_by_id() == VER2.stop_limits_by_id()
    assert (VER3.foot_roll.limit, VER3.foot_roll.coeffs) == (VER2.foot_roll.limit, VER2.foot_roll.coeffs)
    assert VER3.foot_roll.pairs == (
        ("left_ankle_upper_joint", "left_ankle_lower_joint", 1.0),
        ("right_ankle_upper_joint", "right_ankle_lower_joint", -1.0),
    )
    assert VER3.motor_ids == VER2.motor_ids == VARIANT_MOTOR_IDS["ver2"]


@pytest.mark.parametrize("pose", ["home", *sorted(ACTION_POSES)])
def test_ver3_action_contract_equals_ver2(pose):
    v2 = action_normalization(0.01, model="ver2", pose=pose)
    v3 = action_normalization(0.01, model="ver3", pose=pose)
    for old, new in zip(v2, v3):
        assert new == _to_v3(old)
    assert action_pose("ver3", pose) == _to_v3(action_pose("ver2", pose))
    assert action_limit_reach(0.01, model="ver3") == _to_v3(action_limit_reach(0.01, model="ver2"))


@pytest.mark.parametrize("motor_id", ALL_IDS)
def test_limits_by_motor_id_are_the_same_for_both_models(motor_id):
    for margin in (0.0, 0.01, 0.05):
        assert joint_limit_for(motor_id, margin, model="ver3") == joint_limit_for(motor_id, margin, model="ver2")
        assert stop_limit_for(motor_id, margin, model="ver3") == stop_limit_for(motor_id, margin, model="ver2")
    lower, upper = stop_limit_for(motor_id, model="ver3")
    for position in (lower, upper, 0.5 * (lower + upper), math.nan):
        assert exceeds_stop_limit(position, motor_id, model="ver3") == exceeds_stop_limit(position, motor_id)
        assert exceeds_joint_limit(position, motor_id, model="ver3") == exceeds_joint_limit(position, motor_id)


def test_ver3_gains_equal_ver2_by_motor_id():
    gains = control_gains("ver3")
    assert set(gains) == set(VER3_NAME_BY_ID.values())
    for motor_id in ALL_IDS:
        assert gains[_v3(motor_id)] == CONTROL_GAINS_BY_JOINT[_v2(motor_id)]
    assert gains["left_wrist_roll_joint"] == gains["right_wrist_roll_joint"] == (10.0, 0.5)
    assert control_gains("ver2") == {_v2(motor_id): CONTROL_GAINS_BY_JOINT[_v2(motor_id)] for motor_id in ALL_IDS}


def test_ver3_holds_every_arm_and_neck_motor_at_zero_inside_its_limits():
    assert VER3.hold_joint_pos == {_v3(joint.motor_id): 0.0 for joint in AUXILIARY_JOINTS}
    assert VER3.hold_joint_pos_by_id() == {joint.motor_id: 0.0 for joint in AUXILIARY_JOINTS}
    for motor_id, target in VER3.hold_joint_pos_by_id().items():
        lower, upper = MOTOR_LIMITS_BY_ID[motor_id]
        assert lower < target < upper
    assert VER2.hold_joint_pos == VER2_EDU.hold_joint_pos == {}


def test_ver3_is_kept_out_of_the_registries_that_callers_iterate():
    assert set(ROBOT_MODELS) == set(LEG_PROFILES) == {"ver2_edu"}
    assert set(VARIANTS) == set(VARIANT_MOTOR_IDS) == {"ver2"}
    assert set(VER3_MODELS) == {"ver3"}
    assert robot_model("ver3") is VER3 and leg_profile("ver3") is VER3
    assert leg_profile("ver2") is VER2_EDU


def test_ver2_keeps_its_names_order_and_passive_set():
    for model in (VER2, VER2_EDU):
        assert model.names_by_id == {joint.motor_id: joint.model_name for joint in MOTOR_BY_ID.values()}
        assert model.policy_joint_order == POLICY_JOINT_ORDER
        assert model.passive_joints == PASSIVE_CLOSED_LOOP_JOINTS
    assert {joint.model_name for joint in ACTUATED_JOINTS} == set(POLICY_JOINT_ORDER)


def test_ver3_manifest_round_trips(tmp_path):
    contract = _load(tmp_path, _v3_payload())
    assert contract.robot_model == "ver3"
    assert tuple(contract.joint_order) == VER3_POLICY_JOINT_ORDER


def test_ver3_names_are_refused_by_the_ver2_contract(tmp_path):
    payload = _v3_payload()
    payload.update(robot_model="ver2_edu")
    with pytest.raises(ValueError, match="foot-roll clip mismatch"):
        _load(tmp_path, payload)
    payload["foot_roll_pairs"] = [list(pair) for pair in VER2_EDU.foot_roll.pairs]
    with pytest.raises(ValueError, match="unknown or passive joint"):
        _load(tmp_path, payload)


def test_ver2_names_are_refused_by_the_ver3_contract(tmp_path):
    payload = _payload("ver2_edu")
    payload.update(
        robot_model="ver3",
        foot_roll_pairs=[list(pair) for pair in VER3.foot_roll.pairs],
    )
    with pytest.raises(ValueError, match="unknown or passive joint"):
        _load(tmp_path, payload)


def test_ver3_contract_refuses_a_permuted_order(tmp_path):
    payload = _v3_payload()
    order = list(VER3_POLICY_JOINT_ORDER)
    order[8], order[9] = order[9], order[8]
    index = {name: i for i, name in enumerate(VER3_POLICY_JOINT_ORDER)}
    for key in ("action_offsets", "action_scales", "target_clips"):
        payload[key] = [payload[key][index[name]] for name in order]
    payload["joint_order"] = order
    with pytest.raises(ValueError, match="must match POLICY_JOINT_ORDER exactly"):
        _load(tmp_path, payload)


@pytest.mark.parametrize("name", [
    "left_knee_coupler_joint", "left_knee_coupler_joint:0", "left_knee_coupler_joint_x", "left_ankle_pitch_joint",
    "left_ankle_upper_coupler_joint_y", "neck_pitch_joint", "left_wrist_roll_joint", "l_hip_yaw_joint",
])
def test_ver3_contract_refuses_passive_auxiliary_and_ver2_names(tmp_path, name):
    payload = _v3_payload()
    payload["joint_order"][0] = name
    with pytest.raises(ValueError, match="unknown or passive joint|passive closed-loop"):
        _load(tmp_path, payload)


def test_ver3_contract_refuses_other_action_values(tmp_path):
    payload = _v3_payload()
    payload["action_scales"][0] += 1.0e-6
    with pytest.raises(ValueError, match="action contract mismatch for left_hip_yaw_joint"):
        _load(tmp_path, payload)


def test_ver3_pipeline_matches_the_ver2_pipeline(tmp_path):
    v2 = ActionPipeline(_load(tmp_path, _payload("ver2_edu")))
    v3 = ActionPipeline(_load(tmp_path, _v3_payload()))
    for action in ([0.0] * 12, [3.0] * 12, [-14.0, 14.0] * 6, [14.0] * 12, [-14.0] * 12):
        a = v2.apply(list(action))
        b = v3.apply(list(action))
        assert [part.tolist() for part in a] == [part.tolist() for part in b]
    assert (v2.roll_clip_count, v2.target_clip_count) == (v3.roll_clip_count, v3.target_clip_count)
    assert v3.roll_clip_count > 0
