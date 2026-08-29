import json

import pytest

from robonex_common.joints import ACTUATED_JOINTS, JOINT_BY_ID, PASSIVE_CLOSED_LOOP_JOINTS, POLICY_JOINT_ORDER
from robonex_common.limits import action_normalization, joint_limit_for
from robonex_common.policy import PolicyContract
from robonex_common.protocol import build_arbitration_id, parse_arbitration_id


def test_joint_contract_is_complete_and_disjoint():
    assert tuple(joint.motor_id for joint in ACTUATED_JOINTS) == tuple(range(1, 13))
    assert len(POLICY_JOINT_ORDER) == 12
    assert set(POLICY_JOINT_ORDER) == {joint.model_name for joint in ACTUATED_JOINTS}
    assert set(POLICY_JOINT_ORDER).isdisjoint(PASSIVE_CLOSED_LOOP_JOINTS)


def test_action_normalization_reaches_margin_clips():
    offsets, scales, clips = action_normalization(0.01)
    for name in POLICY_JOINT_ORDER:
        assert offsets[name] - scales[name] == pytest.approx(clips[name][0])
        assert offsets[name] + scales[name] == pytest.approx(clips[name][1])


def test_joint_limit_margin_is_validated():
    lower, upper = JOINT_BY_ID[3].lower, JOINT_BY_ID[3].upper
    assert joint_limit_for(3) == pytest.approx((lower + 0.05, upper - 0.05))
    with pytest.raises(ValueError):
        joint_limit_for(3, margin=1.0)


def test_arbitration_id_round_trip():
    arbitration_id = build_arbitration_id(0x12, 0xFDEE, 7)
    assert parse_arbitration_id(arbitration_id) == (0x12, 0xFDEE, 7)


def test_policy_contract_rejects_passive_joint(tmp_path):
    payload = {
        "schema_version": 1,
        "policy_file": "policy.onnx",
        "policy_sha256": "0" * 64,
        "joint_order": ["l_knee_joint"],
        "action_offsets": [0.0],
        "action_scales": [1.0],
        "target_clips": [[-1.0, 1.0]],
        "runner_action_clip": 3.0,
        "observation_size": 42,
        "action_size": 1,
        "policy_hz": 50.0,
        "description_commit": "test",
        "common_commit": "test",
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        PolicyContract.load(path)
