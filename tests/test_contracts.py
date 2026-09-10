import json

import pytest

from robonex_common.joints import (
    ACTUATED_JOINTS,
    DEFAULT_JOINT_POS,
    JOINT_BY_ID,
    PASSIVE_CLOSED_LOOP_JOINTS,
    POLICY_JOINT_ORDER,
)
from robonex_common.limits import (
    ACTION_SCALE_RAD,
    RUNNER_ACTION_CLIP,
    action_limit_reach,
    action_normalization,
    joint_limit_for,
)
from robonex_common.can import Motor
from robonex_common.motors import MOTOR_SPECS
from robonex_common.policy import PolicyContract, mujoco_bundle_sha256, python_source_sha256
from robonex_common.protocol import build_arbitration_id, parse_arbitration_id


def test_joint_contract_is_complete_and_disjoint():
    assert tuple(joint.motor_id for joint in ACTUATED_JOINTS) == tuple(range(1, 13))
    assert len(POLICY_JOINT_ORDER) == 12
    assert set(POLICY_JOINT_ORDER) == {joint.model_name for joint in ACTUATED_JOINTS}
    assert set(POLICY_JOINT_ORDER).isdisjoint(PASSIVE_CLOSED_LOOP_JOINTS)


def test_action_zero_commands_the_standing_pose():
    offsets, scales, clips = action_normalization(0.01)
    for name in POLICY_JOINT_ORDER:
        assert offsets[name] == pytest.approx(DEFAULT_JOINT_POS[name])
        assert scales[name] == pytest.approx(ACTION_SCALE_RAD[name])
        assert clips[name][0] <= offsets[name] <= clips[name][1]


def test_action_scales_match_the_independent_physical_contract():
    expected = {
        "l_hip_yaw_joint": 0.117718,
        "l_hip_pitch_joint": 0.116809,
        "l_hip_roll_joint": 0.031698,
        "l_knee_pitch_joint": 0.078943,
        "l_ankle_upper_joint": 0.025735,
        "l_ankle_lower_joint": 0.028228,
        "r_hip_yaw_joint": 0.117718,
        "r_hip_pitch_joint": 0.116809,
        "r_hip_roll_joint": 0.031698,
        "r_knee_pitch_joint": 0.078943,
        "r_ankle_upper_joint": 0.025735,
        "r_ankle_lower_joint": 0.028228,
    }
    assert ACTION_SCALE_RAD == pytest.approx(expected)


def test_runner_clip_does_not_cross_any_target_clip_fence():
    reach = action_limit_reach(0.01)
    assert set(reach) == set(POLICY_JOINT_ORDER)
    nearest = {name: min(abs(low), abs(high)) for name, (low, high) in reach.items()}
    assert min(nearest.values()) >= RUNNER_ACTION_CLIP
    assert max(nearest.values()) == pytest.approx(RUNNER_ACTION_CLIP, abs=1.e-3)


def test_action_normalization_rejects_a_target_clip_dead_zone(monkeypatch):
    monkeypatch.setitem(ACTION_SCALE_RAD, "l_hip_roll_joint", 0.25)
    with pytest.raises(ValueError, match="target-clip dead zone"):
        action_normalization(0.01)


def test_action_normalization_rejects_a_margin_that_excludes_the_standing_pose():
    with pytest.raises(ValueError):
        action_normalization(0.5)


def test_action_scales_are_mirrored_left_to_right():
    for name in POLICY_JOINT_ORDER:
        if not name.startswith("l_"):
            continue
        assert ACTION_SCALE_RAD[name] == pytest.approx(ACTION_SCALE_RAD["r_" + name[2:]])


def test_joint_limit_margin_is_validated():
    joint = JOINT_BY_ID[3]
    assert joint_limit_for(3) == pytest.approx((joint.lower + 0.05, joint.upper - 0.05))
    with pytest.raises(ValueError):
        joint_limit_for(3, margin=(joint.upper - joint.lower))


def test_arbitration_id_round_trip():
    arbitration_id = build_arbitration_id(0x12, 0xFDEE, 7)
    assert parse_arbitration_id(arbitration_id) == (0x12, 0xFDEE, 7)


def test_policy_contract_rejects_passive_joint(tmp_path):
    payload = {
        "schema_version": 2,
        "task": "test",
        "policy_file": "policy.onnx",
        "policy_sha256": "0" * 64,
        "description_sha256": "1" * 64,
        "common_sha256": "2" * 64,
        "training_sha256": "3" * 64,
        "description_model": "mujoco/robot/scene.xml",
        "joint_order": ["l_knee_joint"],
        "observation_terms": ["joint_pos"],
        "action_offsets": [0.0],
        "action_scales": [1.0],
        "target_clips": [[-1.0, 1.0]],
        "runner_action_clip": RUNNER_ACTION_CLIP,
        "observation_size": 42,
        "action_size": 1,
        "policy_hz": 50.0,
        "description_commit": "test",
        "common_commit": "test",
        "training_commit": "test",
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        PolicyContract.load(path)


def test_policy_contract_rejects_old_schema_cleanly(tmp_path):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported policy manifest schema: 1"):
        PolicyContract.load(path)


def _manifest_payload_for(joint_order):
    offsets, scales, clips = action_normalization(0.01)
    return {
        "schema_version": 2,
        "task": "test",
        "policy_file": "policy.onnx",
        "policy_sha256": "0" * 64,
        "description_sha256": "1" * 64,
        "common_sha256": "2" * 64,
        "training_sha256": "3" * 64,
        "description_model": "mujoco/robot/scene.xml",
        "joint_order": list(joint_order),
        "observation_terms": ["joint_pos"],
        "action_offsets": [offsets[name] for name in joint_order],
        "action_scales": [scales[name] for name in joint_order],
        "target_clips": [list(clips[name]) for name in joint_order],
        "runner_action_clip": RUNNER_ACTION_CLIP,
        "observation_size": 42,
        "action_size": len(joint_order),
        "policy_hz": 50.0,
        "description_commit": "test",
        "common_commit": "test",
        "training_commit": "test",
    }


def test_policy_contract_accepts_correct_order(tmp_path):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(_manifest_payload_for(POLICY_JOINT_ORDER)), encoding="utf-8")
    contract = PolicyContract.load(path)
    assert tuple(contract.joint_order) == tuple(POLICY_JOINT_ORDER)


def test_policy_contract_rejects_permuted_order(tmp_path):
    reversed_order = tuple(reversed(POLICY_JOINT_ORDER))
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(_manifest_payload_for(reversed_order)), encoding="utf-8")
    with pytest.raises(ValueError, match="joint_order must match POLICY_JOINT_ORDER"):
        PolicyContract.load(path)


def test_policy_contract_rejects_a_manifest_from_the_old_action_mapping(tmp_path):
    payload = _manifest_payload_for(POLICY_JOINT_ORDER)
    index = POLICY_JOINT_ORDER.index("l_hip_roll_joint")
    lower, upper = payload["target_clips"][index]
    payload["action_offsets"][index] = (lower + upper) * 0.5
    payload["action_scales"][index] = (upper - lower) * 0.5
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="action contract mismatch for l_hip_roll_joint"):
        PolicyContract.load(path)


def test_policy_contract_rejects_target_clips_from_narrower_joint_limits(tmp_path):
    payload = _manifest_payload_for(POLICY_JOINT_ORDER)
    index = POLICY_JOINT_ORDER.index("r_knee_pitch_joint")
    lower, upper = payload["target_clips"][index]
    payload["target_clips"][index] = [lower + 0.2, upper]
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="action contract mismatch for r_knee_pitch_joint"):
        PolicyContract.load(path)


def test_policy_contract_tolerates_json_float_round_trip(tmp_path):
    payload = _manifest_payload_for(POLICY_JOINT_ORDER)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    contract = PolicyContract.load(path)
    offsets, scales, clips = action_normalization()
    assert contract.action_scales == pytest.approx(tuple(scales[n] for n in POLICY_JOINT_ORDER))
    assert contract.action_offsets == pytest.approx(tuple(offsets[n] for n in POLICY_JOINT_ORDER))


def test_python_source_hash_tracks_source_but_not_other_files(tmp_path):
    source = tmp_path / "src" / "package"
    source.mkdir(parents=True)
    module = source / "module.py"
    module.write_text("VALUE = 1\n", encoding="utf-8")
    first = python_source_sha256(tmp_path, ("src/package",))
    (tmp_path / "README.md").write_text("changed\n", encoding="utf-8")
    assert python_source_sha256(tmp_path, ("src/package",)) == first
    module.write_text("VALUE = 2\n", encoding="utf-8")
    assert python_source_sha256(tmp_path, ("src/package",)) != first


def test_mujoco_bundle_hash_tracks_includes_and_meshes(tmp_path):
    model_dir = tmp_path / "mujoco" / "robot"
    mesh_dir = tmp_path / "meshes"
    model_dir.mkdir(parents=True)
    mesh_dir.mkdir()
    (model_dir / "scene.xml").write_text(
        '<mujoco><include file="robonex.xml"/></mujoco>\n', encoding="utf-8"
    )
    (model_dir / "robonex.xml").write_text(
        '<mujoco><compiler meshdir="../../meshes"/><asset>'
        '<mesh name="body" file="body.stl"/></asset></mujoco>\n', encoding="utf-8"
    )
    mesh = mesh_dir / "body.stl"
    mesh.write_bytes(b"mesh-a")
    first = mujoco_bundle_sha256(tmp_path, "mujoco/robot/scene.xml")
    mesh.write_bytes(b"mesh-b")
    assert mujoco_bundle_sha256(tmp_path, "mujoco/robot/scene.xml") != first


def test_feedback_decoding_includes_mode_status():
    motor = Motor(None, 1, MOTOR_SPECS["rs02"])
    data16 = (2 << 14) | (5 << 8) | 1
    payload = bytes.fromhex("8000800080000190")
    result = motor.ingest_feedback(payload, data16=data16, now=12.5)
    assert result[0] == 12.5
    assert motor.last_mode_status == 2
    assert motor.last_fault == 5
    assert motor.last_temp == 40.0
