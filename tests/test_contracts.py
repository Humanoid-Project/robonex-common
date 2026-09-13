import json

import pytest

from robonex_common.joints import (
    ACTUATED_JOINTS,
    DEFAULT_JOINT_POS,
    JOINT_BY_ID,
    JOINT_LIMITS_BY_NAME,
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
    assert set(ACTION_SCALE_RAD) == set(POLICY_JOINT_ORDER)
    expected = {
        "l_hip_yaw_joint": 0.25, "r_hip_yaw_joint": 0.25,
        "l_hip_pitch_joint": 0.25, "r_hip_pitch_joint": 0.25,
        "l_knee_pitch_joint": 0.25, "r_knee_pitch_joint": 0.25,
        "l_hip_roll_joint": 0.152626, "r_hip_roll_joint": 0.152626,
        "l_ankle_upper_joint": 0.1201, "r_ankle_upper_joint": 0.1201,
        "l_ankle_lower_joint": 0.131736, "r_ankle_lower_joint": 0.131736,
    }
    assert ACTION_SCALE_RAD == pytest.approx(expected, abs=1e-6)


def test_every_fence_sits_at_least_three_sigma_from_the_action_mean():
    """A fence inside 3 sigma clips a quarter of all steps at init_noise_std=1."""
    reach = action_limit_reach(0.01)
    for name, (low, high) in reach.items():
        assert min(abs(low), abs(high)) >= 2.9, name


def test_every_target_clip_is_reachable_with_headroom_before_the_runner_clip():
    reach = action_limit_reach(0.01)
    assert set(reach) == set(POLICY_JOINT_ORDER)
    farthest = {name: max(abs(low), abs(high)) for name, (low, high) in reach.items()}
    nearest = {name: min(abs(low), abs(high)) for name, (low, high) in reach.items()}
    # the policy can drive every joint onto its fence without being clipped by the runner
    assert max(farthest.values()) <= RUNNER_ACTION_CLIP
    # and it reaches the fence well before the runner clip, so exploration never sits on it
    assert max(nearest.values()) <= 0.5 * RUNNER_ACTION_CLIP
    # every fence is at least three sigma out, so init noise rarely clips
    assert min(nearest.values()) >= 2.9


def test_target_clip_fence_stays_inside_the_hard_joint_limits():
    offsets, scales, clips = action_normalization(0.01)
    for name, (low, high) in clips.items():
        lower, upper = JOINT_LIMITS_BY_NAME[name]
        assert lower < low < high < upper
        assert low == pytest.approx(lower + 0.01)
        assert high == pytest.approx(upper - 0.01)
        assert low < offsets[name] < high


def test_action_normalization_rejects_a_scale_that_cannot_reach_the_fence(monkeypatch):
    monkeypatch.setitem(ACTION_SCALE_RAD, "l_hip_roll_joint", 0.001)
    with pytest.raises(ValueError, match="cannot reach its target clip"):
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


def test_observation_history_is_term_major_with_the_oldest_frame_first():
    from robonex_common.runtime import (
        OBSERVATION_FRAME_SIZE,
        OBSERVATION_HISTORY_LENGTH,
        OBSERVATION_SIZE,
        OBSERVATION_TERM_SIZES,
        ObservationHistory,
    )
    import numpy as np

    history = ObservationHistory()
    # frame k is filled entirely with the value k so the layout is readable
    for k in range(OBSERVATION_HISTORY_LENGTH):
        history.append(np.full(OBSERVATION_FRAME_SIZE, float(k), dtype=np.float32))
    observation = history.observation()
    assert observation.shape == (OBSERVATION_SIZE,)

    # Isaac Lab keeps one buffer per term and flattens it oldest-first, then
    # concatenates the terms: [term0(t-4..t), term1(t-4..t), ...]
    offset = 0
    for _, size in OBSERVATION_TERM_SIZES:
        block = observation[offset : offset + size * OBSERVATION_HISTORY_LENGTH]
        for k in range(OBSERVATION_HISTORY_LENGTH):
            assert np.all(block[k * size : (k + 1) * size] == float(k))
        offset += size * OBSERVATION_HISTORY_LENGTH
    assert offset == OBSERVATION_SIZE


def test_observation_history_prefills_and_rolls():
    from robonex_common.runtime import OBSERVATION_FRAME_SIZE, ObservationHistory
    import numpy as np

    history = ObservationHistory()
    history.append(np.full(OBSERVATION_FRAME_SIZE, 7.0, dtype=np.float32))
    # the first frame back-fills the whole buffer, so nothing reads as zero
    assert np.all(history.observation() == 7.0)
    history.append(np.full(OBSERVATION_FRAME_SIZE, 9.0, dtype=np.float32))
    joint_pos_block = history.observation()[:60]
    assert np.all(joint_pos_block[:48] == 7.0)
    assert np.all(joint_pos_block[48:] == 9.0)


def test_observation_frame_rejects_a_wrong_command_width():
    from robonex_common.runtime import assemble_observation_frame
    import numpy as np

    ok = assemble_observation_frame(
        np.zeros(12), np.zeros(12), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(2), np.zeros(12)
    )
    assert ok.shape == (47,)
    with pytest.raises(ValueError):
        assemble_observation_frame(
            np.zeros(12), np.zeros(12), np.zeros(3), np.zeros(3), np.zeros(2), np.zeros(2), np.zeros(12)
        )


def test_observation_history_follows_the_manifest_layout():
    from robonex_common.runtime import ObservationHistory

    class _Contract:
        def __init__(self, terms, size):
            self.observation_terms = terms
            self.observation_size = size

    walking = ObservationHistory.from_contract(
        _Contract(
            (
                "joint_pos_rel:12x5",
                "joint_vel_rel:12x5",
                "imu_ang_vel:3x5",
                "projected_gravity:3x5",
                "velocity_commands:3x5",
                "gait_phase:2x5",
                "last_action:12x5",
            ),
            235,
        )
    )
    assert (walking.frame_size, walking.history_length, walking.expected_size) == (47, 5, 235)

    # a manifest without an "x<history>" suffix is a single-frame layout
    balancing = ObservationHistory.from_contract(
        _Contract(
            ("joint_pos_rel:12", "joint_vel_rel:12", "imu_ang_vel:3", "projected_gravity:3", "last_action:12"),
            42,
        )
    )
    assert (balancing.frame_size, balancing.history_length, balancing.expected_size) == (42, 1, 42)

    with pytest.raises(ValueError, match="observation_size"):
        ObservationHistory.from_contract(_Contract(("joint_pos_rel:12x5",), 42))
    with pytest.raises(ValueError, match="one history length"):
        ObservationHistory.from_contract(_Contract(("a:3x5", "b:3x2"), 45))
    with pytest.raises(ValueError, match="malformed"):
        ObservationHistory.from_contract(_Contract(("a:bad",), 1))
