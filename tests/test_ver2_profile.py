import json
import math

import numpy as np
import pytest

from robonex_common.foot_roll import clip_foot_roll, foot_roll
from robonex_common.joints import POLICY_JOINT_ORDER
from robonex_common.limits import RUNNER_ACTION_CLIP, action_normalization, joint_limit_for
from robonex_common.models import ROBOT_MODELS, VER2_EDU, robot_model
from robonex_common.paths import DESCRIPTION_REPO_NAMES, resolve_repo
from robonex_common.policy import PolicyContract
from robonex_common.runtime import ActionPipeline

CONTRACT_SCALES = {
    "hip_yaw": 0.25, "hip_pitch": 0.25, "hip_roll": 0.148886,
    "knee_pitch": 0.186909, "ankle_upper": 0.169625, "ankle_lower": 0.211194,
}
CONTRACT_LEFT_CLIPS = {
    "hip_yaw": (-0.827758, 0.827758), "hip_pitch": (-1.648063, 1.648063), "hip_roll": (-2.084395, 0.164533),
    "knee_pitch": (-1.21173, 0.164533), "ankle_upper": (-0.269253, 0.862665), "ankle_lower": (-0.862665, 0.513599),
}


def _payload(model, schema=3):
    offsets, scales, clips = action_normalization(0.01, model=model)
    payload = {
        "schema_version": schema,
        "task": "test",
        "policy_file": "policy.onnx",
        "policy_sha256": "0" * 64,
        "description_sha256": "1" * 64,
        "common_sha256": "2" * 64,
        "training_sha256": "3" * 64,
        "description_model": "ver2/mujoco/robot/edu/scene_fixed.xml",
        "joint_order": list(POLICY_JOINT_ORDER),
        "observation_terms": ["joint_pos"],
        "action_offsets": [offsets[name] for name in POLICY_JOINT_ORDER],
        "action_scales": [scales[name] for name in POLICY_JOINT_ORDER],
        "target_clips": [list(clips[name]) for name in POLICY_JOINT_ORDER],
        "runner_action_clip": RUNNER_ACTION_CLIP,
        "observation_size": 42,
        "action_size": len(POLICY_JOINT_ORDER),
        "policy_hz": 50.0,
        "description_commit": "test",
        "common_commit": "test",
        "training_commit": "test",
    }
    if schema >= 3:
        roll = robot_model(model).foot_roll
        payload.update(
            robot_model=model,
            foot_roll_limit=roll.limit if roll else 0.0,
            foot_roll_coeffs=list(roll.coeffs) if roll else [],
            foot_roll_pairs=[list(pair) for pair in roll.pairs] if roll else [],
        )
    return payload


def _load(tmp_path, payload):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return PolicyContract.load(path)


def test_ver2_normalization_matches_the_pinned_contract():
    offsets, scales, clips = action_normalization(0.01, model="ver2_edu")
    for short, scale in CONTRACT_SCALES.items():
        assert scales["l_" + short + "_joint"] == pytest.approx(scale, abs=1e-9)
        assert scales["r_" + short + "_joint"] == pytest.approx(scale, abs=1e-9)
    for short, (lower, upper) in CONTRACT_LEFT_CLIPS.items():
        assert clips["l_" + short + "_joint"] == pytest.approx((lower, upper), abs=1e-9)
        assert clips["r_" + short + "_joint"] == pytest.approx((-upper, -lower), abs=1e-9)
    assert offsets["l_knee_pitch_joint"] == pytest.approx(-0.3961936767101065)


def test_ver2_edu_is_the_default():
    assert action_normalization(0.01) == action_normalization(0.01, model="ver2_edu")
    assert joint_limit_for(1) == joint_limit_for(1, model="ver2_edu")


def test_ver2_limits_match_the_description_constants():
    try:
        root = resolve_repo(DESCRIPTION_REPO_NAMES)
    except Exception:
        pytest.skip("robonex-description is not checked out next to robonex-common")
    constants = root / "ver2" / "ver2_constants.json"
    if not constants.is_file():
        pytest.skip("ver2_constants.json not found")
    data = json.loads(constants.read_text(encoding="utf-8"))
    for name, (lower, upper) in VER2_EDU.joint_limits.items():
        assert (lower, upper) == pytest.approx(tuple(data["provisional_limits"][name]), abs=1e-9)
        assert VER2_EDU.default_joint_pos[name] == pytest.approx(data["default_actuated_pos"][name], abs=1e-12)


def test_schema3_ver2_manifest_round_trips(tmp_path):
    contract = _load(tmp_path, _payload("ver2_edu"))
    assert contract.robot_model == "ver2_edu"
    assert contract.foot_roll_limit == pytest.approx(math.radians(12.0))
    again = _load(tmp_path, contract.to_dict())
    assert again == contract


def test_schema2_manifest_is_refused(tmp_path):
    with pytest.raises(ValueError, match="unsupported policy manifest schema: 2"):
        _load(tmp_path, _payload("ver2_edu", schema=2))


def test_a_manifest_with_the_2_0_0_default_pose_is_refused(tmp_path):
    old_left = {"hip_pitch": 0.1, "knee_pitch": -0.3298656951565011, "ankle_upper": 0.21255773798848565,
                "ankle_lower": -0.2038048914121279}
    old_scales = {"knee_pitch": 0.1648, "ankle_upper": 0.160604, "ankle_lower": 0.219621}
    payload = _payload("ver2_edu")
    for i, name in enumerate(POLICY_JOINT_ORDER):
        side, short = name[0], name[2:-6]
        if short in old_left:
            payload["action_offsets"][i] = old_left[short] if side == "l" else -old_left[short]
        if short in old_scales:
            payload["action_scales"][i] = old_scales[short]
    with pytest.raises(ValueError, match="action contract mismatch"):
        _load(tmp_path, payload)


def test_a_policy_with_other_action_scales_is_refused(tmp_path):
    payload = _payload("ver2_edu")
    payload["action_scales"] = [0.5 * value for value in payload["action_scales"]]
    with pytest.raises(ValueError, match="action contract mismatch"):
        _load(tmp_path, payload)


@pytest.mark.parametrize("field,value", [
    ("foot_roll_limit", math.radians(20.0)),
    ("foot_roll_coeffs", [0.0, 0.5, 0.5, 0.0, 0.0, 0.0]),
    ("foot_roll_pairs", []),
])
def test_a_manifest_cannot_weaken_the_roll_clip(tmp_path, field, value):
    payload = _payload("ver2_edu")
    payload[field] = value
    with pytest.raises(ValueError, match="foot-roll clip mismatch"):
        _load(tmp_path, payload)


def test_unknown_model_is_refused(tmp_path):
    payload = _payload("ver2_edu")
    payload["robot_model"] = "ver1"
    with pytest.raises(ValueError, match="unknown robot model"):
        _load(tmp_path, payload)


def _roll_of(targets, pair_index):
    roll = VER2_EDU.foot_roll
    upper_name, lower_name, sign = roll.pairs[pair_index]
    order = list(POLICY_JOINT_ORDER)
    return foot_roll(sign * targets[order.index(upper_name)], sign * targets[order.index(lower_name)], roll.coeffs)


def test_pipeline_keeps_both_feet_inside_the_roll_band(tmp_path):
    pipeline = ActionPipeline(_load(tmp_path, _payload("ver2_edu")))
    rng = np.random.default_rng(0)
    limit = VER2_EDU.foot_roll.limit
    worst = 0.0
    for _ in range(2000):
        _, targets = pipeline.apply(rng.uniform(-RUNNER_ACTION_CLIP, RUNNER_ACTION_CLIP, len(POLICY_JOINT_ORDER)))
        assert np.all(targets >= pipeline.target_low - 1e-6) and np.all(targets <= pipeline.target_high + 1e-6)
        worst = max(worst, abs(_roll_of(targets, 0)), abs(_roll_of(targets, 1)))
    assert worst <= limit + 1e-4
    assert pipeline.roll_clip_count > 0


def test_pipeline_leaves_the_standing_pose_alone(tmp_path):
    pipeline = ActionPipeline(_load(tmp_path, _payload("ver2_edu")))
    _, targets = pipeline.apply(np.zeros(len(POLICY_JOINT_ORDER)))
    np.testing.assert_allclose(targets, pipeline.offsets, atol=1e-7)
    assert pipeline.roll_clip_count == 0


def test_roll_clip_is_mirror_symmetric(tmp_path):
    pipeline = ActionPipeline(_load(tmp_path, _payload("ver2_edu")))
    order = list(POLICY_JOINT_ORDER)
    rng = np.random.default_rng(1)
    for _ in range(200):
        left = {name: rng.uniform(-6.0, 6.0) for name in order if name.startswith("l_")}
        action = np.zeros(len(order))
        for name, value in left.items():
            action[order.index(name)] = value
            action[order.index("r_" + name[2:])] = -value
        _, targets = pipeline.apply(action)
        for name in left:
            assert targets[order.index("r_" + name[2:])] == pytest.approx(-targets[order.index(name)], abs=1e-6)


def test_clip_is_idempotent():
    roll = VER2_EDU.foot_roll
    upper_range, lower_range = (-0.269253, 0.862665), (-0.862665, 0.513599)
    upper, lower = clip_foot_roll(0.8, 0.5, upper_range, lower_range, roll.coeffs, roll.limit)
    again = clip_foot_roll(upper, lower, upper_range, lower_range, roll.coeffs, roll.limit)
    assert float(again[0]) == pytest.approx(float(upper), abs=1e-9)
    assert float(again[1]) == pytest.approx(float(lower), abs=1e-9)


def test_every_model_is_registered():
    assert set(ROBOT_MODELS) == {"ver2_edu"}
