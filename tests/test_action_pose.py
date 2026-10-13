import math

import pytest

from robonex_common.foot_roll import foot_roll
from robonex_common.joints import DEFAULT_JOINT_POS, DEFAULT_JOINT_POS_LEAN25, POLICY_JOINT_ORDER
from robonex_common.limits import action_normalization
from robonex_common.models import ACTION_POSES, VARIANTS, VER2_EDU, action_pose
from test_ver2_profile import _load, _payload


def test_home_is_the_profile_default_and_unchanged():
    assert action_pose("ver2_edu") == DEFAULT_JOINT_POS
    assert action_normalization(0.01, model="ver2_edu", pose="home") == action_normalization(0.01, model="ver2_edu")


def test_lean25_moves_only_the_offsets_of_hip_pitch_and_the_ankle_cranks():
    home = action_normalization(0.01, model="ver2_edu")
    for model in VARIANTS:
        offsets, scales, clips = action_normalization(0.01, model=model, pose="lean25")
        assert scales == home[1]
        assert clips == home[2]
        assert offsets == DEFAULT_JOINT_POS_LEAN25
    moved = {n for n in POLICY_JOINT_ORDER if abs(DEFAULT_JOINT_POS_LEAN25[n] - DEFAULT_JOINT_POS[n]) > 1e-12}
    assert moved == {"l_hip_pitch_joint", "r_hip_pitch_joint", "l_ankle_upper_joint", "l_ankle_lower_joint",
                     "r_ankle_upper_joint", "r_ankle_lower_joint"}


def test_lean25_is_mirrored_and_tilts_the_hip_back_by_two_and_a_half_degrees():
    for name in POLICY_JOINT_ORDER:
        if name.startswith("l_"):
            assert DEFAULT_JOINT_POS_LEAN25["r_" + name[2:]] == pytest.approx(-DEFAULT_JOINT_POS_LEAN25[name], abs=1e-12)
    delta = DEFAULT_JOINT_POS_LEAN25["l_hip_pitch_joint"] - DEFAULT_JOINT_POS["l_hip_pitch_joint"]
    assert math.degrees(delta) == pytest.approx(-2.5, abs=1e-9)


def test_lean25_keeps_the_foot_level_in_roll():
    roll = VER2_EDU.foot_roll
    for upper, lower, sign in roll.pairs:
        home = foot_roll(sign * DEFAULT_JOINT_POS[upper], sign * DEFAULT_JOINT_POS[lower], roll.coeffs)
        lean = foot_roll(sign * DEFAULT_JOINT_POS_LEAN25[upper], sign * DEFAULT_JOINT_POS_LEAN25[lower], roll.coeffs)
        assert abs(math.degrees(lean - home)) < 0.5
        assert abs(lean) < roll.limit


def test_unknown_pose_is_refused():
    with pytest.raises(ValueError, match="unknown action pose"):
        action_normalization(0.01, model="ver2_edu", pose="lean99")
    assert set(ACTION_POSES) == {"lean25"}


def _lean_payload():
    payload = _payload("ver2_edu")
    offsets = action_normalization(0.01, model="ver2_edu", pose="lean25")[0]
    payload["action_offsets"] = [offsets[name] for name in POLICY_JOINT_ORDER]
    payload["action_pose"] = "lean25"
    return payload


def test_manifest_carries_the_pose_and_round_trips(tmp_path):
    contract = _load(tmp_path, _lean_payload())
    assert contract.action_pose == "lean25"
    assert contract.to_dict()["action_pose"] == "lean25"
    assert _load(tmp_path, contract.to_dict()) == contract


def test_home_manifest_does_not_gain_a_pose_field(tmp_path):
    contract = _load(tmp_path, _payload("ver2_edu"))
    assert contract.action_pose == "home"
    assert "action_pose" not in contract.to_dict()


def test_offsets_and_pose_must_agree(tmp_path):
    mixed = _lean_payload()
    del mixed["action_pose"]
    with pytest.raises(ValueError, match="action contract mismatch"):
        _load(tmp_path, mixed)
    mixed = _payload("ver2_edu")
    mixed["action_pose"] = "lean25"
    with pytest.raises(ValueError, match="action contract mismatch"):
        _load(tmp_path, mixed)
    mixed["action_pose"] = "lean99"
    with pytest.raises(ValueError, match="unknown action pose"):
        _load(tmp_path, mixed)
