import subprocess
import sys
from pathlib import Path

import pytest

import robonex_common
from robonex_common.actuators import ACTUATOR_PARAMETERS
from robonex_common.imu import DEFAULT_IMU_BAUDRATE, DEFAULT_IMU_PORT, MOUNT_ROLL_DEG
from robonex_common.joints import ACTUATED_JOINTS, JOINT_BY_ID, channel_for_motor_id
from robonex_common.motors import MOTOR_CONTROL_KD, MOTOR_CONTROL_KP, MOTOR_PHYSICS
from robonex_common.paths import resolve_repo
from robonex_common.protocol import FAULT_BIT_NAMES, decode_fault_bits


def test_package_exports_resolve():
    assert robonex_common.__version__
    for name in robonex_common.__all__:
        assert hasattr(robonex_common, name), name


def _probe(body):
    return subprocess.run(
        [sys.executable, "-c", "import sys\nsys.modules['numpy'] = None\nsys.modules['can'] = None\n" + body],
        capture_output=True,
        text=True,
        cwd=Path(__file__).resolve().parents[1],
    )


def test_core_package_imports_with_numpy_and_python_can_unavailable():
    result = _probe("import robonex_common\nprint(robonex_common.__version__)")
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip()


def test_runtime_reports_the_missing_policy_extra():
    result = _probe("import robonex_common.runtime")
    assert result.returncode != 0
    assert "robonex-common[policy]" in result.stderr


MEASURED_JOINTS = {
    1: ("l_hip_yaw_joint", "rs02", "can0", -1.658063, 1.658063),
    2: ("l_hip_pitch_joint", "rs03", "can0", -1.745329, 1.745329),
    3: ("l_hip_roll_joint", "rs03", "can0", -2.146755, 0.453786),
    4: ("l_knee_pitch_joint", "rs03", "can0", -1.500983, 0.942478),
    5: ("l_ankle_upper_joint", "rs02", "can0", -0.610865, 0.575959),
    6: ("l_ankle_lower_joint", "rs02", "can0", -0.610865, 0.575959),
    7: ("r_hip_yaw_joint", "rs02", "can1", -1.658063, 1.658063),
    8: ("r_hip_pitch_joint", "rs03", "can1", -1.745329, 1.745329),
    9: ("r_hip_roll_joint", "rs03", "can1", -0.453786, 2.146755),
    10: ("r_knee_pitch_joint", "rs03", "can1", -0.942478, 1.500983),
    11: ("r_ankle_upper_joint", "rs02", "can1", -0.575959, 0.610865),
    12: ("r_ankle_lower_joint", "rs02", "can1", -0.575959, 0.610865),
}
MEASURED_PHYSICS = {
    "rs02": (0.0042, 0.1),
    "rs03": (0.02, 0.2),
}
MEASURED_GAINS = (40.0, 2.0)


def test_joint_table_matches_the_measured_hardware():
    assert {joint.motor_id for joint in ACTUATED_JOINTS} == set(MEASURED_JOINTS)
    for joint in ACTUATED_JOINTS:
        name, model, channel, lower, upper = MEASURED_JOINTS[joint.motor_id]
        assert joint.model_name == name
        assert joint.motor_model == model
        assert joint.channel == channel
        assert joint.lower == pytest.approx(lower)
        assert joint.upper == pytest.approx(upper)
        assert channel_for_motor_id(joint.motor_id) == channel
    with pytest.raises(ValueError):
        channel_for_motor_id(99)


def test_joint_limits_match_the_description_urdf():
    try:
        description = resolve_repo(("robonex-description",), "ROBONEX_DESCRIPTION_ROOT")
    except Exception:
        pytest.skip("robonex-description checkout not available")
    urdf = description / "urdf/robonex.urdf"
    if not urdf.is_file():
        pytest.skip("urdf/robonex.urdf not found")
    import xml.etree.ElementTree as ET

    limits = {}
    for joint in ET.parse(urdf).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    for joint in ACTUATED_JOINTS:
        assert joint.model_name in limits, joint.model_name
        assert (joint.lower, joint.upper) == pytest.approx(limits[joint.model_name], abs=1e-6), joint.model_name


def test_every_right_joint_is_the_sign_mirror_of_its_left_counterpart():
    by_name = {joint.model_name: joint for joint in ACTUATED_JOINTS}
    pairs = [(name, "r_" + name[2:]) for name in by_name if name.startswith("l_")]
    assert len(pairs) == 6
    for left_name, right_name in pairs:
        left, right = by_name[left_name], by_name[right_name]
        assert (right.lower, right.upper) == pytest.approx((-left.upper, -left.lower)), left_name
        assert left.motor_model == right.motor_model


def test_actuator_parameters_carry_the_measured_gains_and_physics():
    stiffness, damping = MEASURED_GAINS
    assert (MOTOR_CONTROL_KP, MOTOR_CONTROL_KD) == pytest.approx(MEASURED_GAINS)
    for model, (armature, friction) in MEASURED_PHYSICS.items():
        assert MOTOR_PHYSICS[model]["armature"] == pytest.approx(armature)
        assert MOTOR_PHYSICS[model]["frictionloss"] == pytest.approx(friction)
        assert ACTUATOR_PARAMETERS[model] == pytest.approx(
            {"stiffness": stiffness, "damping": damping, "armature": armature, "friction": friction}
        )


def test_decode_fault_bits_reads_single_bits():
    assert decode_fault_bits(0) == []
    assert decode_fault_bits(1 << 2) == [FAULT_BIT_NAMES[2]]
    assert set(decode_fault_bits((1 << 2) | (1 << 14))) == {FAULT_BIT_NAMES[2], FAULT_BIT_NAMES[14]}


def test_imu_constants_are_the_measured_values():
    assert MOUNT_ROLL_DEG == 180.0
    assert DEFAULT_IMU_BAUDRATE == 921600
    assert DEFAULT_IMU_PORT.startswith("/dev/")


def test_resolve_repo_prefers_an_explicit_path(tmp_path):
    target = tmp_path / "robonex-description"
    target.mkdir()
    assert resolve_repo(("robonex-description",), explicit=target) == target.resolve()
    with pytest.raises(FileNotFoundError):
        resolve_repo(("robonex-description",), explicit=tmp_path / "missing")


def test_resolve_repo_accepts_legacy_directory_names(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    legacy = tmp_path / "nest" / "robonex_description"
    legacy.mkdir(parents=True)
    found = resolve_repo(
        ("robonex-description", "robonex_description"),
        anchors=(legacy / "inner",),
    )
    assert found == legacy


def test_resolve_repo_searches_cwd_before_the_caller_anchor(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    near = tmp_path / "robonex-description"
    near.mkdir()
    far = tmp_path / "nest" / "robonex-description"
    far.mkdir(parents=True)
    assert resolve_repo(("robonex-description",), anchors=(far / "inner",)) == near


def test_resolve_repo_reports_the_environment_variable():
    with pytest.raises(FileNotFoundError) as error:
        resolve_repo(("robonex-nonexistent",), env_var="ROBONEX_NONEXISTENT_ROOT")
    assert "ROBONEX_NONEXISTENT_ROOT" in str(error.value)


np = pytest.importorskip("numpy")


def _contract():
    from robonex_common.limits import action_normalization
    from robonex_common.joints import POLICY_JOINT_ORDER
    from robonex_common.policy import PolicyContract

    offsets, scales, clips = action_normalization(0.01)
    return PolicyContract(
        schema_version=1,
        task="test",
        policy_file="p.onnx",
        policy_sha256="0" * 64,
        description_model="mujoco/basic/scene.xml",
        joint_order=POLICY_JOINT_ORDER,
        observation_terms=("joint_pos_rel:12",),
        action_offsets=tuple(offsets[n] for n in POLICY_JOINT_ORDER),
        action_scales=tuple(scales[n] for n in POLICY_JOINT_ORDER),
        target_clips=tuple(clips[n] for n in POLICY_JOINT_ORDER),
        runner_action_clip=3.0,
        observation_size=42,
        action_size=12,
        policy_hz=50.0,
        description_commit="a" * 40,
        common_commit="b" * 40,
        training_commit="c" * 40,
    )


def test_action_pipeline_clips_at_both_stages():
    from robonex_common.runtime import ActionPipeline

    pipeline = ActionPipeline(_contract())
    clipped, targets = pipeline.apply(np.full(12, 50.0, dtype=np.float32))
    assert np.all(clipped == 3.0)
    assert np.all(targets <= np.asarray([pair[1] for pair in _contract().target_clips]) + 1e-6)
    assert pipeline.policy_call_count == 1
    assert pipeline.runner_clip_count == 12


def test_action_pipeline_applies_offset_not_just_scale():
    from robonex_common.runtime import ActionPipeline

    pipeline = ActionPipeline(_contract())
    _, targets = pipeline.apply(np.zeros(12, dtype=np.float32))
    assert np.allclose(targets, np.asarray(_contract().action_offsets, dtype=np.float32))


def test_action_pipeline_rejects_bad_shapes_and_values():
    from robonex_common.runtime import ActionPipeline

    pipeline = ActionPipeline(_contract())
    with pytest.raises(ValueError):
        pipeline.apply(np.zeros(11))
    with pytest.raises(ValueError):
        pipeline.apply(np.full(12, np.nan))
    with pytest.raises(ValueError):
        pipeline.apply(np.full(12, 9.0), max_raw_action=3.0)


def test_assemble_observation_orders_and_validates():
    from robonex_common.runtime import assemble_observation

    observation = assemble_observation(
        np.arange(12), np.arange(12) + 100, (1, 2, 3), (0, 0, -1), np.arange(12) + 200
    )
    assert observation.shape == (42,)
    assert observation[0] == 0 and observation[12] == 100
    assert tuple(observation[24:27]) == (1, 2, 3)
    assert tuple(observation[27:30]) == (0, 0, -1)
    assert observation[30] == 200
    with pytest.raises(ValueError):
        assemble_observation(np.arange(11), np.arange(12), (1, 2, 3), (0, 0, -1), np.arange(12))
    with pytest.raises(ValueError):
        assemble_observation(np.full(12, np.nan), np.arange(12), (1, 2, 3), (0, 0, -1), np.arange(12))


def test_pyproject_version_matches_the_package():
    pyproject = (Path(__file__).resolve().parents[1] / "pyproject.toml").read_text()
    declared = [
        line.split("=", 1)[1].strip().strip('"')
        for line in pyproject.splitlines()
        if line.startswith("version =")
    ]
    assert declared == [robonex_common.__version__]
