import math
from dataclasses import dataclass, replace

from .joints import (
    ACTUATED_JOINTS,
    DEFAULT_JOINT_POS,
    DEFAULT_JOINT_POS_LEAN25,
    JOINT_BY_MODEL_NAME,
    JOINT_LIMITS_BY_NAME,
    MOTOR_BY_ID,
    VARIANT_MOTOR_IDS,
    canonical_variant,
)


@dataclass(frozen=True)
class FootRollClip:
    limit: float
    coeffs: tuple[float, ...]
    pairs: tuple[tuple[str, str, float], ...]


@dataclass(frozen=True)
class RobotModel:
    name: str
    joint_limits: dict
    default_joint_pos: dict
    foot_roll: FootRollClip | None = None
    motor_ids: tuple[int, ...] = ()
    leg_profile: str = ""

    def joint_limits_by_id(self):
        return {JOINT_BY_MODEL_NAME[name].motor_id: bounds for name, bounds in self.joint_limits.items()}


VER2_EDU = RobotModel(
    "ver2_edu",
    dict(JOINT_LIMITS_BY_NAME),
    dict(DEFAULT_JOINT_POS),
    FootRollClip(
        limit=math.radians(12.0),
        coeffs=(-0.001195, 0.498159, 0.491024, -0.105182, -0.031960, 0.006356),
        pairs=(("l_ankle_upper_joint", "l_ankle_lower_joint", 1.0), ("r_ankle_upper_joint", "r_ankle_lower_joint", -1.0)),
    ),
    VARIANT_MOTOR_IDS["ver2"],
    "ver2_edu",
)
VER2 = replace(VER2_EDU, name="ver2")

ROBOT_MODELS = {model.name: model for model in (VER2_EDU,)}
LEG_PROFILES = ROBOT_MODELS
VARIANTS = {model.name: model for model in (VER2,)}


def robot_model(name):
    model = VARIANTS.get(name) or ROBOT_MODELS.get(name)
    if model is None:
        raise ValueError(f"unknown robot model {name!r}; known: {', '.join(sorted({*VARIANTS, *ROBOT_MODELS}))}")
    return model


def leg_profile(name):
    return ROBOT_MODELS[robot_model(name).leg_profile]


ACTION_POSES = {"lean25": dict(DEFAULT_JOINT_POS_LEAN25)}


def action_pose(model, pose="home"):
    profile = robot_model(model)
    if pose == "home":
        return dict(profile.default_joint_pos)
    if pose not in ACTION_POSES:
        raise ValueError(f"unknown action pose {pose!r}; known: home, {', '.join(sorted(ACTION_POSES))}")
    return dict(ACTION_POSES[pose])


def _check(model):
    names = {joint.model_name for joint in ACTUATED_JOINTS}
    if set(model.joint_limits) != names or set(model.default_joint_pos) != names:
        raise ValueError(f"robot model {model.name} does not cover every actuated joint")
    for name, (lower, upper) in model.joint_limits.items():
        if not lower < model.default_joint_pos[name] < upper:
            raise ValueError(f"robot model {model.name}: default pose of {name} is outside its limits")


def _check_variant(model):
    expected = VARIANT_MOTOR_IDS.get(canonical_variant(model.name))
    if model.motor_ids != expected or not set(model.motor_ids) <= set(MOTOR_BY_ID):
        raise ValueError(f"robot variant {model.name}: motor_ids do not match VARIANT_MOTOR_IDS")
    if not {joint.motor_id for joint in ACTUATED_JOINTS} <= set(model.motor_ids):
        raise ValueError(f"robot variant {model.name} is missing a leg motor")
    legs = ROBOT_MODELS.get(model.leg_profile)
    if legs is None or (model.joint_limits, model.default_joint_pos, model.foot_roll) != (
        legs.joint_limits, legs.default_joint_pos, legs.foot_roll
    ):
        raise ValueError(f"robot variant {model.name} does not share its leg profile {model.leg_profile!r}")


for _model in ROBOT_MODELS.values():
    _check(_model)
if set(VARIANTS) != set(VARIANT_MOTOR_IDS):
    raise ValueError("VARIANTS and VARIANT_MOTOR_IDS name different robots")
for _model in VARIANTS.values():
    _check(_model)
    _check_variant(_model)
for _name, _pose in ACTION_POSES.items():
    for _model in VARIANTS.values():
        _check(replace(_model, name=f"{_model.name}:{_name}", default_joint_pos=_pose))
