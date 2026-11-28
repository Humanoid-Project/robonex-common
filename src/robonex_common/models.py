import math
from dataclasses import dataclass, field, replace

from .joints import (
    ACTUATED_JOINTS,
    ALL_MOTORS,
    AUXILIARY_JOINTS,
    DEFAULT_JOINT_POS,
    DEFAULT_JOINT_POS_LEAN25,
    JOINT_BY_MODEL_NAME,
    JOINT_LIMITS_BY_NAME,
    MOTOR_BY_ID,
    MOTOR_LIMITS_BY_ID,
    PASSIVE_CLOSED_LOOP_JOINTS,
    POLICY_JOINT_ORDER,
    STOP_LIMITS_BY_NAME,
    VARIANT_MOTOR_IDS,
    VER3_NAME_BY_ID,
    VER3_PASSIVE_CLOSED_LOOP_JOINTS,
    VER3_POLICY_JOINT_ORDER,
    canonical_variant,
)

VER2_NAME_BY_ID = {joint.motor_id: joint.model_name for joint in ALL_MOTORS}


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
    stop_limits: dict = field(default_factory=dict)
    names_by_id: dict = field(default_factory=lambda: dict(VER2_NAME_BY_ID))
    policy_joint_order: tuple[str, ...] = POLICY_JOINT_ORDER
    passive_joints: tuple[str, ...] = PASSIVE_CLOSED_LOOP_JOINTS
    hold_joint_pos: dict = field(default_factory=dict)

    def motor_id_for(self, name):
        for motor_id, model_name in self.names_by_id.items():
            if model_name == name:
                return motor_id
        raise KeyError(f"robot model {self.name} has no joint {name!r}")

    def joint_limits_by_id(self):
        return {self.motor_id_for(name): bounds for name, bounds in self.joint_limits.items()}

    def stop_limits_by_id(self):
        return {
            self.motor_id_for(name): self.stop_limits.get(name, bounds)
            for name, bounds in self.joint_limits.items()
        }

    def hold_joint_pos_by_id(self):
        return {self.motor_id_for(name): target for name, target in self.hold_joint_pos.items()}


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
    dict(STOP_LIMITS_BY_NAME),
)
VER2 = replace(VER2_EDU, name="ver2")


def _ver3_name(name):
    return VER3_NAME_BY_ID[JOINT_BY_MODEL_NAME[name].motor_id]


def _ver3_keys(values):
    return {_ver3_name(name): value for name, value in values.items()}


VER3 = RobotModel(
    "ver3",
    _ver3_keys(VER2_EDU.joint_limits),
    _ver3_keys(VER2_EDU.default_joint_pos),
    FootRollClip(
        limit=VER2_EDU.foot_roll.limit,
        coeffs=VER2_EDU.foot_roll.coeffs,
        pairs=tuple((_ver3_name(upper), _ver3_name(lower), sign) for upper, lower, sign in VER2_EDU.foot_roll.pairs),
    ),
    VARIANT_MOTOR_IDS["ver2"],
    "ver3",
    _ver3_keys(VER2_EDU.stop_limits),
    dict(VER3_NAME_BY_ID),
    VER3_POLICY_JOINT_ORDER,
    VER3_PASSIVE_CLOSED_LOOP_JOINTS,
    {VER3_NAME_BY_ID[joint.motor_id]: 0.0 for joint in AUXILIARY_JOINTS},
)

ROBOT_MODELS = {model.name: model for model in (VER2_EDU,)}
LEG_PROFILES = ROBOT_MODELS
VARIANTS = {model.name: model for model in (VER2,)}
VER3_MODELS = {model.name: model for model in (VER3,)}


def robot_model(name):
    model = VARIANTS.get(name) or ROBOT_MODELS.get(name) or VER3_MODELS.get(name)
    if model is None:
        known = ", ".join(sorted({*VARIANTS, *ROBOT_MODELS, *VER3_MODELS}))
        raise ValueError(f"unknown robot model {name!r}; known: {known}")
    return model


def leg_profile(name):
    model = robot_model(name)
    return VER3_MODELS[model.leg_profile] if model.leg_profile in VER3_MODELS else ROBOT_MODELS[model.leg_profile]


ACTION_POSES = {"lean25": dict(DEFAULT_JOINT_POS_LEAN25)}


def action_pose(model, pose="home"):
    profile = robot_model(model)
    if pose == "home":
        return dict(profile.default_joint_pos)
    if pose not in ACTION_POSES:
        raise ValueError(f"unknown action pose {pose!r}; known: home, {', '.join(sorted(ACTION_POSES))}")
    return {profile.names_by_id[JOINT_BY_MODEL_NAME[name].motor_id]: value for name, value in ACTION_POSES[pose].items()}


def _check(model):
    if set(model.names_by_id) != set(MOTOR_BY_ID) or len(set(model.names_by_id.values())) != len(model.names_by_id):
        raise ValueError(f"robot model {model.name} does not name every motor exactly once")
    names = {model.names_by_id[joint.motor_id] for joint in ACTUATED_JOINTS}
    if set(model.joint_limits) != names or set(model.default_joint_pos) != names:
        raise ValueError(f"robot model {model.name} does not cover every actuated joint")
    if len(model.policy_joint_order) != len(names) or set(model.policy_joint_order) != names:
        raise ValueError(f"robot model {model.name}: policy joint order does not list every actuated joint once")
    order_ids = tuple(model.motor_id_for(name) for name in model.policy_joint_order)
    if order_ids != tuple(JOINT_BY_MODEL_NAME[name].motor_id for name in POLICY_JOINT_ORDER):
        raise ValueError(f"robot model {model.name}: policy joint order differs from the motor order of POLICY_JOINT_ORDER")
    if not set(model.passive_joints).isdisjoint(model.names_by_id.values()):
        raise ValueError(f"robot model {model.name}: a passive joint carries a motor name")
    if model.foot_roll is not None and any(
        upper not in names or lower not in names for upper, lower, _ in model.foot_roll.pairs
    ):
        raise ValueError(f"robot model {model.name}: foot-roll pair names an unknown joint")
    auxiliary = {model.names_by_id[joint.motor_id] for joint in AUXILIARY_JOINTS}
    for name, target in model.hold_joint_pos.items():
        if name not in auxiliary:
            raise ValueError(f"robot model {model.name}: hold target for {name} is not an auxiliary motor")
        lower, upper = MOTOR_LIMITS_BY_ID[model.motor_id_for(name)]
        if not lower < target < upper:
            raise ValueError(f"robot model {model.name}: hold target of {name} is outside its limits")
    for name, (lower, upper) in model.joint_limits.items():
        if not lower < model.default_joint_pos[name] < upper:
            raise ValueError(f"robot model {model.name}: default pose of {name} is outside its limits")
    if not set(model.stop_limits) <= names:
        raise ValueError(f"robot model {model.name} gives a stop limit for an unknown joint")
    for name, (lower, upper) in model.stop_limits.items():
        if not lower < model.default_joint_pos[name] < upper:
            raise ValueError(f"robot model {model.name}: default pose of {name} is outside its stop limit")
        command_lower, command_upper = model.joint_limits[name]
        if not lower <= command_lower < command_upper <= upper:
            raise ValueError(f"robot model {model.name}: command limit of {name} is not inside its stop limit")


def _check_variant(model):
    expected = VARIANT_MOTOR_IDS.get(canonical_variant(model.name))
    if model.motor_ids != expected or not set(model.motor_ids) <= set(MOTOR_BY_ID):
        raise ValueError(f"robot variant {model.name}: motor_ids do not match VARIANT_MOTOR_IDS")
    if not {joint.motor_id for joint in ACTUATED_JOINTS} <= set(model.motor_ids):
        raise ValueError(f"robot variant {model.name} is missing a leg motor")
    legs = ROBOT_MODELS.get(model.leg_profile)
    if legs is None or (
        model.joint_limits, model.default_joint_pos, model.foot_roll, model.stop_limits, model.names_by_id,
        model.policy_joint_order, model.passive_joints,
    ) != (
        legs.joint_limits, legs.default_joint_pos, legs.foot_roll, legs.stop_limits, legs.names_by_id,
        legs.policy_joint_order, legs.passive_joints,
    ):
        raise ValueError(f"robot variant {model.name} does not share its leg profile {model.leg_profile!r}")


for _model in ROBOT_MODELS.values():
    _check(_model)
if set(VARIANTS) != set(VARIANT_MOTOR_IDS):
    raise ValueError("VARIANTS and VARIANT_MOTOR_IDS name different robots")
for _model in VARIANTS.values():
    _check(_model)
    _check_variant(_model)
for _model in VER3_MODELS.values():
    _check(_model)
    if _model.motor_ids != VARIANT_MOTOR_IDS["ver2"] or _model.leg_profile != _model.name:
        raise ValueError(f"robot model {_model.name}: motor set or leg profile is not the ver2 layout")
for _name in ACTION_POSES:
    for _model in (*VARIANTS.values(), *VER3_MODELS.values()):
        _check(replace(_model, name=f"{_model.name}:{_name}", default_joint_pos=action_pose(_model.name, _name)))
