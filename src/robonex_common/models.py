import math
from dataclasses import dataclass

from .joints import ACTUATED_JOINTS, DEFAULT_JOINT_POS, JOINT_BY_MODEL_NAME, JOINT_LIMITS_BY_NAME


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
)

ROBOT_MODELS = {model.name: model for model in (VER2_EDU,)}


def robot_model(name):
    model = ROBOT_MODELS.get(name)
    if model is None:
        raise ValueError(f"unknown robot model {name!r}; known: {', '.join(sorted(ROBOT_MODELS))}")
    return model


def _check(model):
    names = {joint.model_name for joint in ACTUATED_JOINTS}
    if set(model.joint_limits) != names or set(model.default_joint_pos) != names:
        raise ValueError(f"robot model {model.name} does not cover every actuated joint")
    for name, (lower, upper) in model.joint_limits.items():
        if not lower < model.default_joint_pos[name] < upper:
            raise ValueError(f"robot model {model.name}: default pose of {name} is outside its limits")


for _model in ROBOT_MODELS.values():
    _check(_model)
