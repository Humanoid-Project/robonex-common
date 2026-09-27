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


VER1 = RobotModel("ver1", dict(JOINT_LIMITS_BY_NAME), dict(DEFAULT_JOINT_POS))

VER2_EDU = RobotModel(
    "ver2_edu",
    {
        "l_hip_yaw_joint": (-0.837758, 0.837758),
        "l_hip_pitch_joint": (-1.658063, 1.658063),
        "l_hip_roll_joint": (-2.094395, 0.174533),
        "l_knee_pitch_joint": (-1.22173, 0.174533),
        "l_ankle_upper_joint": (-0.279253, 0.872665),
        "l_ankle_lower_joint": (-0.872665, 0.523599),
        "r_hip_yaw_joint": (-0.837758, 0.837758),
        "r_hip_pitch_joint": (-1.658063, 1.658063),
        "r_hip_roll_joint": (-0.174533, 2.094395),
        "r_knee_pitch_joint": (-0.174533, 1.22173),
        "r_ankle_upper_joint": (-0.872665, 0.279253),
        "r_ankle_lower_joint": (-0.523599, 0.872665),
    },
    {
        "l_hip_yaw_joint": 0.0,
        "l_hip_pitch_joint": 0.1,
        "l_hip_roll_joint": 0.0,
        "l_knee_pitch_joint": -0.3298656951565011,
        "l_ankle_upper_joint": 0.21255773798848565,
        "l_ankle_lower_joint": -0.2038048914121279,
        "r_hip_yaw_joint": 0.0,
        "r_hip_pitch_joint": -0.1,
        "r_hip_roll_joint": 0.0,
        "r_knee_pitch_joint": 0.3298656951565011,
        "r_ankle_upper_joint": -0.21255773798848565,
        "r_ankle_lower_joint": 0.2038048914121279,
    },
    FootRollClip(
        limit=math.radians(12.0),
        coeffs=(-0.001195, 0.498159, 0.491024, -0.105182, -0.031960, 0.006356),
        pairs=(("l_ankle_upper_joint", "l_ankle_lower_joint", 1.0), ("r_ankle_upper_joint", "r_ankle_lower_joint", -1.0)),
    ),
)

ROBOT_MODELS = {model.name: model for model in (VER1, VER2_EDU)}


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
