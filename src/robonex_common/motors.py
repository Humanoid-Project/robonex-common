from dataclasses import dataclass


@dataclass(frozen=True)
class MotorSpec:
    name: str
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    t_min: float
    t_max: float
    kp_max: float
    kd_max: float


MOTOR_SPECS = {
    "rs02": MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0, 500.0, 5.0),
    "rs03": MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0, 5000.0, 100.0),
}
MOTOR_CONTROL_KP = 40.0
MOTOR_CONTROL_KD = 2.0
JOINT_CONTROL_GAINS = {
    "hip_yaw": (100.0, 2.0),
    "hip_pitch": (100.0, 2.0),
    "hip_roll": (100.0, 2.0),
    "knee_pitch": (150.0, 4.0),
    "ankle_upper": (40.0, 2.0),
    "ankle_lower": (40.0, 2.0),
}
MOTOR_PHYSICS = {
    "rs02": {
        "armature": 0.0042,
        "frictionloss": 0.0,
        "static_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "rs03": {
        "armature": 0.02,
        "frictionloss": 0.0,
        "static_friction": 0.0,
        "viscous_friction": 0.0,
    },
}
RATED_TORQUE = {"rs02": 6.0, "rs03": 13.0}
PEAK_TORQUE = {"rs02": 17.0, "rs03": 60.0}
NO_LOAD_SPEED = {"rs02": 42.9, "rs03": 20.9}
VELOCITY_LIMIT_DERATE = 0.9
VELOCITY_LIMIT = {
    model: round(speed * VELOCITY_LIMIT_DERATE, 2) for model, speed in NO_LOAD_SPEED.items()
}
DEFAULT_VELOCITY_LIMIT_CURRENT = {"rs02": 4.0, "rs03": 8.0}
DEFAULT_VELOCITY_ACCELERATION = {"rs02": 20.0, "rs03": 10.0}
