from dataclasses import dataclass

from .buses import GROUPS, bus_map, channel_for_group


@dataclass(frozen=True)
class JointSpec:
    motor_id: int
    model_name: str
    hardware_name: str
    motor_model: str
    group: str
    lower: float
    upper: float

    @property
    def channel(self):
        return channel_for_group(self.group)


ACTUATED_JOINTS = (
    JointSpec(1, "l_hip_yaw_joint", "left_hip_yaw", "rs02", "left_leg", -0.837758, 0.837758),
    JointSpec(2, "l_hip_pitch_joint", "left_hip_pitch", "rs03", "left_leg", -1.658063, 1.658063),
    JointSpec(3, "l_hip_roll_joint", "left_hip_roll", "rs03", "left_leg", -2.094395, 0.174533),
    JointSpec(4, "l_knee_pitch_joint", "left_knee_pitch", "rs03", "left_leg", -1.22173, 0.174533),
    JointSpec(5, "l_ankle_upper_joint", "left_ankle_upper", "rs02", "left_leg", -0.279253, 0.872665),
    JointSpec(6, "l_ankle_lower_joint", "left_ankle_lower", "rs02", "left_leg", -0.872665, 0.523599),
    JointSpec(7, "r_hip_yaw_joint", "right_hip_yaw", "rs02", "right_leg", -0.837758, 0.837758),
    JointSpec(8, "r_hip_pitch_joint", "right_hip_pitch", "rs03", "right_leg", -1.658063, 1.658063),
    JointSpec(9, "r_hip_roll_joint", "right_hip_roll", "rs03", "right_leg", -0.174533, 2.094395),
    JointSpec(10, "r_knee_pitch_joint", "right_knee_pitch", "rs03", "right_leg", -0.174533, 1.22173),
    JointSpec(11, "r_ankle_upper_joint", "right_ankle_upper", "rs02", "right_leg", -0.872665, 0.279253),
    JointSpec(12, "r_ankle_lower_joint", "right_ankle_lower", "rs02", "right_leg", -0.523599, 0.872665),
)

AUXILIARY_JOINTS = (
    JointSpec(13, "neck_pitch_joint", "neck_pitch", "rs05", "head", -0.523599, 0.523599),
    JointSpec(14, "neck_yaw_joint", "neck_yaw", "rs05", "head", -0.523599, 0.523599),
    JointSpec(15, "l_shoulder_pitch_joint", "left_shoulder_pitch", "rs02", "left_arm", -0.785398, 0.785398),
    JointSpec(16, "l_shoulder_roll_joint", "left_shoulder_roll", "rs02", "left_arm", -0.785398, 0.785398),
    JointSpec(17, "l_shoulder_yaw_joint", "left_shoulder_yaw", "rs02", "left_arm", -0.785398, 0.785398),
    JointSpec(18, "l_elbow_joint", "left_elbow", "rs02", "left_arm", -0.785398, 0.785398),
    JointSpec(20, "r_shoulder_pitch_joint", "right_shoulder_pitch", "rs02", "right_arm", -0.785398, 0.785398),
    JointSpec(21, "r_shoulder_roll_joint", "right_shoulder_roll", "rs02", "right_arm", -0.785398, 0.785398),
    JointSpec(22, "r_shoulder_yaw_joint", "right_shoulder_yaw", "rs02", "right_arm", -0.785398, 0.785398),
    JointSpec(23, "r_elbow_joint", "right_elbow", "rs02", "right_arm", -0.785398, 0.785398),
)
ALL_MOTORS = ACTUATED_JOINTS + AUXILIARY_JOINTS
GROUP_ID_RANGES = {
    "left_leg": range(1, 7),
    "right_leg": range(7, 13),
    "head": range(13, 15),
    "left_arm": range(15, 19),
    "right_arm": range(20, 24),
}
VARIANT_MOTOR_IDS = {
    "ver2_edu": tuple(range(1, 14)),
    "ver2_pro": tuple(range(1, 14)) + tuple(range(15, 19)) + tuple(range(20, 24)),
    "ver2_max": tuple(range(1, 15)) + tuple(range(15, 19)) + tuple(range(20, 24)),
}

JOINT_BY_ID = {joint.motor_id: joint for joint in ACTUATED_JOINTS}
MOTOR_BY_ID = {joint.motor_id: joint for joint in ALL_MOTORS}
MOTOR_LIMITS_BY_ID = {joint.motor_id: (joint.lower, joint.upper) for joint in ALL_MOTORS}
JOINT_BY_MODEL_NAME = {joint.model_name: joint for joint in ACTUATED_JOINTS}
JOINT_BY_HARDWARE_NAME = {joint.hardware_name: joint for joint in ACTUATED_JOINTS}
JOINT_LIMITS_BY_ID = {joint.motor_id: (joint.lower, joint.upper) for joint in ACTUATED_JOINTS}
JOINT_LIMITS_BY_NAME = {joint.model_name: (joint.lower, joint.upper) for joint in ACTUATED_JOINTS}
DEFAULT_JOINT_POS = {
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
}


def motor_ids_by_channel(motors=ACTUATED_JOINTS, mapping=None):
    mapping = bus_map() if mapping is None else mapping
    channels = {}
    for joint in motors:
        channels.setdefault(channel_for_group(joint.group, mapping), []).append(joint.motor_id)
    return {channel: tuple(ids) for channel, ids in channels.items()}


CHANNEL_MOTOR_IDS = motor_ids_by_channel()
ALL_CHANNEL_MOTOR_IDS = motor_ids_by_channel(ALL_MOTORS)
POLICY_JOINT_ORDER = (
    "l_hip_yaw_joint",
    "r_hip_yaw_joint",
    "l_hip_pitch_joint",
    "r_hip_pitch_joint",
    "l_hip_roll_joint",
    "r_hip_roll_joint",
    "l_knee_pitch_joint",
    "r_knee_pitch_joint",
    "l_ankle_lower_joint",
    "l_ankle_upper_joint",
    "r_ankle_lower_joint",
    "r_ankle_upper_joint",
)
PASSIVE_CLOSED_LOOP_JOINTS = (
    "l_knee_joint",
    "r_knee_joint",
    "l_knee_coupler_joint_a",
    "r_knee_coupler_joint_a",
    "l_ankle_roll_joint",
    "r_ankle_roll_joint",
    "l_ankle_pitch_joint",
    "r_ankle_pitch_joint",
)


def motors_for_variant(name):
    ids = VARIANT_MOTOR_IDS.get(name)
    if ids is None:
        raise ValueError(f"unknown robot variant {name!r}; known: {', '.join(sorted(VARIANT_MOTOR_IDS))}")
    return tuple(MOTOR_BY_ID[motor_id] for motor_id in ids)


def channel_for_motor_id(motor_id):
    joint = MOTOR_BY_ID.get(motor_id)
    if joint is None:
        raise ValueError(f"No CAN channel for motor ID {motor_id}")
    return joint.channel
