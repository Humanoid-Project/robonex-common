from dataclasses import dataclass


@dataclass(frozen=True)
class JointSpec:
    motor_id: int
    model_name: str
    hardware_name: str
    motor_model: str
    channel: str
    lower: float
    upper: float


ACTUATED_JOINTS = (
    JointSpec(1, "l_hip_yaw_joint", "left_hip_yaw", "rs02", "can0", -0.698132, 0.698132),
    JointSpec(2, "l_hip_pitch_joint", "left_hip_pitch", "rs03", "can0", -0.872665, 0.872665),
    JointSpec(3, "l_hip_roll_joint", "left_hip_roll", "rs03", "can0", -1.047198, 0.087266),
    JointSpec(4, "l_knee_pitch_joint", "left_knee_pitch", "rs03", "can0", -0.872665, 0.087266),
    JointSpec(5, "l_ankle_upper_joint", "left_ankle_upper", "rs02", "can0", -0.610865, 0.436332),
    JointSpec(6, "l_ankle_lower_joint", "left_ankle_lower", "rs02", "can0", -0.436332, 0.610865),
    JointSpec(7, "r_hip_yaw_joint", "right_hip_yaw", "rs02", "can1", -0.698132, 0.698132),
    JointSpec(8, "r_hip_pitch_joint", "right_hip_pitch", "rs03", "can1", -0.872665, 0.872665),
    JointSpec(9, "r_hip_roll_joint", "right_hip_roll", "rs03", "can1", -0.087266, 1.047198),
    JointSpec(10, "r_knee_pitch_joint", "right_knee_pitch", "rs03", "can1", -0.087266, 0.872665),
    JointSpec(11, "r_ankle_upper_joint", "right_ankle_upper", "rs02", "can1", -0.436332, 0.610865),
    JointSpec(12, "r_ankle_lower_joint", "right_ankle_lower", "rs02", "can1", -0.610865, 0.436332),
)

JOINT_BY_ID = {joint.motor_id: joint for joint in ACTUATED_JOINTS}
JOINT_BY_MODEL_NAME = {joint.model_name: joint for joint in ACTUATED_JOINTS}
JOINT_BY_HARDWARE_NAME = {joint.hardware_name: joint for joint in ACTUATED_JOINTS}
JOINT_LIMITS_BY_ID = {joint.motor_id: (joint.lower, joint.upper) for joint in ACTUATED_JOINTS}
JOINT_LIMITS_BY_NAME = {joint.model_name: (joint.lower, joint.upper) for joint in ACTUATED_JOINTS}
CHANNEL_MOTOR_IDS = {
    "can0": tuple(joint.motor_id for joint in ACTUATED_JOINTS if joint.channel == "can0"),
    "can1": tuple(joint.motor_id for joint in ACTUATED_JOINTS if joint.channel == "can1"),
}
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


def channel_for_motor_id(motor_id):
    joint = JOINT_BY_ID.get(motor_id)
    if joint is None:
        raise ValueError(f"No CAN channel for motor ID {motor_id}")
    return joint.channel
