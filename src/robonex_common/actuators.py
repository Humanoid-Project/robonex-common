from .joints import ACTUATED_JOINTS
from .motors import (
    JOINT_CONTROL_GAINS,
    MOTOR_PHYSICS,
    PEAK_TORQUE,
    VELOCITY_LIMIT,
)


def _joint_role(model_name):
    return model_name.removeprefix("l_").removeprefix("r_").removesuffix("_joint")


CONTROL_GAINS_BY_JOINT = {
    joint.model_name: JOINT_CONTROL_GAINS[_joint_role(joint.model_name)] for joint in ACTUATED_JOINTS
}

ACTUATOR_PARAMETERS = {
    model: {
        "stiffness": {
            joint.model_name: CONTROL_GAINS_BY_JOINT[joint.model_name][0]
            for joint in ACTUATED_JOINTS
            if joint.motor_model == model
        },
        "damping": {
            joint.model_name: CONTROL_GAINS_BY_JOINT[joint.model_name][1]
            for joint in ACTUATED_JOINTS
            if joint.motor_model == model
        },
        "armature": values["armature"],
        "friction": values["static_friction"],
        "dynamic_friction": values["frictionloss"],
        "viscous_friction": values["viscous_friction"],
        "effort_limit_sim": PEAK_TORQUE[model],
        "velocity_limit_sim": VELOCITY_LIMIT[model],
    }
    for model, values in MOTOR_PHYSICS.items()
}
