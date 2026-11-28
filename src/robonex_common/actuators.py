from .joints import ACTUATED_JOINTS, ALL_MOTORS
from .motors import (
    JOINT_CONTROL_GAINS,
    MOTOR_PHYSICS,
    PEAK_TORQUE,
    VELOCITY_LIMIT,
)


def _joint_role(model_name):
    return model_name.removeprefix("l_").removeprefix("r_").removesuffix("_joint")


CONTROL_GAINS_BY_JOINT = {
    joint.model_name: JOINT_CONTROL_GAINS[_joint_role(joint.model_name)] for joint in ALL_MOTORS
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


def control_gains(model="ver2"):
    from .models import robot_model

    profile = robot_model(model)
    return {
        profile.names_by_id[joint.motor_id]: CONTROL_GAINS_BY_JOINT[joint.model_name]
        for joint in ALL_MOTORS
        if joint.motor_id in profile.motor_ids
    }
