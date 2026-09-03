from .motors import MOTOR_CONTROL_KD, MOTOR_CONTROL_KP, MOTOR_PHYSICS

ACTUATOR_PARAMETERS = {
    model: {
        "stiffness": MOTOR_CONTROL_KP,
        "damping": MOTOR_CONTROL_KD,
        "armature": values["armature"],
        "friction": values["frictionloss"],
    }
    for model, values in MOTOR_PHYSICS.items()
}
