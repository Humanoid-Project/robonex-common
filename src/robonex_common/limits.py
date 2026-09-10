from .joints import DEFAULT_JOINT_POS, JOINT_LIMITS_BY_ID, JOINT_LIMITS_BY_NAME


DEFAULT_LIMIT_MARGIN_RAD = 0.05
DEFAULT_ACTION_MARGIN_RAD = 0.01
ACTION_SCALE_RAD = {
    "l_hip_yaw_joint": 0.117718,
    "l_hip_pitch_joint": 0.116809,
    "l_hip_roll_joint": 0.031698,
    "l_knee_pitch_joint": 0.078943,
    "l_ankle_upper_joint": 0.025735,
    "l_ankle_lower_joint": 0.028228,
    "r_hip_yaw_joint": 0.117718,
    "r_hip_pitch_joint": 0.116809,
    "r_hip_roll_joint": 0.031698,
    "r_knee_pitch_joint": 0.078943,
    "r_ankle_upper_joint": 0.025735,
    "r_ankle_lower_joint": 0.028228,
}
RUNNER_ACTION_CLIP = 14.0


def joint_limit_for(motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lower, upper = JOINT_LIMITS_BY_ID[motor_id]
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError(f"invalid joint-limit margin for motor {motor_id}: {margin}")
    return lower + margin, upper - margin


def exceeds_joint_limit(position, motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lower, upper = joint_limit_for(motor_id, margin)
    return position <= lower or position >= upper


def action_normalization(margin=DEFAULT_ACTION_MARGIN_RAD):
    offsets = {}
    scales = {}
    clips = {}
    for name, (lower, upper) in JOINT_LIMITS_BY_NAME.items():
        clip_lower = lower + margin
        clip_upper = upper - margin
        if margin < 0.0 or clip_lower >= clip_upper:
            raise ValueError(f"invalid action margin for {name}: {margin}")
        default = DEFAULT_JOINT_POS[name]
        if not clip_lower <= default <= clip_upper:
            raise ValueError(f"default pose for {name} is outside its clipped range: {default}")
        scale = ACTION_SCALE_RAD[name]
        if scale <= 0.0:
            raise ValueError(f"invalid action scale for {name}: {scale}")
        nearest_clip = min(default - clip_lower, clip_upper - default)
        if scale * RUNNER_ACTION_CLIP > nearest_clip:
            raise ValueError(f"action scale for {name} creates a target-clip dead zone: {scale}")
        offsets[name] = default
        scales[name] = scale
        clips[name] = (clip_lower, clip_upper)
    return offsets, scales, clips


def action_limit_reach(margin=DEFAULT_ACTION_MARGIN_RAD):
    offsets, scales, clips = action_normalization(margin)
    return {
        name: ((clips[name][0] - offsets[name]) / scales[name],
               (clips[name][1] - offsets[name]) / scales[name])
        for name in clips
    }
