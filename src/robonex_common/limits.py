from .joints import JOINT_LIMITS_BY_ID, JOINT_LIMITS_BY_NAME


DEFAULT_LIMIT_MARGIN_RAD = 0.05


def joint_limit_for(motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lower, upper = JOINT_LIMITS_BY_ID[motor_id]
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError(f"invalid joint-limit margin for motor {motor_id}: {margin}")
    return lower + margin, upper - margin


def exceeds_joint_limit(position, motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lower, upper = joint_limit_for(motor_id, margin)
    return position <= lower or position >= upper


def action_normalization(margin=0.01):
    offsets = {}
    scales = {}
    clips = {}
    for name, (lower, upper) in JOINT_LIMITS_BY_NAME.items():
        clip_lower = lower + margin
        clip_upper = upper - margin
        if margin < 0.0 or clip_lower >= clip_upper:
            raise ValueError(f"invalid action margin for {name}: {margin}")
        offsets[name] = (clip_lower + clip_upper) * 0.5
        scales[name] = (clip_upper - clip_lower) * 0.5
        clips[name] = (clip_lower, clip_upper)
    return offsets, scales, clips
