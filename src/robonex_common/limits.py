import math

from .joints import DEFAULT_JOINT_POS, JOINT_LIMITS_BY_ID, JOINT_LIMITS_BY_NAME


DEFAULT_LIMIT_MARGIN_RAD = 0.05
DEFAULT_ACTION_MARGIN_RAD = 0.01
RUNNER_ACTION_CLIP = 14.0
MAX_ACTION_SCALE_RAD = 0.25
ACTION_REACH_SIGMA = 3.0


def _action_scale(name, margin=DEFAULT_ACTION_MARGIN_RAD):
    """Scale so the fence sits at least ``ACTION_REACH_SIGMA`` sigma away.

    The bent default pose leaves the ankles only ~0.36 rad of upward travel, so
    a uniform 0.25 rad scale puts their fence at 1.4 sigma and clips a quarter of
    all steps. Shrinking the global exploration instead would also halve it on
    hip_pitch and knee, which never clip and are the joints a step needs.
    The lower bound keeps every fence reachable inside RUNNER_ACTION_CLIP.
    """
    lower, upper = JOINT_LIMITS_BY_NAME[name]
    default = DEFAULT_JOINT_POS[name]
    near = min(default - (lower + margin), (upper - margin) - default)
    far = max(default - (lower + margin), (upper - margin) - default)
    scale = max(near / ACTION_REACH_SIGMA, far / RUNNER_ACTION_CLIP)
    # round up: rounding down can leave the far fence a hair out of reach
    return min(MAX_ACTION_SCALE_RAD, math.ceil(scale * 1e6) / 1e6)


ACTION_SCALE_RAD = {name: _action_scale(name) for name in JOINT_LIMITS_BY_NAME}
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
        farthest_clip = max(default - clip_lower, clip_upper - default)
        if scale * RUNNER_ACTION_CLIP < farthest_clip:
            raise ValueError(f"action scale for {name} cannot reach its target clip: {scale}")
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
