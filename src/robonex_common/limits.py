import math

from .joints import AUXILIARY_JOINTS, JOINT_BY_ID, JOINT_LIMITS_BY_NAME, MOTOR_LIMITS_BY_ID
from .models import action_pose, robot_model


DEFAULT_LIMIT_MARGIN_RAD = 0.05
DEFAULT_ACTION_MARGIN_RAD = 0.01
RUNNER_ACTION_CLIP = 14.0
MAX_ACTION_SCALE_RAD = 0.25
ACTION_REACH_SIGMA = 3.0


def _action_scale(name, margin=DEFAULT_ACTION_MARGIN_RAD, model="ver2"):
    """Scale so the fence sits at least ``ACTION_REACH_SIGMA`` sigma away.

    The bent default pose leaves the ankles only ~0.36 rad of upward travel, so
    a uniform 0.25 rad scale puts their fence at 1.4 sigma and clips a quarter of
    all steps. Shrinking the global exploration instead would also halve it on
    hip_pitch and knee, which never clip and are the joints a step needs.
    The lower bound keeps every fence reachable inside RUNNER_ACTION_CLIP.
    """
    profile = robot_model(model)
    lower, upper = profile.joint_limits[name]
    default = profile.default_joint_pos[name]
    near = min(default - (lower + margin), (upper - margin) - default)
    far = max(default - (lower + margin), (upper - margin) - default)
    scale = max(near / ACTION_REACH_SIGMA, far / RUNNER_ACTION_CLIP)
    # round up: rounding down can leave the far fence a hair out of reach
    return min(MAX_ACTION_SCALE_RAD, math.ceil(scale * 1e6) / 1e6)


ACTION_SCALE_RAD = {name: _action_scale(name) for name in JOINT_LIMITS_BY_NAME}
def joint_limit_for(motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD, model="ver2"):
    profile = robot_model(model)
    limits = profile.joint_limits
    name = profile.names_by_id.get(motor_id) if motor_id in JOINT_BY_ID else None
    if name is not None and name in limits:
        lower, upper = limits[name]
    elif motor_id in MOTOR_LIMITS_BY_ID:
        lower, upper = MOTOR_LIMITS_BY_ID[motor_id]
    else:
        raise KeyError(f"no joint limit for motor {motor_id}")
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError(f"invalid joint-limit margin for motor {motor_id}: {margin}")
    return lower + margin, upper - margin


def exceeds_joint_limit(position, motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD, model="ver2"):
    lower, upper = joint_limit_for(motor_id, margin, model)
    return position <= lower or position >= upper


def stop_limit_for(motor_id, margin=0.0, model="ver2"):
    profile = robot_model(model)
    name = profile.names_by_id.get(motor_id) if motor_id in JOINT_BY_ID else None
    if name is not None and name in profile.stop_limits:
        lower, upper = profile.stop_limits[name]
    elif name is not None and name in profile.joint_limits:
        lower, upper = profile.joint_limits[name]
    elif motor_id in MOTOR_LIMITS_BY_ID:
        lower, upper = MOTOR_LIMITS_BY_ID[motor_id]
    else:
        raise KeyError(f"no stop limit for motor {motor_id}")
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError(f"invalid stop-limit margin for motor {motor_id}: {margin}")
    return lower + margin, upper - margin


def exceeds_stop_limit(position, motor_id, margin=0.0, model="ver2"):
    lower, upper = stop_limit_for(motor_id, margin, model)
    return position <= lower or position >= upper


def action_normalization(margin=DEFAULT_ACTION_MARGIN_RAD, model="ver2", pose="home"):
    profile = robot_model(model)
    defaults = action_pose(model, pose)
    offsets = {}
    scales = {}
    clips = {}
    for name, (lower, upper) in profile.joint_limits.items():
        clip_lower = lower + margin
        clip_upper = upper - margin
        if margin < 0.0 or clip_lower >= clip_upper:
            raise ValueError(f"invalid action margin for {name}: {margin}")
        default = defaults[name]
        if not clip_lower <= default <= clip_upper:
            raise ValueError(f"default pose for {name} is outside its clipped range: {default}")
        scale = _action_scale(name, margin=DEFAULT_ACTION_MARGIN_RAD, model=model)
        if scale <= 0.0:
            raise ValueError(f"invalid action scale for {name}: {scale}")
        farthest_clip = max(default - clip_lower, clip_upper - default)
        if scale * RUNNER_ACTION_CLIP < farthest_clip:
            raise ValueError(f"action scale for {name} cannot reach its target clip: {scale}")
        offsets[name] = default
        scales[name] = scale
        clips[name] = (clip_lower, clip_upper)
    return offsets, scales, clips


def action_limit_reach(margin=DEFAULT_ACTION_MARGIN_RAD, model="ver2"):
    offsets, scales, clips = action_normalization(margin, model)
    return {
        name: ((clips[name][0] - offsets[name]) / scales[name],
               (clips[name][1] - offsets[name]) / scales[name])
        for name in clips
    }


PLACEHOLDER_LIMIT_MOTOR_IDS = tuple(sorted(joint.motor_id for joint in AUXILIARY_JOINTS))
PLACEHOLDER_CONSENT_FLAG = "--allow-placeholder-limits"


class PlaceholderLimitError(ValueError):
    pass


def limit_status(motor_id):
    if motor_id in PLACEHOLDER_LIMIT_MOTOR_IDS:
        return "placeholder"
    if motor_id in JOINT_BY_ID:
        return "measured"
    raise ValueError(f"motor ID {motor_id} is not a registered motor")


def placeholder_motor_ids(motor_ids):
    return tuple(sorted({motor_id for motor_id in motor_ids if limit_status(motor_id) == "placeholder"}))


def require_placeholder_consent(motor_ids, allowed):
    found = placeholder_motor_ids(motor_ids)
    if found and not allowed:
        ids = ", ".join(str(motor_id) for motor_id in found)
        raise PlaceholderLimitError(
            f"motor IDs {ids} have placeholder limits (not measured); "
            f"pass {PLACEHOLDER_CONSENT_FLAG} to move them inside those limits"
        )
    return found
