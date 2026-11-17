import math

import numpy as np

FOOT_ROLL_ITERATIONS = 8


def foot_roll(upper, lower, coeffs):
    c0, c1, c2, c3, c4, c5 = coeffs
    return c0 + c1 * upper + c2 * lower + c3 * upper * upper + c4 * upper * lower + c5 * lower * lower


def _clip_foot_roll_numpy(upper, lower, upper_range, lower_range, coeffs, limit, iterations=FOOT_ROLL_ITERATIONS):
    upper = np.asarray(upper, dtype=np.float64)
    lower = np.asarray(lower, dtype=np.float64)
    _, c1, c2, c3, c4, c5 = coeffs
    for _ in range(iterations):
        roll = foot_roll(upper, lower, coeffs)
        excess = roll - np.clip(roll, -limit, limit)
        d_upper = c1 + 2.0 * c3 * upper + c4 * lower
        d_lower = c2 + c4 * upper + 2.0 * c5 * lower
        step = excess / (d_upper * d_upper + d_lower * d_lower)
        upper = np.clip(upper - step * d_upper, upper_range[0], upper_range[1])
        lower = np.clip(lower - step * d_lower, lower_range[0], lower_range[1])
    return upper, lower


_SCALARS = (float, np.float64)
_SEQUENCES = (tuple, list)


def _clip_foot_roll_scalar(upper, lower, upper_range, lower_range, coeffs, limit, iterations):
    """Pure-float twin of _clip_foot_roll_numpy for scalar inputs, bit-identical to it.

    Returns None whenever it cannot promise that; the caller then uses the numpy body."""
    if type(iterations) is not int or iterations < 1:
        return None
    if type(upper_range) not in _SEQUENCES or type(lower_range) not in _SEQUENCES or type(coeffs) not in _SEQUENCES:
        return None
    if len(upper_range) != 2 or len(lower_range) != 2 or len(coeffs) != 6:
        return None
    values = (upper, lower, limit, *upper_range, *lower_range, *coeffs)
    for value in values:
        if type(value) not in _SCALARS:
            return None
    try:
        u, l, limit, u_lo, u_hi, l_lo, l_hi, c0, c1, c2, c3, c4, c5 = map(float, values)
    except (TypeError, ValueError, OverflowError):
        return None
    isfinite = math.isfinite
    if not all(map(isfinite, (u, l, limit, u_lo, u_hi, l_lo, l_hi, c0, c1, c2, c3, c4, c5))):
        return None
    for _ in range(iterations):
        roll = c0 + c1 * u + c2 * l + c3 * u * u + c4 * u * l + c5 * l * l
        excess = roll - min(max(roll, -limit), limit)
        d_upper = c1 + 2.0 * c3 * u + c4 * l
        d_lower = c2 + c4 * u + 2.0 * c5 * l
        denom = d_upper * d_upper + d_lower * d_lower
        if not (isfinite(roll) and isfinite(excess) and isfinite(denom)) or denom == 0.0:
            return None
        step = excess / denom
        new_u = u - step * d_upper
        new_l = l - step * d_lower
        if not (isfinite(step) and isfinite(new_u) and isfinite(new_l)):
            return None
        u = min(max(new_u, u_lo), u_hi)
        l = min(max(new_l, l_lo), l_hi)
    return np.float64(u), np.float64(l)


def clip_foot_roll(upper, lower, upper_range, lower_range, coeffs, limit, iterations=FOOT_ROLL_ITERATIONS):
    fast = _clip_foot_roll_scalar(upper, lower, upper_range, lower_range, coeffs, limit, iterations)
    if fast is not None:
        return fast
    return _clip_foot_roll_numpy(upper, lower, upper_range, lower_range, coeffs, limit, iterations)
