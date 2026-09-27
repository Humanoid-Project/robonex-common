import numpy as np

FOOT_ROLL_ITERATIONS = 8


def foot_roll(upper, lower, coeffs):
    c0, c1, c2, c3, c4, c5 = coeffs
    return c0 + c1 * upper + c2 * lower + c3 * upper * upper + c4 * upper * lower + c5 * lower * lower


def clip_foot_roll(upper, lower, upper_range, lower_range, coeffs, limit, iterations=FOOT_ROLL_ITERATIONS):
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
