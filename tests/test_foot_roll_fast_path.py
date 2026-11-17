"""The scalar fast path of clip_foot_roll must be bit-identical to the numpy body it replaces."""
import itertools
import json
import math
import os
import struct

import numpy as np
import pytest

import robonex_common.foot_roll as foot_roll_module
import robonex_common.runtime as runtime_module
from robonex_common.foot_roll import _clip_foot_roll_numpy, _clip_foot_roll_scalar, clip_foot_roll, foot_roll
from robonex_common.models import robot_model
from robonex_common.policy import PolicyContract
from robonex_common.runtime import ActionPipeline
from test_ver2_profile import _payload

ROLL = robot_model("ver2_edu").foot_roll
RANDOM_CASES = 200_000


def _pipeline():
    return ActionPipeline(PolicyContract.from_dict(_payload("ver2_edu")))


def _bits(value):
    if isinstance(value, np.ndarray):
        return value.tobytes()
    return struct.pack("<d", value)


def _same(got, expected):
    assert len(got) == len(expected) == 2
    for new, old in zip(got, expected):
        assert type(new) is type(old)
        assert np.shape(new) == np.shape(old)
        assert new.dtype == old.dtype
        assert _bits(new) == _bits(old)  # NaN results match too: same bits from the same numpy body


def _check(make_args):
    """Public function against the numpy oracle; make_args builds fresh arguments (iterators are single-use).

    Returns True when the public call went through the numpy body."""
    numpy_calls = []

    def spy(*args):
        numpy_calls.append(args)
        return _clip_foot_roll_numpy(*args)

    with np.errstate(all="ignore"):
        try:
            expected = _clip_foot_roll_numpy(*make_args())
        except Exception as error:  # the oracle decides; the public function must raise the same type
            expected = type(error)
        foot_roll_module._clip_foot_roll_numpy = spy
        try:
            if isinstance(expected, type):
                with pytest.raises(expected) as raised:
                    clip_foot_roll(*make_args())
                assert raised.type is expected
            else:
                _same(clip_foot_roll(*make_args()), expected)
        finally:
            foot_roll_module._clip_foot_roll_numpy = _clip_foot_roll_numpy
    assert len(numpy_calls) <= 1
    return bool(numpy_calls)


def _args(upper, lower, upper_range, lower_range, coeffs=ROLL.coeffs, limit=ROLL.limit, *more):
    return lambda: (upper, lower, upper_range, lower_range, coeffs, limit, *more)


def _roll_at(target, lower):
    """An upper angle whose foot roll is exactly `target` for this lower angle, or None."""
    low, high = -1.0, 1.0
    for _ in range(200):
        middle = 0.5 * (low + high)
        if foot_roll(middle, lower, ROLL.coeffs) < target:
            low = middle
        else:
            high = middle
    candidate = math.nextafter(low, -math.inf)
    for _ in range(200):
        if foot_roll(candidate, lower, ROLL.coeffs) == target:
            return candidate
        candidate = math.nextafter(candidate, math.inf)
    return None


def test_random_inputs_inside_and_outside_the_ankle_ranges():
    pipeline = _pipeline()
    assert [pair[2] for pair in pipeline.roll_pairs] == [1.0, -1.0]
    assert pipeline.roll_coeffs == ROLL.coeffs and pipeline.roll_limit == ROLL.limit
    rng = np.random.default_rng(20261117)
    fallbacks = 0
    per_pair = RANDOM_CASES // 2
    for _, _, sign, upper_range, lower_range in pipeline.roll_pairs:
        inside = np.column_stack([rng.uniform(*upper_range, per_pair // 2), rng.uniform(*lower_range, per_pair // 2)])
        outside = rng.uniform(-3.0, 3.0, (per_pair - per_pair // 2, 2))
        for index, (upper, lower) in enumerate(np.vstack([inside, outside])):
            if index % 2:  # the deploy passes np.float64 (sign * targets[i]); safety.py passes Python floats
                upper, lower = float(upper), float(lower)
            assert type(sign * upper) in (float, np.float64)
            fallbacks += _check(_args(upper, lower, upper_range, lower_range, pipeline.roll_coeffs, pipeline.roll_limit))
    assert fallbacks == 0


def test_boundary_grid():
    pipeline = _pipeline()
    exact = []
    for target, lower in itertools.product((ROLL.limit, -ROLL.limit), (-0.4, -0.1, 0.0, 0.1, 0.3, 0.5)):
        upper = _roll_at(target, lower)
        if upper is not None:
            exact.append((upper, lower))
    assert exact, "no input with roll exactly at the limit was found"
    fast = slow = 0
    for _, _, _, upper_range, lower_range in pipeline.roll_pairs:
        values = [0.0, -0.0, 5e-324, -5e-324, 2.2250738585072014e-308, -2.2250738585072014e-308,
                  1e150, -1e150, 1e300, -1e300, 1.7976931348623157e308, -1.7976931348623157e308]
        for bound in (*upper_range, *lower_range):
            values += [bound, math.nextafter(bound, math.inf), math.nextafter(bound, -math.inf)]
        cases = list(itertools.product(values, values)) + exact
        for (upper, lower), kind in itertools.product(cases, (float, np.float64)):
            if _check(_args(kind(upper), kind(lower), upper_range, lower_range)):
                slow += 1
            else:
                fast += 1
    assert fast > 0 and slow > 0  # the large values overflow and must take the numpy body
    for upper, lower in exact:  # signed zeros and a roll exactly on the band, with bounds that are zeros
        for bounds in itertools.product(((-0.0, 0.0), (0.0, -0.0), (0.0, 0.0), (-0.0, -0.0)), repeat=2):
            _check(_args(upper, lower, *bounds))
    for upper, lower, bounds in itertools.product((0.0, -0.0), (0.0, -0.0), ((-0.0, 0.0), (0.0, -0.0), (0.0, 0.0))):
        for coeffs, limit in ((ROLL.coeffs, ROLL.limit), ((0.0, 1.0, -1.0, 0.0, 0.0, 0.0), 0.0),
                              ((-0.0, 1.0, 1.0, 0.0, 0.0, 0.0), -0.0)):
            _check(_args(upper, lower, bounds, bounds, coeffs, limit))


def test_input_kinds():
    upper_range, lower_range = (-0.269253, 0.862665), (-0.862665, 0.513599)
    coeffs, limit = ROLL.coeffs, ROLL.limit
    fast = [
        _args(0.8, 0.5, upper_range, lower_range),
        _args(np.float64(0.8), np.float64(0.5), upper_range, lower_range),
        _args(0.8, np.float64(0.5), list(upper_range), list(lower_range), list(coeffs), np.float64(limit)),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, 1),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, 30),
    ]
    for make in fast:
        assert not _check(make)

    class Pair(tuple):
        pass

    slow = [
        _args(np.float32(0.8), 0.5, upper_range, lower_range),
        _args(0.8, np.float32(0.5), upper_range, lower_range),
        _args(1, 0.5, upper_range, lower_range),
        _args(0.8, 1, upper_range, lower_range),
        _args(True, 0.5, upper_range, lower_range),
        _args(0.8, False, upper_range, lower_range),
        _args(np.array(0.8), 0.5, upper_range, lower_range),
        _args(0.8, np.array(0.5), upper_range, lower_range),
        _args(np.array([0.8, 0.1]), 0.5, upper_range, lower_range),
        _args(np.array([0.8, 0.1]), np.array([0.5, -0.9]), upper_range, lower_range),
        _args([0.8, 0.1], [0.5, -0.9], upper_range, lower_range),
        _args("0.8", 0.5, upper_range, lower_range),
        _args(None, 0.5, upper_range, lower_range),
        _args(0.8, 0.5, np.array(upper_range), lower_range),
        _args(0.8, 0.5, upper_range, np.array(lower_range)),
        _args(0.8, 0.5, Pair(upper_range), lower_range),
        _args(0.8, 0.5, (0, 1), lower_range),
        _args(0.8, 0.5, upper_range, (np.float32(-0.5), 0.5)),
        _args(0.8, 0.5, ("-0.2", "0.8"), lower_range),
        _args(0.8, 0.5, (-0.2, 0.5, 0.8), lower_range),
        _args(0.8, 0.5, (0.5,), lower_range),
        _args(0.8, 0.5, upper_range, lower_range, np.array(coeffs)),
        _args(0.8, 0.5, upper_range, lower_range, (0, 1, 1, 0, 0, 0)),
        _args(0.8, 0.5, upper_range, lower_range, coeffs[:5]),
        _args(0.8, 0.5, upper_range, lower_range, coeffs + (0.0,)),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, 1),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, np.float32(limit)),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, np.array(limit)),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, 0),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, -1),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, True),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, 2.0),
        _args(0.8, 0.5, upper_range, lower_range, coeffs, limit, np.int64(3)),
        # Iterators are single-use, so each call gets a fresh one.
        lambda: (0.8, 0.5, iter(upper_range), lower_range, coeffs, limit),
        lambda: (0.8, 0.5, upper_range, (value for value in lower_range), coeffs, limit),
        lambda: (0.8, 0.5, upper_range, lower_range, iter(coeffs), limit),
        lambda: (0.8, 0.5, upper_range, lower_range, (value for value in coeffs), limit),
        lambda: (0.8, 0.5, upper_range, lower_range, (value for value in coeffs), limit, 0),
    ]
    for index, make in enumerate(slow):
        assert _check(make), f"case {index} did not take the numpy body"


def test_non_finite_values_fall_back():
    base = (0.8, 0.5, -0.269253, 0.862665, -0.862665, 0.513599, *ROLL.coeffs, ROLL.limit)
    for position, bad, kind in itertools.product(range(len(base)), (math.nan, math.inf, -math.inf), (float, np.float64)):
        v = list(base)
        v[position] = kind(bad)
        assert _check(_args(v[0], v[1], (v[2], v[3]), (v[4], v[5]), tuple(v[6:12]), v[12]))
    # Zero gradient with a roll outside the band: the numpy body divides by zero and returns NaN.
    assert _check(_args(0.0, 0.0, (-1.0, 1.0), (-1.0, 1.0), (1.0, 0.0, 0.0, 1.0, 0.0, 1.0), 0.1))
    # Zero gradient inside the band: 0/0.
    assert _check(_args(0.0, 0.0, (-1.0, 1.0), (-1.0, 1.0), (0.0, 0.0, 0.0, 1.0, 0.0, 1.0), 0.1))


@pytest.mark.parametrize("upper_range", [(0.0, 0.0), (1e200, 1e200)])
def test_fast_path_aborting_mid_loop_restarts_in_numpy(upper_range):
    # roll = upper^2. The first iteration is finite and clips upper onto the bound; the second one
    # has a zero gradient (bound 0) or an overflowing roll (bound 1e200).
    coeffs = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    make = _args(4.0, 0.25, upper_range, (-1.0, 1.0), coeffs, 1.0)
    assert _clip_foot_roll_scalar(*make(), 1) is not None
    assert _clip_foot_roll_scalar(*make(), 2) is None
    assert _check(make)
    with np.errstate(all="ignore"):
        upper, lower = clip_foot_roll(*make())
    assert math.isnan(upper) and math.isnan(lower)


_KINDS = {"float": float, "float64": np.float64, "tuple": tuple, "list": list}


def _decode(item):
    kind, value = item
    if kind in ("tuple", "list"):
        return _KINDS[kind](_decode(entry) for entry in value)
    return _KINDS[kind](float.fromhex(value))


def test_replay_of_recorded_deploy_calls(monkeypatch):
    path = os.environ.get("ROBONEX_ROLL_REPLAY")
    if not path:
        pytest.skip("set ROBONEX_ROLL_REPLAY to a recording of the deploy loop's clip_foot_roll calls")
    with open(path, encoding="utf-8") as stream:
        recording = json.load(stream)

    assert len(recording["calls"]) >= 10_000
    for encoded, old_output in recording["calls"]:
        args = _decode(encoded)
        assert not _check(lambda: args)
        new = clip_foot_roll(*args)
        assert [float(value).hex() for value in new] == old_output  # what the unmodified package returned

    pipeline = _pipeline()
    pipeline.roll_pairs = [tuple(_decode(entry) for entry in pair) for pair in recording["roll_pairs"]]
    pipeline.roll_pairs = [(int(a), int(b), sign, upper, lower) for a, b, sign, upper, lower in pipeline.roll_pairs]
    pipeline.roll_coeffs, pipeline.roll_limit = _decode(recording["roll_coeffs"]), _decode(recording["roll_limit"])
    targets = []
    for kind, values in recording["clip_roll_inputs"]:
        floats = [float.fromhex(value) for value in values]
        targets.append(floats if kind == "list" else np.asarray(floats, dtype=np.dtype(kind)))
    assert len(targets) * len(pipeline.roll_pairs) == len(recording["calls"])
    fast = [pipeline.clip_roll(target) for target in targets]
    monkeypatch.setattr(runtime_module, "clip_foot_roll", _clip_foot_roll_numpy)
    slow = [pipeline.clip_roll(target) for target in targets]
    assert b"".join(array.tobytes() for array, _ in fast) == b"".join(array.tobytes() for array, _ in slow)
    assert [changed for _, changed in fast] == [changed for _, changed in slow]
    assert [array.tobytes().hex() for array, _ in fast] == recording["clip_roll_outputs"]
