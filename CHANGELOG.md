# Changelog

## Versioning

| Change | Bump |
| --- | --- |
| Bug fix, no behavior change | Patch |
| New constant or function | Minor |
| Changed physical value — joint limits, motor specs, gains, `POLICY_JOINT_ORDER` | Minor |
| Renamed or removed public name | Major |

A physical value breaks no API but changes how the real robot moves, so it is never a patch.

Bump `pyproject.toml` `version` and `__init__.__version__` in the same commit as the tag.

<br>

## 0.2.0 — 2026-09-07

Joint limits in `ACTUATED_JOINTS` replaced with the full measured reachable range.
The previous values were the conservative minimum needed for walking; these are the
mechanical travel measured on the assembled robot (`joint_limit_20260907T202634.csv`),
matched left/right to the smaller magnitude of each mirrored pair and rounded inward.

| Joint | 0.1.0 (deg) | 0.2.0 (deg) |
| --- | --- | --- |
| `l/r_hip_yaw` | −40 / +40 | −95 / +95 |
| `l/r_hip_pitch` | −50 / +50 | −100 / +100 |
| `l_hip_roll` | −60 / +5 | −123 / +26 |
| `r_hip_roll` | −5 / +60 | −26 / +123 |
| `l_knee_pitch` | −50 / +5 | −86 / +54 |
| `r_knee_pitch` | −5 / +50 | −54 / +86 |
| `l_ankle_upper`, `l_ankle_lower` | −35 / +25, −25 / +35 | −35 / +33 |
| `r_ankle_upper`, `r_ankle_lower` | −25 / +35, −35 / +25 | −33 / +35 |

Every right joint is now the exact sign mirror of its left counterpart
(`r.lower, r.upper == -l.upper, -l.lower`), verified in MuJoCo: only that pairing
produces `roll_L == -roll_R` and `pitch_L == pitch_R` at a mirrored ankle pose.
The old `ID11 == ID6 / ID12 == ID5` crossed-identical description happened to agree
with the mirror only because the previous left ankle ranges were antisymmetric
within the leg; it no longer holds and its test was replaced.

`action_normalization()` derives `offset`/`scale`/`clip` from this table, so every
policy trained against 0.1.0 maps actions to different targets under 0.2.0 and must
be retrained. Checkpoints and exported ONNX from before this release are not portable.

Tests now cross-check the table against `robonex-description`'s `urdf/robonex.urdf`
so the two cannot drift apart again.

## 0.1.0 — 2026-09-03

First tagged release.

| Added | Contents |
| --- | --- |
| `paths` | `resolve_repo`, `repo_file`, `description_model`, `git_commit` |
| `imu` | `MOUNT_ROLL_DEG`, `DEFAULT_IMU_PORT`, `DEFAULT_IMU_BAUDRATE`, `EXPECTED_UPRIGHT_GRAVITY` |
| `actuators` | `ACTUATOR_PARAMETERS` |
| `runtime` | `ActionPipeline`, `assemble_observation` — needs the `policy` extra |
| `joints` | `channel_for_motor_id` |
| `protocol` | `FAULT_BIT_NAMES`, `decode_fault_bits` |
| Packaging | `__version__`, `py.typed` |

| Removed | Reason |
| --- | --- |
| `protocol.build_arb`, `protocol.parse_arb` | No callers |
| `Motor.write_param_u16`, `Motor.write_param_f32` | No callers |

`__init__` re-exports every stdlib-only module. `runtime` is excluded so `import robonex_common` never requires numpy.
