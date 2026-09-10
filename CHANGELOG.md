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

## 0.4.0 — 2026-09-10

`ACTION_SCALE_RAD` is rescaled per joint so the nearest target clip is reached at
approximately `|a| = RUNNER_ACTION_CLIP = 14`. Under 0.3.0 the nearest clip was reached
as early as `|a| = 1.78` on `hip_roll`; the rest of the runner range produced the same
clipped target and gave the policy no motion response or useful gradient. The new scales
remove that target-clip dead zone and reduce target sensitivity, especially on hip roll,
knee, and ankle.

| Joint pair | 0.3.0 | 0.4.0 |
| --- | ---: | ---: |
| hip yaw | 0.12 | 0.117718 |
| hip pitch | 0.25 | 0.116809 |
| hip roll | 0.25 | 0.031698 |
| knee pitch | 0.25 | 0.078943 |
| ankle upper | 0.15 | 0.025735 |
| ankle lower | 0.15 | 0.028228 |

The farther side of an asymmetric joint range is intentionally no longer reachable.
The standing-pose offset, target clips, `RUNNER_ACTION_CLIP`, observation layout, gains,
and joint limits are unchanged. Existing checkpoints and policy manifests are incompatible
with this action mapping and must not be resumed or deployed.

<br>

## 0.3.0 — 2026-09-08

`action_normalization()` no longer normalizes against the joint range. It now maps
the policy action relative to the standing pose with a fixed per-joint step size,
the convention used by every published humanoid sim-to-real stack (Unitree G1's
`unitree_rl_gym` is `target = default_joint_angles + 0.25 * action`).

| | 0.2.0 | 0.3.0 |
| --- | --- | --- |
| `offset` | `(clip_lo + clip_hi) / 2` | `DEFAULT_JOINT_POS[name]` |
| `scale` | `(clip_hi - clip_lo) / 2` | `ACTION_SCALE_RAD[name]` |
| `clip` | `(lo + margin, hi - margin)` | unchanged |
| `a = 0` commands | the midpoint of the joint range | the standing pose |
| `\|a\| = 1` spans | the whole joint range | 7-25 % of it |
| joint limit reached at | `\|a\| = 1` | `\|a\|` up to 13.73 |

Under 0.2.0 an asymmetric joint put its near mechanical limit at `a = 1` while the
standing pose sat at a non-zero action (hip_roll `a = 0.651`), so a policy with
`sigma = 0.3` crossed the clip on 13 % of steps and every action unit was a
different number of radians per joint (hip_roll 73.9 deg, hip_yaw 95.0 deg). Under
0.3.0 the standing pose is `a = 0` for every joint, one action unit is a fixed
small angle, and the nearest limit sits 1.78 action units away instead of 1.00.

`PolicyContract.validate()` now rejects a manifest whose `action_offsets`, `action_scales`
or `target_clips` disagree with the current `action_normalization()`. Before this, a manifest
written under an older mapping loaded without complaint and commanded different joint angles
on the real robot; `joint_order` was checked but the numbers that decide the actual target
were not. `runner_action_clip` is deliberately not checked — it is a training hyperparameter,
and a smaller clip is self-consistent rather than a wrong-target hazard.

Manifest schema 2 records SHA-256 fingerprints for the complete MuJoCo XML/mesh bundle,
the `robonex-common` Python source, and the training package/scripts in addition to the
policy file. Deployment verifies the model bundle and common source before simulation,
so an uncommitted source or generated-model change can no longer hide behind an unchanged
Git commit.

| Added | Contents |
| --- | --- |
| `joints.DEFAULT_JOINT_POS` | Mildly bent policy standing pose; hardware and URDF mechanical zero remain all zero |
| `limits.ACTION_SCALE_RAD` | Radians per action unit. hip_yaw 0.12, hip_pitch/roll/knee 0.25, ankle 0.15 |
| `limits.DEFAULT_ACTION_MARGIN_RAD` | 0.01, the former `action_normalization` default made explicit |
| `limits.RUNNER_ACTION_CLIP` | 14.0, the raw-action clip the RL runner applies |
| `limits.action_limit_reach()` | Action value at which each joint reaches its clip |
| `policy.ACTION_CONTRACT_TOLERANCE_RAD` | 1e-9, the manifest-vs-contract comparison tolerance |

`RUNNER_ACTION_CLIP` replaces the 3.0 that `robonex-balancing` and `robonex-walking`
each hardcoded. With the 0.3.0 scales a raw action of 3.0 would leave 9 of 12 joints
unable to reach their mechanical limit at all, so the clip has to be the largest
`action_limit_reach()` magnitude (13.73, at hip_yaw) rounded up. A test enforces this.

`action_normalization()` now raises if the standing pose falls outside the clipped
range or if a scale is non-positive.

Every policy trained against 0.2.0 or earlier maps actions to different targets and
must be retrained. Checkpoints, exported ONNX and manifests from before this release
are not portable.

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
