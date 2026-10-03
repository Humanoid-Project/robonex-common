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

## 1.2.0 — 2026-09-30

**Head 13 (14 reserved for neck yaw, commented out until the motor exists), arms 15–18 / 20–23, per-variant motor sets.** Minor bump: new constants, functions and physical table
entries; nothing renamed or removed. The 12-joint policy contract is byte-identical to 1.1.0 (`ACTUATED_JOINTS`,
`POLICY_JOINT_ORDER`, `JOINT_BY_ID`, `CHANNEL_MOTOR_IDS`, `DEFAULT_JOINT_POS`, `action_normalization`,
`ACTION_SCALE_RAD`, `ACTUATOR_PARAMETERS`, `VER2_EDU` leg tables and foot-roll clip).

| Group | IDs | Joints | Motor | Default CAN | edu | pro | max |
| --- | --- | --- | --- | --- | :---: | :---: | :---: |
| left_leg | 1–6 | as 1.1.0 | rs02/rs03 | can0 | ✓ | ✓ | ✓ |
| right_leg | 7–12 | as 1.1.0 | rs02/rs03 | can1 | ✓ | ✓ | ✓ |
| head | 13 (14 reserved for neck yaw, no motor yet — commented out) | `neck_pitch_joint` | rs05 | can4 | 13 | 13 | 13 |
| left_arm | 15–18 | `l_shoulder_pitch/roll/yaw_joint`, `l_elbow_joint` | rs02 | can2 | - | ✓ | ✓ |
| right_arm | 20–23 | `r_shoulder_pitch/roll/yaw_joint`, `r_elbow_joint` | rs02 | can3 | - | ✓ | ✓ |
| - | 19 | unused | | | | | |

Head limit ±73° (±1.27409 rad) and shoulder pitch (IDs 15 and 20) ±100° (±1.745329 rad), set by the user on the bench
(2026-10-01; ID 20 asked, ID 15 mirrored); they were ±30° / ±45° placeholders.

**PLACEHOLDER values — not measured, do not trust on hardware:** other arm limits ±45° (±0.785398 rad), head gains
kp 20 / kd 1, arm gains kp 40 / kd 2 (open item H29).

- `joints.py`: `AUXILIARY_JOINTS` adds IDs 14–18 and 20–23 (table above); `GROUP_ID_RANGES` head 13–14, left_arm
  15–18, right_arm 20–23 (was 13 / 14–17 / 18–21). New `VARIANT_MOTOR_IDS` and `motors_for_variant(name)`.
  `ALL_CHANNEL_MOTOR_IDS` now spans five channels with the default bus map.
- `models.py`: `RobotModel` gains `motor_ids` and `leg_profile`. New `VER2_PRO`, `VER2_MAX` (same leg limits,
  default pose and foot-roll clip as `VER2_EDU`), `VARIANTS` (all three), `LEG_PROFILES` (alias of `ROBOT_MODELS`) and
  `leg_profile(name)`. `ROBOT_MODELS` stays at the leg-profile level (`ver2_edu` only), so callers that infer the
  profile from a checkpoint or a MuJoCo model by requiring exactly one match keep working unchanged.
  `robot_model(name)` resolves variant names too, so manifests and `joint_limit_for(..., model=)` accept
  `ver2_pro` / `ver2_max`.
- `motors.py`: `JOINT_CONTROL_GAINS` entries `neck_pitch`, `neck_yaw`, `shoulder_pitch`, `shoulder_roll`,
  `shoulder_yaw`, `elbow` (placeholders). `MOTOR_PHYSICS` unchanged (no `rs05`).
- `actuators.py`: `CONTROL_GAINS_BY_JOINT` covers every motor in `ALL_MOTORS` (was the 12 legs; ID 13 had no entry).
  `ACTUATOR_PARAMETERS` still lists only the 12 simulated leg joints.

<br>

## 1.1.0 — 2026-09-30

**Head motor and per-machine CAN channel map.** Minor bump: new constants and one new physical table
entry; the 12-joint policy contract (`ACTUATED_JOINTS`, `POLICY_JOINT_ORDER`, `RobotModel`) is unchanged.

- `motors.py`: `rs05` spec (MIT ranges ±12.57 rad, ±50 rad/s, ±5.5 N·m, kp 0–500, kd 0–5; rated 1.6, peak 5.5 N·m,
  no-load 50.3 rad/s). No `MOTOR_PHYSICS` entry (armature not measured), so `ACTUATOR_PARAMETERS` has no `rs05`.
- `joints.py`: `JointSpec.channel` is now a property derived from a new `group` field (`left_leg`, `right_leg`,
  `left_arm`, `right_arm`, `head`). `AUXILIARY_JOINTS` = ID 13 `neck_pitch_joint` (rs05, head, ±30° placeholder until
  measured), `ALL_MOTORS`, `MOTOR_BY_ID`, `MOTOR_LIMITS_BY_ID`, `GROUP_ID_RANGES` (legs 1–6 / 7–12, head 13, arms
  14–17 / 18–21 reserved), `motor_ids_by_channel()`, `ALL_CHANNEL_MOTOR_IDS`. `channel_for_motor_id` covers ID 13.
- `buses.py`: `DEFAULT_BUS_MAP` (`left_leg can0, right_leg can1, left_arm can2, right_arm can3, head can4`),
  overridden per machine by `~/.config/robonex/bus_map.json` (or the file named by `ROBONEX_BUS_MAP`).
- `limits.joint_limit_for` / `exceeds_joint_limit` fall back to `MOTOR_LIMITS_BY_ID` for motors outside the policy model.

<br>

## 1.0.0 — 2026-09-27

**Ver.2 edu only.** The Ver.1 robot is disassembled (user, 2026-09-27); Ver.1 survives only as a record in
`robonex-description/ver1/`. Major bump: public names removed and every physical value changed.

| Changed | Ver.1 (0.6.0 default) | Ver.2 edu (1.0.0) |
| --- | --- | --- |
| `ACTUATED_JOINTS` limits (left; right mirrored), deg | hip yaw ±95, pitch ±100, roll −123/+26, knee −86/+54, ankle upper/lower −35/+33 | hip yaw ±48, pitch ±95, roll −120/+10, knee −70/+10, ankle upper −16/+50, lower −50/+30 |
| `DEFAULT_JOINT_POS` knee, ankle upper, ankle lower (left), rad | −0.38578, 0.2056595, −0.2056595 | −0.3298657, 0.2125577, −0.2038049 |
| `ACTION_SCALE_RAD` knee, hip roll, ankle upper, ankle lower | 0.25, 0.152626, 0.1201, 0.131736 | 0.1648, 0.148886, 0.160604, 0.219621 |

- `joints.py` holds the Ver.2 edu table; `models.VER2_EDU` is built from it, so `JOINT_LIMITS_BY_ID` (deploy,
  motor tests), `limits.joint_limit_for` and `action_normalization` all use Ver.2 edu. Values are identical to
  0.6.0's `ver2_edu` profile (limits, default pose, normalisation, foot-roll clip), so Ver.2 manifests exported
  under 0.6.0 still validate.
- Removed: `models.VER1`, the `ver1` entry of `ROBOT_MODELS`, manifest schema 2 (`SUPPORTED_SCHEMAS = (3,)`).
  Every function's `model` default is `"ver2_edu"`.
- Ver.2 edu hip roll: the far fence (−120°) sets the action scale, so the near fence (+10°) sits 1.1 sigma from the
  default; the fence test states that exception.

<br>

## 0.6.0 — 2026-09-27

Recorded with 1.0.0 (the tag was cut without an entry).

- `models.py`: `RobotModel` / `FootRollClip` profiles `ver1` and `ver2_edu`; `robot_model(name)`.
- `foot_roll.py`: coupled foot-roll clip (12°, quadratic crank model) in numpy, parity with the training term.
- Manifest schema 3: `robot_model`, `foot_roll_limit`, `foot_roll_coeffs`, `foot_roll_pairs`; `ActionPipeline`
  applies the roll clip in float64.
- `limits` functions take `model=` (default `ver1`).

<br>

## 0.5.0 — 2026-09-14

Recorded after the fact on 2026-09-24: the tag was cut by hand, not with `setup/release.sh`,
so this entry and the pin updates were missing.

**Control gains are per joint.** `JOINT_CONTROL_GAINS` gives kp/kd per joint role and
`CONTROL_GAINS_BY_JOINT` maps it onto every actuated joint. `ACTUATOR_PARAMETERS`
`stiffness` / `damping` are now per-joint dicts instead of one scalar per motor model.
`MOTOR_CONTROL_KP` / `MOTOR_CONTROL_KD` (40 / 2) remain for tools that command a uniform gain.

| Joint role | kp | kd |
| --- | ---: | ---: |
| hip yaw, hip pitch, hip roll | 100 | 2 |
| knee pitch | 150 | 4 |
| ankle upper, ankle lower | 40 | 2 |

**Motor physics.**

- `MOTOR_PHYSICS` splits friction into `frictionloss`, `static_friction` and `viscous_friction`,
  all `0.0` (0.4.0: `frictionloss` 0.1 on RS02, 0.2 on RS03).
- `ACTUATOR_PARAMETERS` adds `dynamic_friction`, `viscous_friction`, `effort_limit_sim`
  (`PEAK_TORQUE`) and `velocity_limit_sim` (`VELOCITY_LIMIT`).
- New `NO_LOAD_SPEED` (RS02 42.9, RS03 20.9 rad/s), `VELOCITY_LIMIT_DERATE = 0.9` and
  `VELOCITY_LIMIT` (38.61 / 18.81 rad/s).
- `RATED_TORQUE["rs03"]` is the standstill continuous value, `13.0` (0.4.0: `20.0`, the
  heatsinked rotating rating).

**Action scales are derived, not tabulated.** `ACTION_SCALE_RAD` is computed per joint as
`max(near / 3, far / RUNNER_ACTION_CLIP)`, capped at `MAX_ACTION_SCALE_RAD = 0.25`, so the near
fence sits at least 3 sigma out and the far fence stays reachable inside the runner clip.
`action_normalization` now rejects a scale that cannot reach its far clip, instead of one that
creates a near dead zone.

| Joint pair | 0.4.0 | 0.5.0 |
| --- | ---: | ---: |
| hip yaw | 0.117718 | 0.25 |
| hip pitch | 0.116809 | 0.25 |
| hip roll | 0.031698 | 0.152626 |
| knee pitch | 0.078943 | 0.25 |
| ankle upper | 0.025735 | 0.1201 |
| ankle lower | 0.028228 | 0.131736 |

**Observation contract with history.** One frame is `OBSERVATION_TERM_SIZES` =
`joint_pos_rel 12, joint_vel_rel 12, imu_ang_vel 3, projected_gravity 3, velocity_commands 3,
gait_phase 2, last_action 12` (47); `OBSERVATION_HISTORY_LENGTH = 5` gives `OBSERVATION_SIZE = 235`,
term-major and oldest frame first. New `assemble_observation_frame`, `ObservationHistory`,
`GAIT_PERIOD_S = 0.8` and `gait_phase_at`. `assemble_observation` takes the new terms, so
0.4.0 callers must be updated.

Checkpoints and manifests built on 0.4.0 are incompatible with these scales, gains and
observations and must not be deployed.

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
