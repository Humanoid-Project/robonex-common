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
