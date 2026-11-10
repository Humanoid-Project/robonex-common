# robonex-common

## Structure

```text
robonex-common/
├── README.md
├── CHANGELOG.md
├── pyproject.toml
├── src/robonex_common/
│   ├── joints.py
│   ├── motors.py
│   ├── actuators.py
│   ├── limits.py
│   ├── models.py
│   ├── buses.py
│   ├── protocol.py
│   ├── can.py
│   ├── imu.py
│   ├── paths.py
│   ├── policy.py
│   └── runtime.py
├── setup/
│   ├── SETUP.md
│   ├── setup.sh
│   └── setup_isaacsim.sh
└── tests/
    ├── test_contracts.py
    └── test_modules.py
```

| Module | Contents | Extra |
| --- | --- | :---: |
| `joints` | Motor ID, group, joint names, `ACTUATED_JOINTS` (policy), `AUXILIARY_JOINTS` (arms, head), `ALL_MOTORS`, `POLICY_JOINT_ORDER`, `channel_for_motor_id` | - |
| `buses` | Motor group → CAN channel (`DEFAULT_BUS_MAP`, `~/.config/robonex/bus_map.json`, `ROBONEX_BUS_MAP`) | - |
| `motors` | RS00/RS02/RS03/RS05/RS10P specs, `MOTOR_PHYSICS`, kp/kd, rated/peak torque | - |
| `actuators` | `ACTUATOR_PARAMETERS` (stiffness/damping/armature/friction) | - |
| `limits` | Joint limits, `action_normalization` | - |
| `models` | `ROBOT_MODELS` (leg profile `ver2_edu`), `VARIANTS` (`ver2` with `motor_ids`), `leg_profile` | - |
| `protocol` | CAN type/index constants, `build_arbitration_id`, `decode_fault_bits` | - |
| `can` | `Motor`, `FeedbackHub`, `drain` | `can` |
| `imu` | N100 port/baudrate, `MOUNT_ROLL_DEG` | - |
| `paths` | `resolve_repo`, `repo_file`, `description_model`, `git_commit` | - |
| `policy` | `PolicyContract` manifest read/write, policy/model/source SHA-256 | - |
| `runtime` | `ActionPipeline`, `assemble_observation` | `policy` |

| ID | Joint | Group | Model | Default CAN |
| --- | --- | --- | --- | --- |
| `1`–`6` | `left_hip_yaw` … `left_ankle_lower` | `left_leg` | rs02/rs03 | `can0` |
| `7`–`12` | `right_hip_yaw` … `right_ankle_lower` | `right_leg` | rs02/rs03 | `can1` |
| `13`–`15` | reserved | `waist` | - | - |
| `16`–`20` | `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw`, `left_elbow`, `left_wrist_yaw` | `left_arm` | rs02, rs10p, rs00, rs00, rs05 | `can2` |
| `21`–`25` | `right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`, `right_elbow`, `right_wrist_yaw` | `right_arm` | rs02, rs10p, rs00, rs00, rs05 | `can3` |
| `26`–`27` | `neck_pitch`, `neck_yaw` (`28` reserved) | `head` | rs05 | `can4` |

One robot model, `ver2` (all IDs above); an identity file that still says `ver2_edu` is read as `ver2`. Head pitch ±73°, shoulder pitch (16, 21) ±100°; the other arm and neck-yaw limits (±45° / ±30°) are PLACEHOLDERs until measured. Policies keep the leg profile `ver2_edu` (`ROBOT_MODELS`); head/arm gains are placeholders.

<br>

## Install

No checkout required — install the published tag.

```bash
# Example
pip install "robonex-common @ git+https://github.com/Humanoid-Project/robonex-common.git@v1.0.0"

pip install "robonex-common[can] @ git+https://github.com/Humanoid-Project/robonex-common.git@v1.0.0"
pip install "robonex-common[can,policy] @ git+https://github.com/Humanoid-Project/robonex-common.git@v1.0.0"
```

| Extra | Pulls in | Needed for |
| --- | --- | --- |
| (none) | - | `joints`, `motors`, `actuators`, `limits`, `protocol`, `imu`, `paths`, `policy` |
| `can` | `python-can` | `Motor` / `FeedbackHub` on a real bus |
| `policy` | `numpy` | `ActionPipeline` / `assemble_observation` |

<br>

## Project setup

Clones `robonex-description`, `robonex-deploy`, `robstride-motor-test` and `IMU_N100_Test`, then sets up each `.venv`.
See [`setup/SETUP.md`](setup/SETUP.md).

```bash
# Example
bash <(curl -fsSL https://raw.githubusercontent.com/Humanoid-Project/robonex-common/main/setup/setup.sh)
```

<br>

## Test

```bash
# Example
cd ~/humanoid_project/robonex-common
PYTHONPATH=src python -m pytest -q
```

<br>

## License

Licensed under the [PolyForm Noncommercial License 1.0.0](LICENSE). Copyright 2026 RoboNex. Commercial use requires a separate licence. See [NOTICE](NOTICE).
