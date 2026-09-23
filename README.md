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
| `joints` | Motor ID, CAN channel, joint names, `POLICY_JOINT_ORDER`, `channel_for_motor_id` | - |
| `motors` | RS02/RS03 specs, `MOTOR_PHYSICS`, kp/kd, rated/peak torque | - |
| `actuators` | `ACTUATOR_PARAMETERS` (stiffness/damping/armature/friction) | - |
| `limits` | Joint limits, `action_normalization` | - |
| `protocol` | CAN type/index constants, `build_arbitration_id`, `decode_fault_bits` | - |
| `can` | `Motor`, `FeedbackHub`, `drain` | `can` |
| `imu` | N100 port/baudrate, `MOUNT_ROLL_DEG` | - |
| `paths` | `resolve_repo`, `repo_file`, `description_model`, `git_commit` | - |
| `policy` | `PolicyContract` manifest read/write, policy/model/source SHA-256 | - |
| `runtime` | `ActionPipeline`, `assemble_observation` | `policy` |

<br>

## Install

No checkout required — install the published tag.

```bash
# Example
pip install "robonex-common @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.5.0"

pip install "robonex-common[can] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.5.0"
pip install "robonex-common[can,policy] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.5.0"
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
