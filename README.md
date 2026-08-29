# robonex-common

## Structure

```text
robonex-common/
├── README.md
├── pyproject.toml
├── src/robonex_common/
│   ├── joints.py
│   ├── motors.py
│   ├── limits.py
│   ├── protocol.py
│   ├── can.py
│   └── policy.py
└── tests/
    └── test_contracts.py
```

| Module | Contents |
| --- | --- |
| `joints` | Motor ID, CAN channel, joint names, `POLICY_JOINT_ORDER` |
| `motors` | RS02/RS03 specs, `MOTOR_PHYSICS`, kp/kd |
| `limits` | Joint limits, `action_normalization` |
| `protocol` | CAN type/index constants, `build_arb` / `parse_arb` |
| `can` | `Motor`, `FeedbackHub` |
| `policy` | `PolicyContract` manifest read/write |

<br>

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
python -m pip install -e ./robonex-common

python -m pip install -e './robonex-common[can]'
```

<br>

## Test
```bash
# Example
cd ~/humanoid_project/robonex-common
python -m pytest -q
```
