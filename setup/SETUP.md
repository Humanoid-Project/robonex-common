# Setup

## Structure

```text
setup/
├── SETUP.md
├── setup.sh
└── setup_isaacsim.sh
```

<br>

## `setup.sh`

Clones `robonex-description`, `robonex-deploy`, `robstride-motor-test` and `IMU_N100_Test` into one project root,
creates each `.venv`, installs its `requirements.txt` (which pins `robonex-common`), and builds the deploy `n100` module.
Anything already present is skipped.

```bash
# Example
bash <(curl -fsSL https://raw.githubusercontent.com/Humanoid-Project/robonex-common/main/setup/setup.sh)

bash <(curl -fsSL https://raw.githubusercontent.com/Humanoid-Project/robonex-common/main/setup/setup.sh) ~/work/humanoid_project

PYTHON=python3.10 ./setup/setup.sh
```

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `root` | `~/humanoid_project` | Project root, created if missing |

| Variable | Required | Default | Description |
| --- | --- | --- | --- |
| `PYTHON` | No | `python3` | Interpreter used to create each `.venv` |

| Output | Description |
| --- | --- |
| `<root>/robonex-description`, `robonex-deploy`, `robstride-motor-test`, `IMU_N100_Test` | Git checkouts |
| `<repo>/.venv` | One venv per Python repo, with `robonex-common` at the pinned tag |
| `robonex-deploy/scripts/policy_test/n100*.so` | IMU Python module, built only if `cmake` and a C++ compiler exist |

<br>

## `setup_isaacsim.sh`

Creates the `isaacsim` conda env (Isaac Sim 5.1.0, Isaac Lab v2.3.2) used by
`robonex-balancing` and `robonex-walking`. Must be `source`d, not `bash`ed.

```bash
# Example
cd ~/humanoid_project
source ./robonex-common/setup/setup_isaacsim.sh

source <(curl -fsSL https://raw.githubusercontent.com/Humanoid-Project/robonex-common/main/setup/setup_isaacsim.sh)
```

<br>

## Installing `robonex-common`

Always the published tag — never a local checkout. Each repo pins it in its own
`requirements.txt`, so a plain `pip install -r requirements.txt` brings it in.

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

| Repo | Pin | Environment |
| --- | --- | --- |
| `robonex-description` | `robonex-common` | `.venv` |
| `robstride-motor-test` | `robonex-common[can]` | `.venv` |
| `robonex-deploy` | `robonex-common[can,policy]` | `.venv` |
| `robonex-balancing` | `robonex-common` | conda `isaacsim` |
| `robonex-walking` | `robonex-common` | conda `isaacsim` |
