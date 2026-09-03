# Setup

Shared venv bootstrap for repos that clone `robonex-common` as a sibling, plus the Isaac Sim/Isaac Lab conda bootstrap used by `robonex-balancing` and `robonex-description`.

## Structure

```text
setup/
├── SETUP.md
├── setup.sh
└── setup_isaacsim.sh
```

## Usage

```bash
source ../robonex-common/setup/setup.sh [sibling-repo ...]
```

Must be `source`d, not `bash`ed — activates `.venv` in the caller's shell.

| Step | Behavior |
| --- | --- |
| Clone each named sibling repo | Skipped if already present |
| Create `.venv` | Skipped if already present; always activated |
| `pip install -r requirements.txt` | Only if the file exists |
| `pip install -e ../robonex-common` | Always last |

If a repository directory was moved or renamed, recreate its `.venv` before
running the setup again. Python virtual environments contain absolute paths and
are not portable between repository paths. The setup stops with an error when
it detects such a moved environment instead of falling back to a system `pip`.

Safe to re-run — every step is a no-op if already done.

## Library-only install (no clone)

Anyone who only needs the shared contracts — joint IDs and limits, motor specs,
the CAN protocol, `Motor`/`FeedbackHub`, the policy manifest — installs the
published tag instead of cloning. `setup.sh` is not involved.

```bash
pip install "robonex-common @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"

pip install "robonex-common[can] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"
pip install "robonex-common[policy] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"
```

| Extra | Pulls in | Needed for |
| --- | --- | --- |
| (none) | nothing | joints, limits, motor specs, protocol constants, paths |
| `can` | `python-can` | `Motor`, `FeedbackHub` on a real bus |
| `policy` | `numpy` | `ActionPipeline`, `assemble_observation` |

Never combine this with `pip install -e ../robonex-common` in the same
environment — whichever `pip` ran last silently wins. Clone plus editable is the
developer path; the pinned tag is the consumer path.

## Used by

`robonex-description`, `robstride-motor-test`, `robonex-deploy`.

`robonex-balancing` uses `conda`, not this script.

## `setup_isaacsim.sh`

Creates the `isaacsim` conda env (Isaac Sim 5.1.0, Isaac Lab v2.3.2) used by `robonex-balancing` and `robonex-description`'s `isaac/` scripts. Same rules: **source, don't `bash`**, safe to re-run.

```bash
source ../robonex-common/setup/setup_isaacsim.sh
```
