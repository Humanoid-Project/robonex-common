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

Safe to re-run — every step is a no-op if already done.

## Used by

`robonex-description`, `robstride-motor-test`, `robonex-deploy`.

`robonex-balancing` uses `conda`, not this script.

## `setup_isaacsim.sh`

Creates the `isaacsim` conda env (Isaac Sim 5.1.0, Isaac Lab v2.3.2) used by `robonex-balancing` and `robonex-description`'s `isaac/` scripts. Same rules: **source, don't `bash`**, safe to re-run.

```bash
source ../robonex-common/setup/setup_isaacsim.sh
```
