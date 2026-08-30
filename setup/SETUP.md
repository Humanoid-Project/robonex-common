# Setup

All RoboNex repos are cloned as siblings under one folder (`~/humanoid_project/` on the dev PC), and every repo depends on `robonex-common`. `setup/setup.sh` in this repo is the shared bootstrap step every venv-based repo's README calls.

## Layout

```text
~/humanoid_project/
├── robonex-common/
│   └── setup/
│       ├── setup.sh
│       └── SETUP.md
├── robonex_description/
├── Robstride-Motor-Test/
├── robonex-deploy/
├── IMU_N100_Test/
└── robonex_balancing/
```

## What `setup.sh` does

Run from inside the repo being set up, **sourced** (not executed) so the venv activation reaches your shell:

```bash
source ../robonex-common/setup/setup.sh [sibling-repo ...]
```

| Step | Behavior |
| --- | --- |
| Clone each named sibling repo | Skipped if the folder already exists |
| Create `.venv` | Skipped if it already exists; always activated |
| `pip install -r requirements.txt` | Only if the file exists in this repo |
| `pip install -e ../robonex-common` | Always last |

Safe to re-run — every step is a no-op if already done.

## Why `robonex-common` is never in `requirements.txt`

`robonex-common` is always installed the same way, in the same repos: cloned as a sibling and installed editable, last. A `requirements.txt` line like `robonex-common @ git+https://...` is a second, competing install path for the same package — whichever `pip install` runs last wins, and it has silently overwritten the editable install twice already (`robonex-deploy`, `Robstride-Motor-Test`, both fixed 2026-08-30). `requirements.txt` in every repo now lists only its own real dependencies (`mujoco`, `numpy`, etc.), never `robonex-common`.

## Repos that use this script

`robonex_description`, `Robstride-Motor-Test`, `robonex-deploy` — plain `venv`.

`robonex_balancing` does **not** use this script. It runs on a `conda` environment (`isaacsim`) that Isaac Sim's own installer creates, not a `venv`; forcing it through this script would misrepresent that setup. Its README documents its own steps, but follows the same two rules: `robonex-common` cloned first, installed editable last.

## Example: adding a new consuming repo

```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/<new-repo>.git
cd <new-repo>
source ../robonex-common/setup/setup.sh [any-other-sibling-repo ...]
```
