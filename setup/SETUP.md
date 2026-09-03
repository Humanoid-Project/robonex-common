# Setup

## Structure

```text
setup/
├── SETUP.md
├── release.sh
└── setup_isaacsim.sh
```

<br>

## Installing `robonex-common`

Always the published tag — never a local checkout. Each repo pins it in its own
`requirements.txt`, so a plain `pip install -r requirements.txt` brings it in.

```bash
# Example
pip install "robonex-common @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"

pip install "robonex-common[can] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"
pip install "robonex-common[can,policy] @ git+https://github.com/Humanoid-Project/robonex-common.git@v0.1.0"
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

<br>

## `release.sh`

Publishes a new `robonex-common` version and refreshes every dependent checkout.
Add the changelog entry first.

```bash
# Example
cd ~/humanoid_project/robonex-common
./setup/release.sh 0.2.0

./setup/release.sh 0.2.0 --no-push
```

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `version` | Yes | - | `MAJOR.MINOR.PATCH`, must be newer than the current one |
| `--no-push` | No | Off | Commit and tag locally, skip push and reinstall |

| Step | Behavior |
| --- | --- |
| Bump `pyproject.toml` and `__init__.__version__` | Fails if the version is not newer |
| Run the test suite | Aborts before any commit on failure |
| Rewrite every pin to the new tag | `requirements.txt` and `README.md` across all repos |
| Commit, tag `v<version>`, push | Skipped with `--no-push` |
| Reinstall into each repo's `.venv` | Skipped for a repo with no `.venv` |

The conda env `isaacsim` is not touched; reinstall there yourself.

<br>

## `setup_isaacsim.sh`

Creates the `isaacsim` conda env (Isaac Sim 5.1.0, Isaac Lab v2.3.2) used by
`robonex-balancing` and `robonex-walking`. Must be `source`d, not `bash`ed.

```bash
# Example
cd ~/humanoid_project
source ./robonex-common/setup/setup_isaacsim.sh
```
