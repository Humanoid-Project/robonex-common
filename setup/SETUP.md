# Setup

Shared venv bootstrap for repos that clone `robonex-common` as a sibling.

## Structure

```text
setup/
├── SETUP.md
└── setup.sh
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

`robonex_description`, `Robstride-Motor-Test`, `robonex-deploy`.

`robonex_balancing` uses `conda`, not this script.
