#!/usr/bin/env bash
set -euo pipefail

ORG_URL="https://github.com/Humanoid-Project"
RAW_URL="https://raw.githubusercontent.com/Humanoid-Project/robonex-common/main/setup"
ROOT="${1:-$HOME/humanoid_project}"
PYTHON="${PYTHON:-python3}"

CLONES=(
    "robonex-description robonex-description"
    "robonex-deploy robonex-deploy"
    "robstride-motor-test robstride-motor-test"
    "imu-n100-test IMU_N100_Test"
)
VENV_REPOS=(robonex-description robonex-deploy robstride-motor-test)

log() { echo "[setup] $*"; }
fail() { echo "[setup] ERROR: $*" >&2; exit 1; }

command -v git >/dev/null 2>&1 || fail "git not found (sudo apt install git)"
command -v "$PYTHON" >/dev/null 2>&1 || fail "$PYTHON not found"
"$PYTHON" -c "import venv, ensurepip" 2>/dev/null \
    || fail "$PYTHON cannot create a venv (sudo apt install python3-venv)"

mkdir -p "$ROOT"
ROOT="$(cd "$ROOT" && pwd)"
log "project root: $ROOT"

for entry in "${CLONES[@]}"; do
    read -r remote dir <<<"$entry"
    target="$ROOT/$dir"
    if [ -d "$target/.git" ]; then
        log "$dir: already cloned, skipped"
    elif [ -e "$target" ]; then
        fail "$target exists but is not a git checkout; move it away and re-run"
    else
        log "$dir: cloning $ORG_URL/$remote.git"
        git clone "$ORG_URL/$remote.git" "$target"
    fi
done

for repo in "${VENV_REPOS[@]}"; do
    dir="$ROOT/$repo"
    venv="$dir/.venv"
    if [ -x "$venv/bin/python" ]; then
        log "$repo: .venv already exists"
    else
        log "$repo: creating .venv"
        "$PYTHON" -m venv "$venv"
    fi
    log "$repo: installing requirements.txt"
    "$venv/bin/python" -m pip install -q --upgrade pip
    "$venv/bin/python" -m pip install -q -r "$dir/requirements.txt"

    pinned="$(grep -o 'robonex-common\.git@v[0-9.]*' "$dir/requirements.txt" | sed 's/.*@v//')"
    installed="$("$venv/bin/python" -c 'import robonex_common as r; print(r.__version__)')"
    if [ "$installed" = "$pinned" ]; then
        log "$repo: robonex-common $installed"
    else
        log "$repo: robonex-common $installed installed, but requirements.txt pins $pinned; reinstalling"
        "$venv/bin/python" -m pip install -q --force-reinstall --no-deps \
            -r <(grep robonex-common "$dir/requirements.txt")
        installed="$("$venv/bin/python" -c 'import robonex_common as r; print(r.__version__)')"
        [ "$installed" = "$pinned" ] || fail "$repo: robonex-common is $installed, expected $pinned"
        log "$repo: robonex-common $installed"
    fi
done

n100_dir="$ROOT/robonex-deploy/scripts/policy_test"
if compgen -G "$n100_dir/n100*.so" >/dev/null; then
    log "robonex-deploy: n100 module already built, skipped"
elif ! command -v cmake >/dev/null 2>&1 || ! command -v c++ >/dev/null 2>&1; then
    log "robonex-deploy: cmake or a C++ compiler is missing, n100 module not built"
    log "  sudo apt install cmake build-essential, then re-run this script"
else
    log "robonex-deploy: building the n100 IMU module"
    cmake -S "$n100_dir" -B "$n100_dir/build" -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE="$ROOT/robonex-deploy/.venv/bin/python" >/dev/null
    cmake --build "$n100_dir/build" -j >/dev/null
    "$ROOT/robonex-deploy/.venv/bin/python" -c "import sys; sys.path.insert(0, '$n100_dir'); import n100" \
        || fail "robonex-deploy: n100 module built but does not import"
    log "robonex-deploy: n100 module ready"
fi

log "done"
log "activate a repo with: cd $ROOT/<repo> && source .venv/bin/activate"
log "Isaac Sim / Isaac Lab (walking): source <(curl -fsSL $RAW_URL/setup_isaacsim.sh)"
