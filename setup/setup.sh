if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "[setup] this script must be sourced, not executed:" >&2
    echo "  source ${0} [sibling-repo ...]" >&2
    exit 1
fi

declare -A ROBONEX_GITHUB_NAME=(
    ["robonex-common"]="robonex-common"
    ["robonex-description"]="robonex-description"
    ["robstride-motor-test"]="robstride-motor-test"
    ["IMU_N100_Test"]="imu-n100-test"
    ["robonex-deploy"]="robonex-deploy"
    ["robonex-balancing"]="robonex-balancing"
    ["robonex-walking"]="robonex-walking"
)

_robonex_setup() {
    local activated_venv common_root humanoid_root repo_root repo slug target venv_python

    common_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    humanoid_root="$(dirname "$common_root")"
    repo_root="$(pwd)"

    for repo in "$@"; do
        target="$humanoid_root/$repo"
        if [ -d "$target" ]; then
            echo "[setup] $repo already present, skipping clone"
            continue
        fi
        slug="${ROBONEX_GITHUB_NAME[$repo]:-$repo}"
        echo "[setup] cloning $repo..."
        if ! git clone "https://github.com/Humanoid-Project/${slug}.git" "$target"; then
            echo "[setup] ERROR: failed to clone $repo" >&2
            return 1
        fi
    done

    if [ -d "$repo_root/.venv" ]; then
        if [ ! -x "$repo_root/.venv/bin/python" ] || [ ! -f "$repo_root/.venv/bin/activate" ]; then
            echo "[setup] ERROR: $repo_root/.venv is incomplete" >&2
            echo "[setup] Recreate this venv, then source setup.sh again." >&2
            return 1
        fi
        activated_venv="$(
            unset VIRTUAL_ENV
            source "$repo_root/.venv/bin/activate" >/dev/null 2>&1 || exit 1
            printf '%s' "${VIRTUAL_ENV:-}"
        )"
        if [ "$activated_venv" != "$repo_root/.venv" ]; then
            echo "[setup] ERROR: .venv was created for a different repository path:" >&2
            echo "  ${activated_venv:-unknown}" >&2
            echo "[setup] Expected: $repo_root/.venv" >&2
            echo "[setup] Recreate this venv after moving or renaming the repository." >&2
            return 1
        fi
    else
        echo "[setup] creating venv..."
        if ! python3 -m venv "$repo_root/.venv"; then
            echo "[setup] ERROR: venv creation failed" >&2
            return 1
        fi
    fi
    source "$repo_root/.venv/bin/activate"
    venv_python="$repo_root/.venv/bin/python"

    if [ -f "$repo_root/requirements.txt" ]; then
        echo "[setup] installing requirements.txt..."
        if ! "$venv_python" -m pip install -r "$repo_root/requirements.txt"; then
            echo "[setup] ERROR: requirements.txt install failed" >&2
            return 1
        fi
    fi

    echo "[setup] installing robonex-common (editable)..."
    if ! "$venv_python" -m pip install -e "$common_root"; then
        echo "[setup] ERROR: robonex-common install failed" >&2
        return 1
    fi

    echo "[setup] done. venv active at $repo_root/.venv"
    echo "[setup] next time, just run: source $repo_root/.venv/bin/activate"
}

_robonex_setup "$@"
_robonex_setup_status=$?
unset -f _robonex_setup
unset ROBONEX_GITHUB_NAME
return "$_robonex_setup_status"
