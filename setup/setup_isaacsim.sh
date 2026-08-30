if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "[setup-isaacsim] this script must be sourced, not executed:" >&2
    echo "  source ${0}" >&2
    exit 1
fi

ISAACSIM_ENV_NAME="isaacsim"
ISAACSIM_PYTHON_VERSION="3.11"
ISAACSIM_VERSION="5.1.0"
TORCH_VERSION="2.7.0"
TORCHVISION_VERSION="0.22.0"
TORCH_INDEX_URL="https://download.pytorch.org/whl/cu128"
ISAACLAB_DIR="$HOME/IsaacLab"
ISAACLAB_REF="v2.3.2"

_robonex_setup_isaacsim() {
    if ! command -v conda >/dev/null 2>&1; then
        echo "[setup-isaacsim] ERROR: conda not found; install Miniconda/Anaconda first" >&2
        return 1
    fi
    source "$(conda info --base)/etc/profile.d/conda.sh"

    if conda env list | grep -q "^${ISAACSIM_ENV_NAME} "; then
        echo "[setup-isaacsim] conda env '${ISAACSIM_ENV_NAME}' already exists, skipping creation"
    else
        echo "[setup-isaacsim] creating conda env '${ISAACSIM_ENV_NAME}' (python ${ISAACSIM_PYTHON_VERSION})..."
        if ! conda create -n "$ISAACSIM_ENV_NAME" "python=${ISAACSIM_PYTHON_VERSION}" -y; then
            echo "[setup-isaacsim] ERROR: conda create failed" >&2
            return 1
        fi
    fi

    conda activate "$ISAACSIM_ENV_NAME"
    if [ "$CONDA_DEFAULT_ENV" != "$ISAACSIM_ENV_NAME" ]; then
        echo "[setup-isaacsim] ERROR: failed to activate conda env '${ISAACSIM_ENV_NAME}'" >&2
        return 1
    fi

    if ! pip install --upgrade pip; then
        echo "[setup-isaacsim] ERROR: pip upgrade failed" >&2
        return 1
    fi

    echo "[setup-isaacsim] installing Isaac Sim ${ISAACSIM_VERSION} (this downloads several GB)..."
    if ! pip install "isaacsim[all,extscache]==${ISAACSIM_VERSION}" --extra-index-url https://pypi.nvidia.com; then
        echo "[setup-isaacsim] ERROR: Isaac Sim install failed" >&2
        return 1
    fi

    echo "[setup-isaacsim] installing PyTorch ${TORCH_VERSION} (cu128)..."
    if ! pip install "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" --index-url "$TORCH_INDEX_URL"; then
        echo "[setup-isaacsim] ERROR: PyTorch install failed" >&2
        return 1
    fi

    if [ -d "$ISAACLAB_DIR" ]; then
        echo "[setup-isaacsim] $ISAACLAB_DIR already present, skipping clone"
    else
        echo "[setup-isaacsim] cloning Isaac Lab..."
        if ! git clone https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_DIR"; then
            echo "[setup-isaacsim] ERROR: failed to clone Isaac Lab" >&2
            return 1
        fi
    fi

    if ! git -C "$ISAACLAB_DIR" checkout "$ISAACLAB_REF"; then
        echo "[setup-isaacsim] ERROR: failed to checkout Isaac Lab $ISAACLAB_REF" >&2
        return 1
    fi

    echo "[setup-isaacsim] installing system build deps (sudo)..."
    if ! sudo apt install cmake build-essential -y; then
        echo "[setup-isaacsim] ERROR: apt install failed" >&2
        return 1
    fi

    echo "[setup-isaacsim] running isaaclab.sh --install (this can take a while)..."
    if ! (cd "$ISAACLAB_DIR" && ./isaaclab.sh --install); then
        echo "[setup-isaacsim] ERROR: isaaclab.sh --install failed" >&2
        return 1
    fi

    echo "[setup-isaacsim] installing Isaac Lab's pinned extra dependencies..."
    if ! { pip install "setuptools==67.8.0" \
        && pip install --no-build-isolation "flatdict==4.0.1" \
        && pip install -e "$ISAACLAB_DIR/source/isaaclab" \
        && pip install "click==8.1.7" "psutil==5.9.8"; }; then
        echo "[setup-isaacsim] ERROR: pinned dependency install failed" >&2
        return 1
    fi

    echo "[setup-isaacsim] done. conda env '${ISAACSIM_ENV_NAME}' is active."
    echo "[setup-isaacsim] next time, just run: conda activate ${ISAACSIM_ENV_NAME}"
}

_robonex_setup_isaacsim
_robonex_setup_isaacsim_status=$?
unset -f _robonex_setup_isaacsim
return "$_robonex_setup_isaacsim_status"
