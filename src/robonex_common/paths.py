import os
import subprocess
from pathlib import Path

COMMON_REPO_NAMES = ("robonex-common",)
DESCRIPTION_REPO_NAMES = ("robonex-description", "robonex_description")
BALANCING_REPO_NAMES = ("robonex-balancing", "robonex_balancing")
WALKING_REPO_NAMES = ("robonex-walking", "robonex_walking")
DEPLOY_REPO_NAMES = ("robonex-deploy",)
IMU_REPO_NAMES = ("IMU_N100_Test", "imu-n100-test")


def resolve_repo(names, env_var=None, explicit=None, anchors=()):
    configured = explicit or (os.environ.get(env_var) if env_var else None)
    if configured:
        root = Path(configured).expanduser().resolve()
        if not root.is_dir():
            label = env_var or "configured path"
            raise FileNotFoundError(f"{label} does not exist: {root}")
        return root
    candidates = (names,) if isinstance(names, str) else tuple(names)
    search = [Path.cwd().resolve()]
    search.extend(Path(anchor).resolve() for anchor in anchors)
    search.append(Path(__file__).resolve())
    for anchor in search:
        for parent in (anchor, *anchor.parents):
            for name in candidates:
                candidate = parent / name
                if candidate.is_dir():
                    return candidate
    hint = f" or set {env_var}" if env_var else ""
    raise FileNotFoundError(f"{candidates[0]} checkout not found; clone it as a sibling{hint}")


def repo_file(names, relative_path, env_var=None, explicit=None, anchors=()):
    root = resolve_repo(names, env_var=env_var, explicit=explicit, anchors=anchors)
    target = root / relative_path
    if not target.is_file():
        raise FileNotFoundError(target)
    return target


def description_model(relative_path, explicit=None, anchors=()):
    return repo_file(
        DESCRIPTION_REPO_NAMES,
        relative_path,
        env_var="ROBONEX_DESCRIPTION_ROOT",
        explicit=explicit,
        anchors=anchors,
    )


def git_commit(path):
    return subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
