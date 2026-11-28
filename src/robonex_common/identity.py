import os
from pathlib import Path

ROBOT_IDENTITY_ENV = "ROBONEX_ROBOT_MODEL_FILE"
ROBOT_IDENTITY_FILE = Path.home() / ".config" / "robonex" / "robot_model"


class IdentityError(ValueError):
    pass


def identity_path():
    configured = os.environ.get(ROBOT_IDENTITY_ENV)
    return Path(configured).expanduser() if configured else ROBOT_IDENTITY_FILE


def read_identity(path=None):
    path = identity_path() if path is None else Path(path)
    try:
        value = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None
    except (OSError, UnicodeDecodeError) as error:
        reason = getattr(error, "strerror", None) or error
        raise IdentityError(f"cannot read the robot identity file {path}: {reason}") from None
    return value or None


def require_identity(path=None):
    path = identity_path() if path is None else Path(path)
    value = read_identity(path)
    if value is None:
        raise IdentityError(
            f"no robot identity file: write the attached robot's model once with "
            f"`mkdir -p {path.parent} && echo ver2 > {path}`; motors will not be moved"
        )
    return value
