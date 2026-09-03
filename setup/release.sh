#!/usr/bin/env bash
set -euo pipefail

REPO_SLUG="Humanoid-Project/robonex-common"
COMMON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HUMANOID_ROOT="$(dirname "$COMMON_ROOT")"

NEW_VERSION="${1:-}"
PUSH=1
[ "${2:-}" = "--no-push" ] && PUSH=0

if [ -z "$NEW_VERSION" ]; then
    echo "usage: $0 <version> [--no-push]" >&2
    echo "  example: $0 0.2.0" >&2
    exit 1
fi
if ! [[ "$NEW_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "[release] ERROR: version must be MAJOR.MINOR.PATCH, got '$NEW_VERSION'" >&2
    exit 1
fi

cd "$COMMON_ROOT"
OLD_VERSION="$(grep '^version = ' pyproject.toml | cut -d'"' -f2)"

if ! python3 - "$OLD_VERSION" "$NEW_VERSION" <<'PY'
import sys
old = tuple(int(p) for p in sys.argv[1].split("."))
new = tuple(int(p) for p in sys.argv[2].split("."))
sys.exit(0 if new > old else 1)
PY
then
    echo "[release] ERROR: $NEW_VERSION is not newer than $OLD_VERSION" >&2
    exit 1
fi

if [ -n "$(git status --porcelain -- CHANGELOG.md)" ]; then
    :
elif ! grep -q "^## $NEW_VERSION" CHANGELOG.md; then
    echo "[release] ERROR: CHANGELOG.md has no '## $NEW_VERSION' entry" >&2
    exit 1
fi

echo "[release] $OLD_VERSION -> $NEW_VERSION"

sed -i "s/^version = \"$OLD_VERSION\"/version = \"$NEW_VERSION\"/" pyproject.toml
sed -i "s/^__version__ = \"$OLD_VERSION\"/__version__ = \"$NEW_VERSION\"/" src/robonex_common/__init__.py

echo "[release] running tests..."
if ! PYTHONPATH=src python3 -m pytest tests/ -q; then
    echo "[release] ERROR: tests failed, nothing was committed" >&2
    exit 1
fi

echo "[release] rewriting pins..."
for f in $(grep -rl "robonex-common.git@v$OLD_VERSION" "$HUMANOID_ROOT" \
           --include=requirements.txt --include='*.md' --include=setup.py 2>/dev/null | grep -v '/\.venv'); do
    sed -i "s|robonex-common.git@v$OLD_VERSION|robonex-common.git@v$NEW_VERSION|g" "$f"
    echo "  $f"
done

git -C "$COMMON_ROOT" add -A
git -C "$COMMON_ROOT" commit -q -m "[Edit] Release robonex-common v$NEW_VERSION"
git -C "$COMMON_ROOT" tag -a "v$NEW_VERSION" -m "robonex-common v$NEW_VERSION"

if [ "$PUSH" = "1" ]; then
    echo "[release] pushing..."
    git -C "$COMMON_ROOT" push origin main
    git -C "$COMMON_ROOT" push origin "v$NEW_VERSION"
else
    echo "[release] --no-push: commit and tag created locally only"
fi

echo "[release] reinstalling into dependent checkouts..."
installed=""
for repo in robonex-description robonex-deploy robstride-motor-test; do
    venv="$HUMANOID_ROOT/$repo/.venv"
    if [ ! -x "$venv/bin/python" ]; then
        echo "  $repo: no .venv, skipped"
        continue
    fi
    if [ "$PUSH" = "0" ]; then
        echo "  $repo: skipped (tag not pushed yet)"
        continue
    fi
    if ! "$venv/bin/python" -m pip install -q --force-reinstall --no-deps \
        -r <(grep robonex-common "$HUMANOID_ROOT/$repo/requirements.txt"); then
        echo "  $repo: ERROR reinstalling" >&2
        continue
    fi
    installed="$("$venv/bin/python" -c 'import robonex_common as r; print(r.__version__)')"
    if [ "$installed" != "$NEW_VERSION" ]; then
        echo "  $repo: ERROR installed $installed, expected $NEW_VERSION" >&2
    else
        echo "  $repo: $installed"
    fi
done

echo "[release] done."
echo "[release] conda env 'isaacsim' is not touched - reinstall there yourself:"
echo "  pip install --force-reinstall \"robonex-common @ git+https://github.com/$REPO_SLUG.git@v$NEW_VERSION\""
