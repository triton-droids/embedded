#!/usr/bin/env bash
# Install the simulation WBC dependencies locally; no system Python changes.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="$repo/rosenv/bin/python"
source_dir="$repo/.deps/tsid-1.7.1"
build_dir="$repo/.deps/tsid-build"
trap 'echo "WBC setup failed at line $LINENO; fix the error above and rerun." >&2' ERR
if [[ ! -x "$python" ]]; then
  echo "Create rosenv first: ROS_DISTRO=humble source scripts/setup_rosenv.sh" >&2
  exit 1
fi
for command in git patch cmake; do
  command -v "$command" >/dev/null || { echo "Missing required command: $command" >&2; exit 1; }
done
env -u PYTHONPATH "$python" -m pip install -r "$repo/requirements-wbc.txt"
prefix="$(env -u PYTHONPATH "$python" -c 'import sysconfig; print(sysconfig.get_path("purelib") + "/cmeel.prefix")')"
mkdir -p "$repo/.deps"
if [[ ! -d "$source_dir" ]]; then
  git clone --recursive --depth 1 --branch v1.7.1 \
    https://github.com/stack-of-tasks/tsid.git "$source_dir"
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != 89d61d845d91a3e1e48bd4dda7a7b28c7c476471 ]]; then
  echo "Unexpected TSID source revision in $source_dir; expected v1.7.1." >&2
  exit 1
fi
git -C "$source_dir" submodule update --init --recursive
for patch_file in "$repo"/experimental/tsid-1.7.1-*.patch; do
  if patch --batch --forward --dry-run -d "$source_dir" -p1 < "$patch_file" >/dev/null 2>&1; then
    patch --batch --forward -d "$source_dir" -p1 < "$patch_file"
  elif ! patch --batch --reverse --dry-run -d "$source_dir" -p1 < "$patch_file" >/dev/null 2>&1; then
    echo "Cannot apply or verify TSID patch: $patch_file" >&2
    exit 1
  fi
done
cmake -S "$source_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix" \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DPYTHON_EXECUTABLE="$python" \
  -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF
# Keep peak compiler memory bounded on the Jetson.
cmake --build "$build_dir" --parallel "${WBC_BUILD_JOBS:-1}"
cmake --install "$build_dir"
# TSID 1.7.1 installs __init__.py one package directory too deep with its
# bundled CMake helpers. Place it beside the extension so `import tsid` works.
python_version="$("$python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
install -m 644 "$source_dir/bindings/python/tsid/__init__.py" \
  "$prefix/lib/python$python_version/site-packages/tsid/__init__.py"
env -u PYTHONPATH "$python" - <<'PY'
import pinocchio
import tsid
assert hasattr(tsid, 'TaskActuationEquality'), 'Patched TSID binding missing'
print(f'WBC dependencies ready: Pinocchio {pinocchio.__version__}, TSID {tsid.__file__}')
PY
echo "Activate with: source scripts/activate_rosenv.sh"
