#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run: source scripts/setup_rosenv.sh" >&2
  exit 2
fi

# Do not change the caller's shell flags: a failed install must not close it.
_setup_workspace_rosenv() {
  local repo distro ros_python venv fingerprint stamp old_fingerprint
  repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  distro="${ROS_DISTRO:-jazzy}"
  ros_python=/usr/bin/python3
  venv="$repo/rosenv"
  if [[ ! -f "/opt/ros/$distro/setup.bash" ]]; then
    echo "Error: /opt/ros/$distro/setup.bash is missing; install ROS first." >&2
    return 1
  fi
  if [[ ! -x "$venv/bin/python" ]]; then
    "$ros_python" -m venv "$venv" || return 1
  fi
  if [[ "$(env -u PYTHONPATH "$venv/bin/python" -c 'import sys; print(sys.version_info[:2])')" != \
        "$("$ros_python" -c 'import sys; print(sys.version_info[:2])')" ]]; then
    echo "Error: rosenv Python does not match system ROS Python; recreate rosenv for this host." >&2
    return 1
  fi
  touch "$venv/COLCON_IGNORE" || return 1

  # Hash all included requirement files, installer logic and Python/platform.
  # A stamp from the old single-file installer is deliberately not reused.
  fingerprint="$(env -u PYTHONPATH "$venv/bin/python" - "$repo" "${INSTALL_TORCH:-0}" <<'PY'
import hashlib
import platform
from pathlib import Path
import sys
root = Path(sys.argv[1])
digest = hashlib.sha256()
def add_requirements(path):
    digest.update(str(path.relative_to(root)).encode())
    digest.update(path.read_bytes())
    for line in path.read_text().splitlines():
        if line.strip().startswith('-r '):
            add_requirements((path.parent / line.strip()[3:].strip()).resolve())
add_requirements(root / 'requirements-rosenv.txt')
add_requirements(root / 'requirements-wbc.txt')
digest.update((root / 'scripts/setup_rosenv.sh').read_bytes())
digest.update((root / 'scripts/setup_wbc.sh').read_bytes())
for patch in sorted((root / 'experimental').glob('tsid-1.7.1-*.patch')):
    digest.update(patch.name.encode())
    digest.update(patch.read_bytes())
digest.update(f'{sys.version}|{platform.machine()}|torch={sys.argv[2]}'.encode())
print(digest.hexdigest())
PY
  )" || return 1
  stamp="$venv/.requirements.sha256"
  old_fingerprint="$(cat "$stamp" 2>/dev/null)" || old_fingerprint=""
  if [[ "${ROSENV_FORCE_INSTALL:-0}" == 1 || "$fingerprint" != "$old_fingerprint" ]]; then
    env -u PYTHONPATH "$venv/bin/python" -m pip install -r "$repo/requirements-rosenv.txt" || return 1
    if [[ "${INSTALL_TORCH:-0}" == 1 ]]; then
      env -u PYTHONPATH "$venv/bin/python" -m pip install torch 'numpy<2' \
        --index-url https://download.pytorch.org/whl/cpu \
        --extra-index-url https://pypi.org/simple || return 1
    fi
    # WBC requires a patched native TSID build, not the stock PyPI wheel.
    bash "$repo/scripts/setup_wbc.sh" || return 1
    printf '%s\n' "$fingerprint" > "$stamp" || return 1
  fi

  # Also make plain `source rosenv/bin/activate` work for ROS console scripts.
  # The marker makes repeated setup idempotent; no system Python is modified.
  env -u PYTHONPATH "$venv/bin/python" - "$venv/bin/activate" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
text = path.read_text()
if '# embedded rosenv package-path hook' not in text:
    marker = 'deactivate () {\n'
    if marker not in text:
        raise SystemExit('Unsupported virtualenv activation script; no deactivate function')
    cleanup = '''    # Restore the package path when leaving this virtualenv.
    if [ "${_ROSENV_SAVED_PYTHONPATH_SET+x}" ]; then
        if [ "$_ROSENV_SAVED_PYTHONPATH_SET" = set ]; then
            export PYTHONPATH="$_ROSENV_SAVED_PYTHONPATH"
        else
            unset PYTHONPATH
        fi
        unset _ROSENV_SAVED_PYTHONPATH_SET _ROSENV_SAVED_PYTHONPATH
    fi
'''
    text = text.replace(marker, marker + cleanup, 1)
    text += '\n# embedded rosenv package-path hook\nsource "$VIRTUAL_ENV/../scripts/rosenv_paths.sh"\n'
    path.write_text(text)
PY
  [[ $? == 0 ]] || return 1
  source "$repo/scripts/activate_rosenv.sh" || return 1
  echo "Activated rosenv (control, ONNX, MuJoCo, WBC and test dependencies): $venv"
}
_setup_workspace_rosenv
_setup_rosenv_result=$?
unset -f _setup_workspace_rosenv
return "$_setup_rosenv_result"
