#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run this script with source so it can activate the environment:"
  echo "  source scripts/setup_rosenv.sh"
  exit 2
fi

# This file runs in the caller's shell. Do not enable errexit here: a failed
# setup command (or a later terminal command) could close an SSH session.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)" || return 1
VENV_DIR="${REPO_ROOT}/rosenv"
REQ_FILE="${REPO_ROOT}/humanoid_control/motor_control_hybrid/requirements.txt"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}" || {
    echo "Error: could not load ROS setup at ${ROS_SETUP}" >&2
    return 1
  }
else
  echo "Warning: ROS setup not found at ${ROS_SETUP}" >&2
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  python3 -m venv "${VENV_DIR}" || {
    echo "Error: could not create rosenv at ${VENV_DIR}" >&2
    return 1
  }
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate" || {
  echo "Error: could not activate rosenv at ${VENV_DIR}" >&2
  return 1
}

if [[ ! -f "${REQ_FILE}" ]]; then
  echo "Warning: requirements file not found at ${REQ_FILE}" >&2
  return 0
fi

STAMP_FILE="${VENV_DIR}/.requirements.stamp"
if [[ ! -f "${STAMP_FILE}" || "${REQ_FILE}" -nt "${STAMP_FILE}" ]]; then
  python -m pip install -r "${REQ_FILE}" || {
    echo "Error: dependency installation failed; fix the error above and source this script again." >&2
    return 1
  }
  touch "${STAMP_FILE}" || {
    echo "Error: could not write requirements stamp at ${STAMP_FILE}" >&2
    return 1
  }
fi

echo "Activated rosenv: ${VENV_DIR}"
