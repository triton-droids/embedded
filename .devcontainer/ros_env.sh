# Sourced from ~/.bashrc in the dev container: ROS, rosenv, and the workspace overlay.
_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$_repo/rosenv/bin/activate" ]]; then
  source "$_repo/rosenv/bin/activate"
  # ros2 run executables use /usr/bin/python3; expose the venv packages to them.
  export PYTHONPATH="$_repo/rosenv/lib/python3.10/site-packages:$PYTHONPATH"
fi
if [[ -f "$_repo/install/setup.bash" ]]; then
  source "$_repo/install/setup.bash"
fi
unset _repo
