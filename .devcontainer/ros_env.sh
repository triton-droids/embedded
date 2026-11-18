# Sourced from ~/.bashrc in the dev container: ROS, rosenv, and the workspace overlay.
_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO=humble source "$_repo/scripts/activate_rosenv.sh" || return 1

# Builds with exported compile commands and merges them into build/ for clangd.
# Extra arguments go to colcon build, e.g. --packages-select humanoid_vision.
_triton_repo="$_repo"
build_compile_commands() {
  (
    cd -- "$_triton_repo" || exit
    CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit
    shopt -s nullglob
    local files=(build/*/compile_commands.json)
    if (( ${#files[@]} == 0 )); then
      echo "build_compile_commands: no compile_commands.json found under build/" >&2
      exit 1
    fi
    # Write to a temp file so a jq failure leaves the previous merge intact.
    jq -s 'add' "${files[@]}" > build/compile_commands.json.tmp \
      && mv build/compile_commands.json.tmp build/compile_commands.json \
      && echo "Merged ${#files[@]} files into build/compile_commands.json"
  )
}
unset _repo
