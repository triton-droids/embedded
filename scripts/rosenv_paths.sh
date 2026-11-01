# Sourced by the managed rosenv activation hook and activate_rosenv.sh.
# ROS console scripts may use /usr/bin/python3 even with an active virtualenv.
_expose_workspace_rosenv() {
  local site_dir package_dir
  [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]] || return 1
  site_dir="$("$VIRTUAL_ENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')" || return 1
  # Keep pip packages first, but expose Ubuntu's ROS GUI dependencies (PyQt5,
  # sip, etc.) to the isolated virtualenv as well.
  for package_dir in "$site_dir" /usr/lib/python3/dist-packages; do
    [[ -d "$package_dir" ]] || continue
    case ":${PYTHONPATH:-}:" in
      *":$package_dir:"*) continue ;;
    esac
    if [[ ! ${_ROSENV_SAVED_PYTHONPATH_SET+x} ]]; then
      _ROSENV_SAVED_PYTHONPATH_SET="${PYTHONPATH+set}"
      _ROSENV_SAVED_PYTHONPATH="${PYTHONPATH:-}"
    fi
    if [[ "$package_dir" == "$site_dir" ]]; then
      export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"
    else
      export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$package_dir"
    fi
  done
}
_expose_workspace_rosenv
_rosenv_paths_result=$?
unset -f _expose_workspace_rosenv
return "$_rosenv_paths_result"
