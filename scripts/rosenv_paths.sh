# Sourced by the managed rosenv activation hook and activate_rosenv.sh.
# ROS console scripts may use /usr/bin/python3 even with an active virtualenv.
_expose_workspace_rosenv() {
  local site_dir
  [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]] || return 1
  site_dir="$("$VIRTUAL_ENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')" || return 1
  case ":${PYTHONPATH:-}:" in
    *":$site_dir:"*) ;;
    *)
      _ROSENV_SAVED_PYTHONPATH_SET="${PYTHONPATH+set}"
      _ROSENV_SAVED_PYTHONPATH="${PYTHONPATH:-}"
      export PYTHONPATH="$site_dir${PYTHONPATH:+:$PYTHONPATH}"
      ;;
  esac
}
_expose_workspace_rosenv
_rosenv_paths_result=$?
unset -f _expose_workspace_rosenv
return "$_rosenv_paths_result"
