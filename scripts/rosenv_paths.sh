# Sourced by the managed rosenv activation hook and activate_rosenv.sh.
# ROS console scripts may use /usr/bin/python3 even with an active virtualenv.
_expose_workspace_rosenv() {
  local site_dir cmeel_dir package_dir path
  [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]] || return 1
  site_dir="$("$VIRTUAL_ENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')" || return 1
  # cmeel wheels (Pinocchio 2.7, TSID) live in a nested site dir that cmeel.pth
  # appends after PYTHONPATH. A ROS package of the same name (e.g. Humble's
  # ros-humble-pinocchio 4.x in /opt/ros) would then win and TSID fails to load
  # (libpinocchio.so not found), so put the rosenv dirs first explicitly.
  cmeel_dir="$("$VIRTUAL_ENV/bin/python" -c 'import sys; print(f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages")')" || return 1
  cmeel_dir="$site_dir/cmeel.prefix/$cmeel_dir"
  if [[ ! ${_ROSENV_SAVED_PYTHONPATH_SET+x} ]]; then
    _ROSENV_SAVED_PYTHONPATH_SET="${PYTHONPATH+set}"
    _ROSENV_SAVED_PYTHONPATH="${PYTHONPATH:-}"
  fi
  # Keep pip packages first (moved to the front again if a ROS setup script
  # prepended its paths since), then the cmeel packages.
  for package_dir in "$cmeel_dir" "$site_dir"; do
    [[ -d "$package_dir" ]] || continue
    path=":${PYTHONPATH:-}:"
    path="${path//:$package_dir:/:}"
    path="${path#:}"
    path="${path%:}"
    export PYTHONPATH="$package_dir${path:+:$path}"
  done
  # Same for the shared libraries: LD_LIBRARY_PATH (with /opt/ros/<distro>/lib
  # from the ROS setup) is searched before the wheels' RUNPATH, so ROS's
  # libeigenpy.so 3.13 was loaded under Pinocchio 2.7 and crashed it. This also
  # lets `import tsid` load libpinocchio.so without importing pinocchio first.
  # Not restored on `deactivate`.
  package_dir="$site_dir/cmeel.prefix/lib"
  if [[ -d "$package_dir" ]]; then
    path=":${LD_LIBRARY_PATH:-}:"
    path="${path//:$package_dir:/:}"
    path="${path#:}"
    path="${path%:}"
    export LD_LIBRARY_PATH="$package_dir${path:+:$path}"
  fi
  # Expose Ubuntu's ROS GUI dependencies (PyQt5, sip, etc.) as well, last.
  package_dir=/usr/lib/python3/dist-packages
  if [[ -d "$package_dir" ]]; then
    case ":${PYTHONPATH:-}:" in
      *":$package_dir:"*) ;;
      *) export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$package_dir" ;;
    esac
  fi
}
_expose_workspace_rosenv
_rosenv_paths_result=$?
unset -f _expose_workspace_rosenv
return "$_rosenv_paths_result"
