#!/usr/bin/env bash
# Source after activate_rosenv.sh in a fresh shell. Private, extracted ROS packages only.
_cyclone_trial="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/generated/cyclone_trial/root/opt/ros/jazzy"
if [[ ! -f "$_cyclone_trial/lib/librmw_cyclonedds_cpp.so" ]]; then
  echo "Private Cyclone DDS packages are missing" >&2
  return 1
fi
export AMENT_PREFIX_PATH="$_cyclone_trial:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_cyclone_trial/lib:$_cyclone_trial/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
unset _cyclone_trial
