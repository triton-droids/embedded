#!/usr/bin/env bash
# Source in a fresh shell after activating the normal workspace.
_strict_dds_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/generated/dds_strict"
if [[ ! -f "$_strict_dds_root/install-rmw/local_setup.bash" ]]; then
  echo "Private strict DDS/RMW build is missing" >&2
  return 1
fi
source "$_strict_dds_root/install-rmw/local_setup.bash"
export LD_LIBRARY_PATH="$_strict_dds_root/install-rmw/lib:$_strict_dds_root/install-fastdds/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$_strict_dds_root/install-rmw:$_strict_dds_root/install-fastdds:${CMAKE_PREFIX_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset _strict_dds_root
