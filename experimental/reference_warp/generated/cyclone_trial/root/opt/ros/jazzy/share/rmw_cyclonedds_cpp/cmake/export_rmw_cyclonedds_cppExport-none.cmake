#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmw_cyclonedds_cpp::rmw_cyclonedds_cpp" for configuration "None"
set_property(TARGET rmw_cyclonedds_cpp::rmw_cyclonedds_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(rmw_cyclonedds_cpp::rmw_cyclonedds_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "CycloneDDS::ddsc;rcutils::rcutils;rcpputils::rcpputils;rmw_dds_common::rmw_dds_common_library;rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c;rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp;rosidl_runtime_c::rosidl_runtime_c;tracetools::tracetools;iceoryx_binding_c::iceoryx_binding_c"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/librmw_cyclonedds_cpp.so"
  IMPORTED_SONAME_NONE "librmw_cyclonedds_cpp.so"
  )

list(APPEND _cmake_import_check_targets rmw_cyclonedds_cpp::rmw_cyclonedds_cpp )
list(APPEND _cmake_import_check_files_for_rmw_cyclonedds_cpp::rmw_cyclonedds_cpp "${_IMPORT_PREFIX}/lib/librmw_cyclonedds_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
