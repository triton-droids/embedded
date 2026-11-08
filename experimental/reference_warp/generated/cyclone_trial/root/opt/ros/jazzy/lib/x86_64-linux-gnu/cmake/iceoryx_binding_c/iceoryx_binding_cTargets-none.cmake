#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_binding_c::iceoryx_binding_c" for configuration "None"
set_property(TARGET iceoryx_binding_c::iceoryx_binding_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_binding_c::iceoryx_binding_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "iceoryx_posh::iceoryx_posh;iceoryx_hoofs::iceoryx_hoofs"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_binding_c.so"
  IMPORTED_SONAME_NONE "libiceoryx_binding_c.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_binding_c::iceoryx_binding_c )
list(APPEND _cmake_import_check_files_for_iceoryx_binding_c::iceoryx_binding_c "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_binding_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
