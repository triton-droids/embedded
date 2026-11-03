#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_posh::iceoryx_posh" for configuration "None"
set_property(TARGET iceoryx_posh::iceoryx_posh APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_posh::iceoryx_posh PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh.so"
  IMPORTED_SONAME_NONE "libiceoryx_posh.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_posh::iceoryx_posh )
list(APPEND _cmake_import_check_files_for_iceoryx_posh::iceoryx_posh "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh.so" )

# Import target "iceoryx_posh::iceoryx_posh_roudi" for configuration "None"
set_property(TARGET iceoryx_posh::iceoryx_posh_roudi APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_posh::iceoryx_posh_roudi PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "iceoryx_hoofs::iceoryx_hoofs"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_roudi.so"
  IMPORTED_SONAME_NONE "libiceoryx_posh_roudi.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_posh::iceoryx_posh_roudi )
list(APPEND _cmake_import_check_files_for_iceoryx_posh::iceoryx_posh_roudi "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_roudi.so" )

# Import target "iceoryx_posh::iceoryx_posh_gateway" for configuration "None"
set_property(TARGET iceoryx_posh::iceoryx_posh_gateway APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_posh::iceoryx_posh_gateway PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_gateway.so"
  IMPORTED_SONAME_NONE "libiceoryx_posh_gateway.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_posh::iceoryx_posh_gateway )
list(APPEND _cmake_import_check_files_for_iceoryx_posh::iceoryx_posh_gateway "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_gateway.so" )

# Import target "iceoryx_posh::iceoryx_posh_config" for configuration "None"
set_property(TARGET iceoryx_posh::iceoryx_posh_config APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_posh::iceoryx_posh_config PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "iceoryx_posh::iceoryx_posh;iceoryx_hoofs::iceoryx_hoofs;iceoryx_posh::iceoryx_posh_roudi"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_config.so"
  IMPORTED_SONAME_NONE "libiceoryx_posh_config.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_posh::iceoryx_posh_config )
list(APPEND _cmake_import_check_files_for_iceoryx_posh::iceoryx_posh_config "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_posh_config.so" )

# Import target "iceoryx_posh::iox-roudi" for configuration "None"
set_property(TARGET iceoryx_posh::iox-roudi APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_posh::iox-roudi PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/bin/iox-roudi"
  )

list(APPEND _cmake_import_check_targets iceoryx_posh::iox-roudi )
list(APPEND _cmake_import_check_files_for_iceoryx_posh::iox-roudi "${_IMPORT_PREFIX}/bin/iox-roudi" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
