# Keep third-party options scoped to this function. Source builds use only the
# CAN adapter, linked statically so deployment does not require plugin files.
function(motor_control_find_encos_driver)
  find_package(encos_driver 4.0 QUIET)
  if(TARGET Encos::encos_driver)
    message(STATUS "ENCOS: using installed encos_driver")
    return()
  endif()
  if(NOT MOTOR_CONTROL_FETCH_ENCOS)
    message(FATAL_ERROR
      "encos_driver >= 4.0 was not found and downloading is disabled. "
      "Set encos_driver_DIR, enable MOTOR_CONTROL_FETCH_ENCOS, or disable "
      "MOTOR_CONTROL_BUILD_ENCOS.")
  endif()

  include(FetchContent)
  # v4.0.0, reviewed API; never follow a moving branch during configuration.
  FetchContent_Declare(encos_driver
    GIT_REPOSITORY https://github.com/EncosTech/encos_driver.git
    GIT_TAG 5ae8c3a92f6387369b674bfe7563bc053a460075
    GIT_SUBMODULES ""
  )
  set(ENCOS_PLUGINS_LIST "can")
  set(ENCOS_STATIC_MODE ON)
  set(ENCOS_ENABLE_INSTALL OFF)
  set(ENCOS_ENABLE_RELAY OFF)
  set(ENCOS_ENABLE_PYTHON ON)
  set(ENCOS_ENABLE_TRACY OFF)
  set(ENCOS_ENABLE_TSAN OFF)
  set(ENCOS_BUILD_TESTS OFF)
  set(ENCOS_BUILD_DOCS OFF)
  set(ENCOS_BUILD_WASM_BINDINGS OFF)
  set(ENCOS_BUILD_WASM_TESTS OFF)
  message(STATUS "ENCOS: fetching pinned encos_driver 4.0.0 (static CAN adapter)")
  FetchContent_MakeAvailable(encos_driver)
  # The pinned upstream CAN target includes the generated model header but
  # does not depend on its generator; enforce ordering on a clean checkout.
  add_dependencies(CanHandle motor_params_generated)
  add_library(Encos::encos_driver ALIAS EncosMotorDriver)

  # Preserve the upstream notices when distributing the statically linked node.
  install(FILES
    "${encos_driver_SOURCE_DIR}/LICENSE"
    "${encos_driver_SOURCE_DIR}/COPYING"
    DESTINATION share/${PROJECT_NAME}/licenses/encos_driver
  )
  install(FILES "${encos_driver_SOURCE_DIR}/include/external/LICENSE.MIT"
    DESTINATION share/${PROJECT_NAME}/licenses/encos_driver/json
  )
endfunction()
