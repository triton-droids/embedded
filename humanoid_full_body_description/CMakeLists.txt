cmake_minimum_required(VERSION 3.8)
project(humanoid_full_body_description)
find_package(ament_cmake REQUIRED)

# urdf/full_body.urdf.xacro is the source. Expand it at build time so every consumer
# (robot_state_publisher, MoveIt, MuJoCo) can keep loading share/.../urdf/full_body.urdf.
find_program(XACRO_EXECUTABLE xacro REQUIRED)
set(URDF_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/urdf/full_body.urdf.xacro)
set(URDF_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/full_body.urdf)
add_custom_command(
  OUTPUT ${URDF_OUTPUT}
  COMMAND ${XACRO_EXECUTABLE} ${URDF_SOURCE} -o ${URDF_OUTPUT}
  DEPENDS ${URDF_SOURCE}
  COMMENT "xacro full_body.urdf.xacro -> full_body.urdf")
add_custom_target(full_body_urdf ALL DEPENDS ${URDF_OUTPUT})

install(FILES ${URDF_SOURCE} ${URDF_OUTPUT} DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

# Detailed variant: the original per-part CAD export (1,086 meshes, 3M triangles, about 150 MB).
# Same links, joints and inertials as full_body.urdf, only heavier geometry. It is also the input for
# scripts/simplify_meshes.py. Set -DINSTALL_DETAILED_MODEL=OFF to skip it on small targets.
option(INSTALL_DETAILED_MODEL "Install urdf/full_body_detailed.urdf and its CAD meshes" ON)
if(INSTALL_DETAILED_MODEL)
  install(FILES cad_source/full_body_parts.urdf DESTINATION share/${PROJECT_NAME}/urdf
          RENAME full_body_detailed.urdf)
  install(DIRECTORY cad_source/meshes DESTINATION share/${PROJECT_NAME}/cad_source)
endif()
ament_package()
