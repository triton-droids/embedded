cmake_minimum_required(VERSION 3.8)
project(humanoid_full_body_simplified)
find_package(ament_cmake REQUIRED)
# urdf/full_body.urdf is pre-expanded from urdf/full_body.urdf.xacro; re-run xacro after editing the xacro.
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
