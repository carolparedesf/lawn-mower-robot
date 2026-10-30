cmake_minimum_required(VERSION 3.8)
project(mow_simplified)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY config launch meshes urdf
	DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/joint_state_from_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
