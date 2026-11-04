cmake_minimum_required(VERSION 3.8)
project(autonomous_mobile_manipulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY urdf launch config worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/litter_detector.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/manipulator_controller.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mission_manager.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/odom_tf_broadcaster.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  PROGRAMS
    scripts/litter_detector.py
    scripts/manipulator_controller.py
    scripts/mission_manager.py
    scripts/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
