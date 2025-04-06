cmake_minimum_required(VERSION 3.5)
project(drive_crusher_pkg)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  move_to_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
