cmake_minimum_required(VERSION 3.8)
project(gripper_force_feedback)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  PROGRAMS
  #src/force_logger.py 
  #src/live_plotter.py
  #src/gripper_opener.py
  src/gripper_opener_with_zeroer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Ziele definieren
#add_executable(moveit_mover src/moveit_mover.cpp)

# Abhängigkeiten mit dem Ziel verknüpfen
#ament_target_dependencies(moveit_mover rclcpp geometry_msgs moveit_ros_planning_interface)

# Installation des ausführbaren Ziels
#install(TARGETS moveit_mover
#  DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
