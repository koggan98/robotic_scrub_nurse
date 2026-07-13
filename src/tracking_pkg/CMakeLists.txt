cmake_minimum_required(VERSION 3.8)
project(tracking_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tracking_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# moveit_msgs / moveit_ros_planning_interface are only needed by the C++ execution
# nodes (NUC) — found inside the BUILD_EXECUTION_NODES guard further down.
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ── Python node installs ──
install(
  PROGRAMS
    # Perception
    src/perception/camera_publisher.py
    src/perception/aruco_marker_manager.py
    src/perception/hand_tracker.py
    src/perception/tool_detection_node.py
    # Reasoning
    src/reasoning/grasp_geometry_node.py
    src/reasoning/tool_semantics_node.py
    src/reasoning/world_model_node.py
    src/reasoning/world_model_builder.py
    # LLM
    src/llm/llm_orchestrator_node.py
    # Interfaces
    src/interfaces/asr_node.py
    src/interfaces/handover_sound_publisher.py
    src/interfaces/annotated_image_viewer.py
    src/interfaces/scrub_nurse_logger.py
    src/interfaces/scrub_nurse_cli.py
    # Scene / visualization helpers
    src/publisher/mir_publisher.py
    src/publisher/instrument_tray_collision_publisher.py
    src/publisher/reclaim_tray_collision_publisher.py
    # Execution
    src/execution/gripper_opener_with_zeroer.py
    src/execution/reclaim_controller.py
    src/execution/grasp_approach_pose_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    src/execution/grasp_orientation_utils.py
    src/reasoning/grasp_point_utils.py
    src/reasoning/tray_geometry_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── C++ MoveIt execution nodes (NUC / robot control only) ──
# These are the only MoveIt-dependent targets and run exclusively on the NUC
# (launched by nuc_launch.py). The Jetson runs perception + RViz and never executes
# them. Both machines now run Humble, so we can no longer key on ROS_DISTRO to tell
# them apart — instead we auto-detect whether MoveIt is installed: present on the NUC
# (build the C++ nodes), absent on the Jetson (skip them, keeping MoveIt off the Jetson
# build). Override with -DBUILD_EXECUTION_NODES=ON/OFF.
if(NOT DEFINED BUILD_EXECUTION_NODES)
  find_package(moveit_ros_planning_interface QUIET)
  if(moveit_ros_planning_interface_FOUND)
    set(BUILD_EXECUTION_NODES ON)
  else()
    set(BUILD_EXECUTION_NODES OFF)
  endif()
endif()
message(STATUS "tracking_pkg: BUILD_EXECUTION_NODES=${BUILD_EXECUTION_NODES} (moveit_ros_planning_interface_FOUND=${moveit_ros_planning_interface_FOUND})")

if(BUILD_EXECUTION_NODES)
  find_package(moveit_msgs REQUIRED)
  find_package(moveit_ros_planning_interface REQUIRED)

  # Legacy MoveIt-based executor retained for direct, manual bench testing.
  # The active nuc_launch.py and llm_launch.py runtimes use skill_executor_node.
  add_executable(loop_mover src/execution/loop_mover.cpp)
  ament_target_dependencies(loop_mover rclcpp geometry_msgs sensor_msgs std_msgs moveit_msgs moveit_ros_planning_interface tracking_msgs)
  target_link_libraries(loop_mover tracking_msgs::tracking_msgs__rosidl_typesupport_cpp)

  add_executable(tool_pick_test_node src/execution/tool_pick_test_node.cpp)
  ament_target_dependencies(tool_pick_test_node rclcpp geometry_msgs std_msgs moveit_msgs moveit_ros_planning_interface tracking_msgs)
  target_link_libraries(tool_pick_test_node tracking_msgs::tracking_msgs__rosidl_typesupport_cpp)

  add_executable(joint_state_jogger_node src/execution/joint_state_jogger_node.cpp)
  ament_target_dependencies(joint_state_jogger_node rclcpp moveit_ros_planning_interface)

  # Skill executor: ROS Action wrapper around MoveIt pick / handover / release / return_home
  add_executable(skill_executor_node src/execution/skill_executor_node.cpp)
  ament_target_dependencies(skill_executor_node
    rclcpp rclcpp_action action_msgs geometry_msgs sensor_msgs std_msgs
    moveit_msgs moveit_ros_planning_interface tracking_msgs)
  target_link_libraries(skill_executor_node tracking_msgs::tracking_msgs__rosidl_typesupport_cpp)

  install(TARGETS
    loop_mover
    tool_pick_test_node
    joint_state_jogger_node
    skill_executor_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY src/publisher/collisionObjects
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/publisher/sounds
  DESTINATION share/${PROJECT_NAME}
)


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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
