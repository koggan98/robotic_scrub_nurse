cmake_minimum_required(VERSION 3.8)
project(tracking_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
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
    src/llm/llm_reasoning_node.py
    # Interfaces
    src/interfaces/asr_node.py
    src/interfaces/handover_sound_publisher.py
    src/interfaces/annotated_image_viewer.py
    src/interfaces/scrub_nurse_logger.py
    src/interfaces/scrub_nurse_cli.py
    # Scene / visualization helpers
    src/publisher/mir_publisher.py
    src/publisher/tray_camera_volume_publisher.py
    # Execution
    src/execution/gripper_opener_with_zeroer.py
    src/execution/reclaim_controller.py
    src/execution/grasp_approach_pose_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    src/execution/grasp_orientation_utils.py
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
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrientedBoundingBox2D.msg"
  "msg/ToolDetection.msg"
  "msg/ToolDetectionArray.msg"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/HandState.msg"
  "msg/SystemState.msg"
  "msg/ExecutionPlan.msg"
  "srv/GetGraspApproachPose.srv"
  "srv/GetWorldState.srv"
  "srv/GetToolCandidates.srv"
  "srv/BuildWorldModel.srv"
  "srv/GetWorldModel.srv"
  "action/PickTool.action"
  "action/HandoverTool.action"
  "action/ReleaseTool.action"
  "action/ReturnHome.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


# C++ executable: legacy MoveIt-based executor (still referenced by llm_launch.py
# until the new Python execution layer replaces it).
add_executable(loop_mover src/execution/loop_mover.cpp)
ament_target_dependencies(loop_mover rclcpp geometry_msgs sensor_msgs std_msgs moveit_msgs moveit_ros_planning_interface)
target_link_libraries(loop_mover "${cpp_typesupport_target}")

add_executable(tool_pick_test_node src/execution/tool_pick_test_node.cpp)
ament_target_dependencies(tool_pick_test_node rclcpp geometry_msgs std_msgs moveit_msgs moveit_ros_planning_interface)
target_link_libraries(tool_pick_test_node "${cpp_typesupport_target}")

add_executable(joint_state_jogger_node src/execution/joint_state_jogger_node.cpp)
ament_target_dependencies(joint_state_jogger_node rclcpp moveit_ros_planning_interface)

# Skill executor: ROS Action wrapper around MoveIt pick / handover / release / return_home
add_executable(skill_executor_node src/execution/skill_executor_node.cpp)
ament_target_dependencies(skill_executor_node
  rclcpp rclcpp_action action_msgs geometry_msgs sensor_msgs std_msgs
  moveit_msgs moveit_ros_planning_interface)
target_link_libraries(skill_executor_node "${cpp_typesupport_target}")

install(TARGETS
  loop_mover
  tool_pick_test_node
  joint_state_jogger_node
  skill_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

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
