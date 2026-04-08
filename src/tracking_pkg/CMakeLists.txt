cmake_minimum_required(VERSION 3.8)
project(tracking_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ── Legacy node installs (existing launch files still reference these) ──
install(
  PROGRAMS
    src/hand_tracker/hand_tracker.py
    src/publisher/aruco_marker_120_publisher.py
    src/publisher/camera_publisher.py
    src/publisher/frame_publisher.py
    src/publisher/gesture_pose_publisher.py
    src/publisher/annotated_image_viewer.py
    src/publisher/frame_capture_node.py
    src/publisher/mir_publisher.py
    src/publisher/joint_states_publisher.py
    src/publisher/tool_selection.py
    src/publisher/handover_sound_publisher.py
    src/publisher/scrub_nurse_logger.py
    src/moveit_mover/gripper_mover.py
    src/moveit_mover/gripper_opener_with_zeroer.py
    src/moveit_mover/grasp_approach_pose_service.py
    src/moveit_mover/reclaim_controller.py
    src/moveit_mover/tool_frame_quaternion_helper.py
    src/socket_mover/socket_mover.py
    src/socket_mover/socket_gripper_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    src/moveit_mover/grasp_orientation_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── New architecture node installs ──
install(
  PROGRAMS
    # Perception
    src/perception/tray_perception_node.py
    src/perception/camera_publisher.py
    src/perception/frame_publisher.py
    src/perception/aruco_marker_120_publisher.py
    src/perception/hand_tracker.py
    # Reasoning
    src/reasoning/grasp_reasoning_node.py
    src/reasoning/world_model_node.py
    # LLM
    src/llm/llm_reasoning_node.py
    # Interfaces
    src/interfaces/asr_node.py
    src/interfaces/handover_sound_publisher.py
    src/interfaces/annotated_image_viewer.py
    src/interfaces/scrub_nurse_logger.py
    # Execution
    src/execution/gripper_opener_with_zeroer.py
    src/execution/reclaim_controller.py
    src/execution/grasp_approach_pose_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    src/perception/mediapipe_and_gesture.py
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
  DEPENDENCIES geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Ziele definieren
add_executable(moveit_mover src/moveit_mover/moveit_mover.cpp)
add_executable(loop_mover src/moveit_mover/loop_mover.cpp)

# Abhängigkeiten mit dem Ziel verknüpfen
ament_target_dependencies(moveit_mover rclcpp geometry_msgs sensor_msgs std_msgs moveit_ros_planning_interface)
ament_target_dependencies(loop_mover rclcpp geometry_msgs sensor_msgs std_msgs moveit_ros_planning_interface)
target_link_libraries(loop_mover "${cpp_typesupport_target}")

# Installation des ausführbaren Ziels
install(TARGETS moveit_mover
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS loop_mover
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
