cmake_minimum_required(VERSION 3.8)
project(tracking_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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
  "action/GraspTool.action"
  "action/HandoverTool.action"
  "action/ReleaseTool.action"
  "action/ReturnHome.action"
  "action/ReturnTool.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs moveit_msgs
)

ament_package()
