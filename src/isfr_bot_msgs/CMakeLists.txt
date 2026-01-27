cmake_minimum_required(VERSION 3.8)
project(isfr_bot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloVisionObject.msg"
  "msg/YoloVisionObjectArray.msg"
  "msg/GraspSafeObject.msg"
  "msg/GraspSafeObjectArray.msg"
  "action/SetGripperOpening.action"
  "action/SetGripperPosition.action"
  "action/FetchObject.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()