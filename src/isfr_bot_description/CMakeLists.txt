cmake_minimum_required(VERSION 3.8)
project(isfr_bot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)

# Install robot description files
install(DIRECTORY urdf meshes protos
        DESTINATION share/${PROJECT_NAME})

ament_package()