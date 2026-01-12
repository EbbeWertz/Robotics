cmake_minimum_required(VERSION 3.8)
project(isfr_bot_bringup)

find_package(ament_cmake REQUIRED)

# Only install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
