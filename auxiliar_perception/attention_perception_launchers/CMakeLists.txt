cmake_minimum_required(VERSION 3.8)
project(attention_perception_launchers)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()