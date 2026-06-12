cmake_minimum_required(VERSION 3.8)
project(attention_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(attention_system_interfaces REQUIRED)
find_package(llm_router_msgs REQUIRED)
find_package(attention_aux_perception_msgs REQUIRED)
find_package(behavior_architecture REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  behaviortree_cpp
  rclcpp_cascade_lifecycle
  rclcpp
  std_srvs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  attention_system_interfaces
  llm_router_msgs
  attention_aux_perception_msgs
  behavior_architecture
  ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/attention_system/AttentionOrchestrator.cpp
  src/attention_system/intelligence/AttentionIntelligence.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(attention_subsystems src/attention_subsystems_main.cpp)
ament_target_dependencies(attention_subsystems ${dependencies})
target_link_libraries(attention_subsystems ${PROJECT_NAME})

add_executable(attention_client_node src/attention_client_v2.cpp)
ament_target_dependencies(attention_client_node ${dependencies})
target_link_libraries(attention_client_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  attention_subsystems
  attention_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
