cmake_minimum_required(VERSION 3.8)
project(nao_attention_actuation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(attention_actuation_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  geometry_msgs
  nao_lola_command_msgs
  nao_lola_sensor_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_ros
  attention_actuation_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/BodyTurnWithNeckCompensation.cpp
  src/${PROJECT_NAME}/TrackTFWithNeck.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(track_tf_with_neck_node src/track_tf_with_neck_main.cpp)
ament_target_dependencies(track_tf_with_neck_node ${dependencies})
target_link_libraries(track_tf_with_neck_node ${PROJECT_NAME})

add_executable(
  body_turn_with_neck_compensation_node
  src/body_turn_with_neck_compensation_main.cpp)
ament_target_dependencies(body_turn_with_neck_compensation_node ${dependencies})
target_link_libraries(body_turn_with_neck_compensation_node ${PROJECT_NAME})

install(TARGETS
  body_turn_with_neck_compensation_node
  track_tf_with_neck_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
