cmake_minimum_required(VERSION 3.8)
project(attention_aux_perception_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(astra_camera_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(attention_aux_perception_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
  ament_cmake
  astra_camera_msgs
  rclcpp
  vision_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  cv_bridge
  attention_aux_perception_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
  std_srvs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/Detection3dProjector.cpp
  src/${PROJECT_NAME}/Detection3dProjectorNoDepht.cpp
  src/${PROJECT_NAME}/Detection3dProjectorAstraDepht.cpp
  src/${PROJECT_NAME}/Detection3dProjectorRgbdDepth.cpp
  src/${PROJECT_NAME}/PubSingleDetectionTF.cpp
  src/${PROJECT_NAME}/PubNDetectionsSameClassTF.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(detection_3d_projector_node src/detection_3d_projector_node.cpp)
ament_target_dependencies(detection_3d_projector_node ${dependencies})
target_link_libraries(detection_3d_projector_node ${PROJECT_NAME})

add_executable(detection_3d_projector_no_depht_node src/detection_3d_projector_no_depht_node.cpp)
ament_target_dependencies(detection_3d_projector_no_depht_node ${dependencies})
target_link_libraries(detection_3d_projector_no_depht_node ${PROJECT_NAME})

add_executable(detection_3d_projector_astra_depht_node src/detection_3d_projector_astra_depht_node.cpp)
ament_target_dependencies(detection_3d_projector_astra_depht_node ${dependencies})
target_link_libraries(detection_3d_projector_astra_depht_node ${PROJECT_NAME})

add_executable(detection_3d_projector_rgbd_depth_node src/detection_3d_projector_rgbd_depth_node.cpp)
ament_target_dependencies(detection_3d_projector_rgbd_depth_node ${dependencies})
target_link_libraries(detection_3d_projector_rgbd_depth_node ${PROJECT_NAME})

add_executable(pub_single_detection_tf_node src/pub_single_detection_tf_main.cpp)
ament_target_dependencies(pub_single_detection_tf_node ${dependencies})
target_link_libraries(pub_single_detection_tf_node ${PROJECT_NAME})

add_executable(pub_n_detections_same_class_tf_node src/pub_n_detections_same_class_tf_main.cpp)
ament_target_dependencies(pub_n_detections_same_class_tf_node ${dependencies})
target_link_libraries(pub_n_detections_same_class_tf_node ${PROJECT_NAME})

install(TARGETS
  detection_3d_projector_node
  detection_3d_projector_no_depht_node
  detection_3d_projector_astra_depht_node
  detection_3d_projector_rgbd_depth_node
  pub_single_detection_tf_node
  pub_n_detections_same_class_tf_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
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
