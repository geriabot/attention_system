cmake_minimum_required(VERSION 3.8)
project(attention_system_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(attention_system_interfaces REQUIRED)
find_package(attention_actuation_msgs REQUIRED)
find_package(omdet_node_msgs REQUIRED)
find_package(attention_aux_perception_msgs REQUIRED)
find_package(vision_msgs REQUIRED)


set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  behaviortree_cpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tf2
  attention_system_interfaces
  attention_actuation_msgs
  omdet_node_msgs
  attention_aux_perception_msgs
  vision_msgs
)

include_directories(include)

add_library(change_visual_detection_class_bt_node SHARED src/attention_system_behaviors/ChangeVisualDetectionClass.cpp)
add_library(clear_visual_detection_class_bt_node SHARED src/attention_system_behaviors/ClearVisualDetectionClass.cpp)
add_library(detection_exists_bt_node SHARED src/attention_system_behaviors/DetectionExists.cpp)
add_library(detection_near_sight_limit_bt_node SHARED src/attention_system_behaviors/DetectionNearSightLimit.cpp)
add_library(tracked_detections_clustered_bt_node SHARED src/attention_system_behaviors/TrackedDetectionsClustered.cpp)
add_library(get_detections_2d_information_bt_node SHARED src/attention_system_behaviors/GetDetections2DInformation.cpp)
add_library(get_tf_compact_tree_information_bt_node SHARED src/attention_system_behaviors/GetTFCompactTreeInformation.cpp)
add_library(is_robot_out_of_bounds_bt_node SHARED src/attention_system_behaviors/IsRobotOutOfBounds.cpp)
add_library(joint_tracking_kinematically_feasible_bt_node SHARED src/attention_system_behaviors/JointTrackingKinematicallyFeasible.cpp)
add_library(n_3d_detections_same_class_exist_bt_node SHARED src/attention_system_behaviors/N3dDetectionsSameClassExist.cpp)
add_library(no_3d_detections_same_class_exist_bt_node SHARED src/attention_system_behaviors/No3dDetectionsSameClassExist.cpp)
add_library(publish_robot_out_of_bounds_reference_tf_bt_node SHARED src/attention_system_behaviors/PublishRobotOutOfBoundsReferenceTF.cpp)
add_library(start_track_tf_bt_node SHARED src/attention_system_behaviors/StartTrackTF.cpp)
add_library(tf_exists_bt_node SHARED src/attention_system_behaviors/TFExists.cpp)
add_library(publish_twist_bt_node SHARED src/attention_system_behaviors/PublishTwist.cpp)
add_library(request_behavior_input_intelligence_bt_node SHARED src/attention_system_behaviors/RequestBehaviorInputIntelligence.cpp)
add_library(genetate_single_detection_track_point_bt_node SHARED src/attention_system_behaviors/RequestSingleDetectionTrackPoint.cpp)
add_library(request_n_detections_same_class_track_point_bt_node SHARED src/attention_system_behaviors/RequestNDetectionsSameClassTrackPoint.cpp)
add_library(trigger_deactivation_service_bt_node SHARED src/attention_system_behaviors/TriggerDeactivationService.cpp)

# ... Individual plugin lib with all bt nodes
add_library(${PROJECT_NAME}_bt_plugins SHARED
  src/register_bt_plugins.cpp
  src/attention_system_behaviors/ChangeVisualDetectionClass.cpp
  src/attention_system_behaviors/ClearVisualDetectionClass.cpp
  src/attention_system_behaviors/DetectionExists.cpp
  src/attention_system_behaviors/DetectionNearSightLimit.cpp
  src/attention_system_behaviors/TrackedDetectionsClustered.cpp
  src/attention_system_behaviors/GetDetections2DInformation.cpp
  src/attention_system_behaviors/GetTFCompactTreeInformation.cpp
  src/attention_system_behaviors/IsRobotOutOfBounds.cpp
  src/attention_system_behaviors/JointTrackingKinematicallyFeasible.cpp
  src/attention_system_behaviors/N3dDetectionsSameClassExist.cpp
  src/attention_system_behaviors/No3dDetectionsSameClassExist.cpp
  src/attention_system_behaviors/PublishRobotOutOfBoundsReferenceTF.cpp
  src/attention_system_behaviors/StartTrackTF.cpp
  src/attention_system_behaviors/TFExists.cpp
  src/attention_system_behaviors/PublishTwist.cpp
  src/attention_system_behaviors/RequestBehaviorInputIntelligence.cpp
  src/attention_system_behaviors/RequestSingleDetectionTrackPoint.cpp
  src/attention_system_behaviors/RequestNDetectionsSameClassTrackPoint.cpp
  src/attention_system_behaviors/TriggerDeactivationService.cpp
)

ament_target_dependencies(${PROJECT_NAME}_bt_plugins ${dependencies})
target_compile_definitions(${PROJECT_NAME}_bt_plugins PRIVATE BT_PLUGIN_EXPORT)
ament_export_libraries(${PROJECT_NAME}_bt_plugins)
# ...

# ... One plugin lib for each bt node
list(APPEND plugin_libs
  change_visual_detection_class_bt_node
  clear_visual_detection_class_bt_node
  detection_exists_bt_node
  detection_near_sight_limit_bt_node
  tracked_detections_clustered_bt_node
  get_detections_2d_information_bt_node
  get_tf_compact_tree_information_bt_node
  is_robot_out_of_bounds_bt_node
  joint_tracking_kinematically_feasible_bt_node
  n_3d_detections_same_class_exist_bt_node
  no_3d_detections_same_class_exist_bt_node
  publish_robot_out_of_bounds_reference_tf_bt_node
  start_track_tf_bt_node
  tf_exists_bt_node
  publish_twist_bt_node
  request_behavior_input_intelligence_bt_node
  genetate_single_detection_track_point_bt_node
  request_n_detections_same_class_track_point_bt_node
  trigger_deactivation_service_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT BUILD_INDIVIDUAL_PLUGIN)
  ament_export_libraries(${bt_plugin})
endforeach()
# ...

install(TARGETS
  ${PROJECT_NAME}_bt_plugins
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY 
  behavior_tree_xml
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
