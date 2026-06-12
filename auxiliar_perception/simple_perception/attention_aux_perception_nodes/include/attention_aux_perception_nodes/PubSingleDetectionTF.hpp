#ifndef ATTENTION_AUX_PERCEPTION_NODES__PUB_SINGLE_DETECTION_TF__HPP
#define ATTENTION_AUX_PERCEPTION_NODES__PUB_SINGLE_DETECTION_TF__HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

#include "attention_aux_perception_msgs/srv/start_single_detection_tf_pub.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

namespace attention_aux_perception_nodes
{

class PubSingleDetectionTF : public rclcpp::Node
{

public:
  PubSingleDetectionTF();
  virtual ~PubSingleDetectionTF() = default;

private:
  // TFs
  bool tf_updated_ = false;
  bool has_detection_tf_ = false;
  std::string optical_frame_id_;

  geometry_msgs::msg::TransformStamped cam2detection_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool is_subscribed_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;

  void update_tf(const vision_msgs::msg::Detection3DArray::SharedPtr detections_msg);
  void publish_detection_tf(std::string child_frame_id);

  std::string actual_detection_id_;
  std::string detection_class_name_;
  std::string frame_id_;

  // Execution
  void execute();
  rclcpp::TimerBase::SharedPtr timer_;
  const std::chrono::milliseconds CONTROL_PERIOD = 100ms;

  // Stop and start services
  rclcpp::Service<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub>::SharedPtr start_pub_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_pub_srv_;

  void start_cb(
    const std::shared_ptr<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Request> request,
    std::shared_ptr<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Response> response);
  
  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

} // namespace attention_aux_perception_nodes

#endif // ATTENTION_AUX_PERCEPTION_NODES__PUB_SINGLE_DETECTION_TF__HPP
