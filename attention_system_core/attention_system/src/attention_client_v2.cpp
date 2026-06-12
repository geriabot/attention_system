#include "rclcpp/rclcpp.hpp"
#include "attention_system_interfaces/srv/use_attention.hpp"

#include <vector>
#include <map>
#include <iostream>

using namespace std::chrono_literals;

using UseAttention = attention_system_interfaces::srv::UseAttention;

struct attention_request {
  std::string task_details;
  std::map<std::string, std::string> behaviors;
  // This is for example client, in real use, behaviors don't have associated available
  // capabilities. This capabilities are defined in the moment, not only by the behavior,
  // but for other things like the context, the robot morphology, etc.
  std::map<std::string, std::vector<bool>> behaviors_capabilities;
};

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("attention_client");

  auto client = node->create_client<UseAttention>("use_attention");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "UseAttention service not available after waiting");
    return 1;
  }

  std::map<std::string, attention_request> requests;

  // Gait speed
  attention_request gait_speed;
  gait_speed.task_details = "Measuring a patient's gait speed during a rehabilitation exercise. Only one patient is involved in this exercise, there are no more persons to track through the whole task.";

  gait_speed.behaviors_capabilities["explain"] = {false, true, false};
  gait_speed.behaviors_capabilities["measure"] = {false, true, false};

  gait_speed.behaviors["explain"] =
    "The robot explains the exercise to the patient.";
  gait_speed.behaviors["measure"] =
    "The robot measures patient's gait speed.";

  requests["gait_speed"] = gait_speed;

  // Sports teacher
  attention_request sports_teacher;
  sports_teacher.task_details = "Be the sports teacher of a group of persons. You are in charge of 2 persons.";

  sports_teacher.behaviors_capabilities["explain"] = {true, true, false};
  sports_teacher.behaviors_capabilities["check"] = {false, true, false};
  sports_teacher.behaviors_capabilities["correct"] = {false, true, false};

  sports_teacher.behaviors["explain"] =
    "The robot explains an exercise to all the persons, not just for one.";
  sports_teacher.behaviors["check"] =
    "The robot checks that the persons are performing the exercise correctly.";
  sports_teacher.behaviors["correct"] =
    "The robot tells a person, described as a girl with white clothes and black hair, but she is a person, that she is not well performing the exercise and will explain the exercise to her again.";

  requests["sports_teacher"] = sports_teacher;

  // Manipulation
  attention_request manipulation;
  manipulation.task_details =
    "Perform close-range hand manipulation tasks where the relevant visual information is the robot's own articulated hand, wrist, fingers and end-effector pose. The task requires continuous visual monitoring of the robot's own moving joint while the hand approaches, grasps or adjusts objects. External objects are secondary and do not need to be followed as independent detections. People, object classes, body turning and navigation are not the main source of information; the critical target to observe is the robot's own hand joint during motion.";

  manipulation.behaviors_capabilities["pick"] = {true, false, false};

  manipulation.behaviors["left_pick"] =
    "The robot uses its left hand to pick an object. During the movement, the robot must keep visual contact with its own left wrist, hand and fingers to verify alignment, finger posture, grasp closure and end-effector position. The object only provides manipulation context; it is not the main visual target. The important visual reference is the robot's own left hand joint, not a person, not a group midpoint and not a detected external object.";
  manipulation.behaviors["right_pick"] =
    "The robot uses its right hand to pick an object. During the movement, the robot must keep visual contact with its own right wrist, hand and fingers to verify alignment, finger posture, grasp closure and end-effector position. The object only provides manipulation context; it is not the main visual target. The important visual reference is the robot's own right hand joint, not a person, not a group midpoint and not a detected external object.";
  manipulation.behaviors["gaze_pick"] = // This one is wrong in purpose
    "The robot should track its gaze joint.";

  requests["manipulation"] = manipulation;
  
  std::string selected_request;
  std::string behavior;

  std::cout << "Available requests:" << std::endl;
  for (const auto & request_entry : requests) {
    std::cout << " - " << request_entry.first << std::endl;
  }
  std::cout << "Enter request:" << std::endl;
  std::cin >> selected_request;

  const auto request_it = requests.find(selected_request);
  if (request_it == requests.end()) {
    RCLCPP_ERROR(node->get_logger(), "Invalid request: %s", selected_request.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "Available behaviors for request '" << selected_request << "':" << std::endl;
  for (const auto & behavior_entry : request_it->second.behaviors) {
    std::cout << " - " << behavior_entry.first << std::endl;
  }
  std::cout << "Enter behavior:" << std::endl;
  std::cin >> behavior;

  const auto behavior_it = request_it->second.behaviors.find(behavior);
  if (behavior_it == request_it->second.behaviors.end()) {
    RCLCPP_ERROR(node->get_logger(), "Invalid behavior '%s' for request '%s'",
      behavior.c_str(), selected_request.c_str());
    rclcpp::shutdown();
    return 1;
  }

  bool can_ta = false;
  bool can_uj = false;
  bool can_ma = false;

  std::string bool_res;

  std::cout << "Can turn around? (y/n)" << std::endl;
  std::cin >> bool_res;
  can_ta = bool_res == "y";

  std::cout << "Can use joint? (y/n)" << std::endl;
  std::cin >> bool_res;
  can_uj = bool_res == "y";

  std::cout << "Can move around? (y/n)" << std::endl;
  std::cin >> bool_res;
  can_ma = bool_res == "y";

  RCLCPP_INFO(
    node->get_logger(),
    "Request: %s, behavior: %s",
    selected_request.c_str(),
    behavior_it->first.c_str());

  auto request = std::make_shared<UseAttention::Request>();
  request->task_details = request_it->second.task_details;
  request->behavior_details = behavior_it->second;

  request->can_turn_around = can_ta;
  request->can_use_joint = can_uj;
  request->can_move_around = can_ma;

  RCLCPP_INFO(node->get_logger(), "Stored capabilities");

  RCLCPP_INFO(node->get_logger(),
    "Attention requested for behavior %s.", behavior_it->first.c_str());
  auto result = client->async_send_request(request);

  auto future_return_code = rclcpp::spin_until_future_complete(node, result);

  if (future_return_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "UseAttention service call failed");
    rclcpp::shutdown();
    return 1;
  }

  auto response = result.get();

  if (response->status == UseAttention::Response::SUCCESSFUL) {
    RCLCPP_INFO(
      node->get_logger(),
      "<attention_client> Attention's frame id: %s.",
      response->frame_id.c_str());
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "<attention_client> Request failed or canceled with status: %i",
      response->status);
  }

  RCLCPP_INFO(node->get_logger(), "Stoping...");
  rclcpp::shutdown();
  return 0;
}
