# ROS interfaces

This document summarizes the main public ROS interfaces exposed by the attention system packages. It is intended as an integration reference for external robot architectures and validation tools.

## Core attention system

| Name | Kind | Type | Provider | Purpose |
| ---- | ---- | ---- | -------- | ------- |
| `/use_attention` | Service | `attention_system_interfaces/srv/UseAttention` | `/attention_orchestrator` from `attention_system` | Main external entrypoint. Requests attention for a task and passes available actuation capabilities. |
| `/attention_system/status` | Topic | `attention_system_interfaces/msg/AttentionSystemStatus` | `/attention_orchestrator` from `attention_system` | Publishes the attention state: `IDLE`, `REASONING`, `RUNNING` or `FAILED`. |
| `/behavior_status` | Topic | `std_msgs/msg/String` | Behavior runners from `behavior_architecture` | Reports Behavior Tree runner status consumed by the attention orchestrator. |
| `/attention/start_intelligence` | Service | `std_srvs/srv/Trigger` | `/attention_intelligence` from `attention_system` | Activates the intelligence service endpoints used during reasoning. |
| `/attention/stop_intelligence` | Service | `std_srvs/srv/Trigger` | `/attention_intelligence` from `attention_system` | Deactivates the intelligence service endpoints. |
| `/attention/ask_intelligence_for_behavior` | Service | `attention_system_interfaces/srv/AskForAttentionBehavior` | `/attention_intelligence` from `attention_system` | Internal service used by the orchestrator to ask the MLLM which behavior should be used. |
| `/attention/ask_intelligence_for_behavior_input` | Service | `attention_system_interfaces/srv/AskForAttentionBehaviorInput` | `/attention_intelligence` from `attention_system` | Internal service used by Behavior Tree nodes to ask the MLLM for behavior-specific inputs. |

### `/use_attention` request

- `task_details` (`string`): Natural-language description of the attention task requested by the external architecture. The attention intelligence uses it to choose the most suitable configured behavior and, when needed, to infer behavior inputs.
- `behavior_details` (`string`): Optional extra information about the desired behavior. It can be left empty when the request should be solved only from `task_details` and the configured behavior catalogue.
- `can_turn_around` (`bool`): Indicates whether the current robot/platform is allowed to satisfy behaviors requiring the `turn_around` actuation capability.
- `can_move_around` (`bool`): Indicates whether the current robot/platform is allowed to satisfy behaviors requiring the `move_around` actuation capability.
- `can_use_joint` (`bool`): Indicates whether the current robot/platform is allowed to satisfy behaviors requiring the `use_joint` actuation capability.

The capability flags are matched against the configured `actuation_capabilities_needed` entries in `attention_system_core/attention_system/config/attention_orchestrator_params.yaml`.

### `/use_attention` response

- `status` (`int8`): Result of the request. Possible values are `SUCCESSFUL`, `CANCELED`, `ABORTED`, `NO_ATTENTION_BEHAVIORS_AVAILABLE` and `UNKNOWN`.
- `frame_id` (`string`): Attention target frame returned by the selected behavior when one is available. It is empty when no target frame is assigned or the request cannot be completed.

### `/attention_system/status` values

- `IDLE`: Idle state of the system.
- `REASONING`: The system is executing reasoning tasks.
- `RUNNING`: The system is running an attention behavior.
- `FAILED`: Processes in `REASONING` and `RUNNING` states failed.

## MLLM management

| Name | Kind | Type | Provider | Purpose |
| ---- | ---- | ---- | -------- | ------- |
| `/ask_llm` | Service | `llm_router_msgs/srv/GetLLMResponse` | `/llm_router` from `llm_router` | Common MLLM entrypoint used by `/attention_intelligence`. |
| `/gemini_bridge_service` | Service | `gemini_bridge_interfaces/srv/GetGeminiResponse` | `/gemini_bridge` from `google_gemini_bridge_cpp` | Remote Gemini backend used when `llm_router` runs with `mode:=remote-gemini`. |
| `/llama/generate_response` | Action | `llama_msgs/action/GenerateResponse` | `llama_ros` | Local backend used when `llm_router` runs with `mode:=local`. |

The Gemini bridge requires the `GOOGLE_GEMINI_API_KEY` environment variable before starting `gemini_bridge_node`.

## Attention actuation

| Name | Kind | Type | Provider | Purpose |
| ---- | ---- | ---- | -------- | ------- |
| `/start_tf_tracking` | Service | `attention_actuation_msgs/srv/StartTracking` | `track_tf_static` or `track_tf_with_neck_node` | Starts tracking a target TF frame. |
| `/stop_tf_tracking` | Service | `std_srvs/srv/Trigger` | `track_tf_static` or `track_tf_with_neck_node` | Stops active TF tracking. |
| `/nao_attention/turn_body_with_neck_compensation` | Action | `attention_actuation_msgs/action/TurnBodyWithNeckCompensation` | `body_turn_with_neck_compensation_node` | Turns the NAO body while compensating the neck to preserve attention toward a frame. |

## Auxiliary perception

| Name | Kind | Type | Provider | Purpose |
| ---- | ---- | ---- | -------- | ------- |
| `/omdet_prompt` | Service | `omdet_node_msgs/srv/SetDetectionPrompt` | `/omdet_node` from `omdet_node` | Changes the open-vocabulary detection prompt used by OMDet. |
| `/image_rgb` | Topic | `sensor_msgs/msg/Image` | Camera or launch remap | RGB image input consumed by `/omdet_node`. |
| `/detections_2d` | Topic | `vision_msgs/msg/Detection2DArray` | `/omdet_node` | 2D detections used by Behavior Tree perception nodes and 3D projectors. |
| `/detections_3d` | Topic | `vision_msgs/msg/Detection3DArray` | Detection projector nodes | 3D detections consumed by tracking and existence-check BT nodes. |
| `/detection_centroid_markers` | Topic | `visualization_msgs/msg/MarkerArray` | Detection projector nodes | RViz markers for projected detection centroids. |
| `/start_single_detection_tf_publisher` | Service | `attention_aux_perception_msgs/srv/StartSingleDetectionTFPub` | `pub_single_detection_tf_node` | Starts publishing a TF for one detection selected by class and id. |
| `/stop_single_detection_tf_publisher` | Service | `std_srvs/srv/Trigger` | `pub_single_detection_tf_node` | Stops the single-detection TF publisher. |
| `/start_n_detections_same_class_tf_publisher` | Service | `attention_aux_perception_msgs/srv/StartNDetectionsSameClassTFPub` | `pub_n_detections_same_class_tf_node` | Starts publishing a TF for the midpoint of `N` detections with the same class. |
| `/stop_n_detections_same_class_tf_publisher` | Service | `std_srvs/srv/Trigger` | `pub_n_detections_same_class_tf_node` | Stops the same-class midpoint TF publisher. |
| `/tracked_detection_ids` | Topic | `vision_msgs/msg/Detection2DArray` | `pub_n_detections_same_class_tf_node` | Debug/trace topic with selected tracked detections. |
| `/detection_point` | Topic | `geometry_msgs/msg/PointStamped` | `pub_n_detections_same_class_tf_node` | Debug/trace topic with the selected tracking point. |

The perception launcher selects one projector implementation through `projector_mode`: `no_depth_cam`, `astra` or `rgbd`.
