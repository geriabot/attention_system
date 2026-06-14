# Validation and troubleshooting

This document provides runtime checks for the attention system. It assumes the workspace has been built and sourced.

## Basic validation commands

Check that expected nodes are running:

```sh
ros2 node list
```

Check that expected services are available:

```sh
ros2 service list
```

Check the attention state:

```sh
ros2 topic echo /attention_system/status
```

Check the LLM router mode:

```sh
ros2 param get /llm_router mode
```

Check whether the MLLM service is available:

```sh
ros2 service type /ask_llm
```

Check whether TF tracking services are available:

```sh
ros2 service type /start_tf_tracking
ros2 service type /stop_tf_tracking
```

Check the OMDet lifecycle state:

```sh
ros2 lifecycle get /omdet_node
```

## Expected runtime state

After startup, the system should expose at least:

- `/use_attention`
- `/attention_system/status`
- `/ask_llm`
- `/start_tf_tracking`
- `/stop_tf_tracking`
- `/omdet_prompt`

When using perception behaviors, `/omdet_node` must be configured and active before it can publish detections.

## Common issues

| Symptom | Likely cause | Check | Fix |
| ------- | ------------ | ----- | --- |
| `/ask_llm` is missing | `llm_router` is not running or not sourced | `ros2 service list` | Start `ros2 run llm_router llm_router --ros-args -p mode:=<local/remote-gemini>`. |
| Gemini backend does not answer | `GOOGLE_GEMINI_API_KEY` is missing or invalid | Check the terminal that launched `gemini_bridge_node` | Export `GOOGLE_GEMINI_API_KEY` before starting the node. |
| Local MLLM backend does not answer | `llama_ros` action is not running | `ros2 action list` | Start `ros2 llama launch <path_to_model_yaml_file>`. TODO: Add a validated local model YAML path. |
| Behavior Tree plugin cannot be loaded | Plugin library was not built or workspace was not sourced | Check launch output and `install/` setup sourcing | Rebuild affected packages and source `install/setup.bash`. |
| `/use_attention` returns no available behavior | Capability flags do not satisfy any behavior | Inspect `attention_orchestrator_params.yaml` | Set `can_turn_around`, `can_move_around` or `can_use_joint` according to the requested behavior. |
| Tracking starts but robot does not move | Actuation backend or command topic is missing | Check `/start_tf_tracking`, `/cmd_vel` or NAO command topics | Start the correct actuation launch and verify robot bringup. |
| Target frame cannot be tracked | TF frame does not exist or is delayed | Use `ros2 run tf2_ros tf2_echo <base_frame> <target_frame>` | Ensure perception publishes the target TF and frame names match the YAML/XML values. |
| Time-dependent nodes appear stalled | `use_sim_time` mismatch | `ros2 param get <node> use_sim_time` | Use the same `use_sim_time` value across perception, actuation and the attention system. |