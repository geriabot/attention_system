# Adding attention behaviors

This guide describes the minimum steps required to add an attention behavior.

## Checklist

1. Create the Behavior Tree XML file in `attention_system_behaviors/behavior_tree_xml/`. See [Behavior Tree XML](#behavior-tree-xml).
2. Register a BehaviorRunner entry in `attention_system_core/attention_system/config/attention_behaviors_config.yaml`. See [Runtime registration](#runtime-registration).
3. Expose the behavior to the selection logic in `attention_system_core/attention_system/config/attention_orchestrator_params.yaml`. See [Selection configuration](#selection-configuration).
4. Add custom BehaviorTree.CPP nodes only if the existing nodes do not cover the behavior. See [Custom BT nodes](#custom-bt-nodes).
5. Register any new BT node in `attention_system_behaviors/src/register_bt_plugins.cpp`. See [Custom BT nodes](#custom-bt-nodes).
6. Ensure blackboard keys match between YAML and XML. See [Selection configuration](#selection-configuration).
7. Build only the affected packages. See [Build and validate](#build-and-validate).
8. Validate with `/use_attention` and `/attention_system/status`. See [Build and validate](#build-and-validate).

## Behavior Tree XML

Create a new XML file under `attention_system_behaviors/behavior_tree_xml/`. Existing examples are:

- `track_unknown_detection_rot.xml`
- `track_detections_same_class_midpoint_rot.xml`
- `track_joint_art.xml`

The XML must use blackboard keys that are either:

- assigned by `bt_blackboard_inputs.assigned`
- requested from the MLLM through `bt_blackboard_inputs.not_assigned`
- produced by another BT node in the same tree

## Runtime registration

Add the runner to `attention_behaviors_config.yaml`:

```yaml
behaviors:
  - name: "<behavior_runner_runtime_name>"
    behavior_file: "behavior_tree_xml/<behavior_tree>.xml"
    control_period_ms: 100
    package_name: "<behavior_tree_package>"
```

For a new behavior, use a unique `name`, point `behavior_file` to the new XML file, and keep `package_name` as `attention_system_behaviors` if the XML is stored in this package.

## Selection configuration

Expose the behavior in `attention_orchestrator_params.yaml`, for example `TrackUnknownDetectionRot`:

```yaml
/attention_orchestrator:
  ros__parameters:
    attention_behaviors:
      behaviors: ["TrackUnknownDetectionRot"]

      TrackUnknownDetectionRot:
        behavior_id: 0
        actuation_capabilities_needed:
          turn_around:
            needed: true
            alternatives: [""]
          move_around:
            needed: false
            alternatives: [""]
          use_joint:
            needed: false
            alternatives: [""]
        prompt_information:
          explanation: "The robot tracks one visual detection using its whole body."
          inputs: ["<int>id", "<string>class"]
        bt_blackboard_inputs:
          not_assigned: ["det_id", "det_class"]
          assigned: [
            "det_prompt_topic:omdet_prompt",
            "det_frame_id:attention_point_1",
            "retry_attempts:<int>5"
          ]
        node_name: "attention_track_unknown_detection_rot"
```

Rules to preserve:

- `behavior_id` must be unique.
- The behavior key must be listed in `attention_behaviors.behaviors`.
- `node_name` must match the BehaviorRunner `name` in `attention_behaviors_config.yaml`.
- Every capability listed in `actuation_capabilities` must have a matching entry under `actuation_capabilities_needed`.
- Values in `bt_blackboard_inputs.not_assigned` must be provided by the MLLM before or during tree execution.
- Values in `bt_blackboard_inputs.assigned` use `key:value`; typed values can use prefixes such as `<int>` or `<double>`.

## Custom BT nodes

If the behavior needs a new BT node:

1. Add the C++ class under `attention_system_behaviors/include/attention_system_behaviors/` and `attention_system_behaviors/src/attention_system_behaviors/`.
2. Register it in `attention_system_behaviors/src/register_bt_plugins.cpp`.
3. Keep the plugin library name available through `attention_behaviors_config.yaml`:

```yaml
plugin_libraries:
  - "libattention_system_behaviors_bt_plugins.so"
```

Prefer using existing ROS services, topics and parameters before introducing new interfaces.

## Build and validate

Build the affected packages:

```sh
cd <path_to_workspace>/<workspace>
source ~/scripts/ros2/source_jazzy.sh
colcon build --symlink-install \
  --packages-select attention_system attention_system_behaviors \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
