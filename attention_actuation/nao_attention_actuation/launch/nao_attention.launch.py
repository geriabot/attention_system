from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  track_head_frame_id_arg = DeclareLaunchArgument(
    'track_head_frame_id',
    default_value='Head',
    description='Head frame used by the TF tracking node'
  )
  track_yaw_kp_arg = DeclareLaunchArgument(
    'track_yaw_kp',
    default_value='0.1',
    description='Yaw proportional gain for the TF tracking node'
  )
  track_pitch_kp_arg = DeclareLaunchArgument(
    'track_pitch_kp',
    default_value='0.1',
    description='Pitch proportional gain for the TF tracking node'
  )
  track_max_yaw_delta_per_tick_arg = DeclareLaunchArgument(
    'track_max_yaw_delta_per_tick',
    default_value='0.2',
    description='Maximum yaw correction per tick for the TF tracking node'
  )
  track_max_pitch_delta_per_tick_arg = DeclareLaunchArgument(
    'track_max_pitch_delta_per_tick',
    default_value='0.2',
    description='Maximum pitch correction per tick for the TF tracking node'
  )
  track_yaw_deadband_arg = DeclareLaunchArgument(
    'track_yaw_deadband',
    default_value='0.01',
    description='Yaw deadband for the TF tracking node'
  )
  track_pitch_deadband_arg = DeclareLaunchArgument(
    'track_pitch_deadband',
    default_value='0.01',
    description='Pitch deadband for the TF tracking node'
  )
  track_min_head_yaw_arg = DeclareLaunchArgument(
    'track_min_head_yaw',
    default_value='-1.0',
    description='Minimum head yaw for the TF tracking node'
  )
  track_max_head_yaw_arg = DeclareLaunchArgument(
    'track_max_head_yaw',
    default_value='1.0',
    description='Maximum head yaw for the TF tracking node'
  )
  track_min_head_pitch_arg = DeclareLaunchArgument(
    'track_min_head_pitch',
    default_value='-0.5',
    description='Minimum head pitch for the TF tracking node'
  )
  track_max_head_pitch_arg = DeclareLaunchArgument(
    'track_max_head_pitch',
    default_value='0.3',
    description='Maximum head pitch for the TF tracking node'
  )
  track_publish_test_traces_arg = DeclareLaunchArgument(
    'track_publish_test_traces',
    default_value='false',
    description='Publish test trace topics from the TF tracking node'
  )

  body_reference_frame_arg = DeclareLaunchArgument(
    'body_reference_frame',
    default_value='odom',
    description='Reference frame for body turn with neck compensation'
  )
  body_base_frame_id_arg = DeclareLaunchArgument(
    'body_base_frame_id',
    default_value='base_link',
    description='Base frame for body turn with neck compensation'
  )
  body_head_frame_id_arg = DeclareLaunchArgument(
    'body_head_frame_id',
    default_value='Head',
    description='Head frame for body turn with neck compensation'
  )
  body_control_period_ms_arg = DeclareLaunchArgument(
    'body_control_period_ms',
    default_value='100',
    description='Control period in milliseconds for body turn with neck compensation'
  )
  body_turn_speed_arg = DeclareLaunchArgument(
    'body_turn_speed',
    default_value='0.2',
    description='Body turn speed for body turn with neck compensation'
  )
  body_yaw_kp_arg = DeclareLaunchArgument(
    'body_yaw_kp',
    default_value='0.1',
    description='Yaw proportional gain for body turn with neck compensation'
  )
  body_max_yaw_delta_per_tick_arg = DeclareLaunchArgument(
    'body_max_yaw_delta_per_tick',
    default_value='0.2',
    description='Maximum yaw correction per tick for body turn with neck compensation'
  )
  body_yaw_deadband_arg = DeclareLaunchArgument(
    'body_yaw_deadband',
    default_value='0.01',
    description='Yaw deadband for body turn with neck compensation'
  )
  body_min_head_yaw_arg = DeclareLaunchArgument(
    'body_min_head_yaw',
    default_value='-1.0',
    description='Minimum head yaw for body turn with neck compensation'
  )
  body_max_head_yaw_arg = DeclareLaunchArgument(
    'body_max_head_yaw',
    default_value='1.0',
    description='Maximum head yaw for body turn with neck compensation'
  )
  body_tf_lookup_timeout_ms_arg = DeclareLaunchArgument(
    'body_tf_lookup_timeout_ms',
    default_value='100',
    description='TF lookup timeout in milliseconds for body turn with neck compensation'
  )
  body_max_tf_failures_before_abort_arg = DeclareLaunchArgument(
    'body_max_tf_failures_before_abort',
    default_value='5',
    description='Maximum consecutive TF failures before aborting the body turn action'
  )

  track_tf_with_neck_node = Node(
    package='nao_attention_actuation',
    executable='track_tf_with_neck_node',
    output='screen',
    parameters=[{
      'head_frame_id': LaunchConfiguration('track_head_frame_id'),
      'yaw_kp': ParameterValue(LaunchConfiguration('track_yaw_kp'), value_type=float),
      'pitch_kp': ParameterValue(LaunchConfiguration('track_pitch_kp'), value_type=float),
      'max_yaw_delta_per_tick': ParameterValue(
        LaunchConfiguration('track_max_yaw_delta_per_tick'),
        value_type=float),
      'max_pitch_delta_per_tick': ParameterValue(
        LaunchConfiguration('track_max_pitch_delta_per_tick'),
        value_type=float),
      'yaw_deadband': ParameterValue(LaunchConfiguration('track_yaw_deadband'), value_type=float),
      'pitch_deadband': ParameterValue(LaunchConfiguration('track_pitch_deadband'), value_type=float),
      'min_head_yaw': ParameterValue(LaunchConfiguration('track_min_head_yaw'), value_type=float),
      'max_head_yaw': ParameterValue(LaunchConfiguration('track_max_head_yaw'), value_type=float),
      'min_head_pitch': ParameterValue(LaunchConfiguration('track_min_head_pitch'), value_type=float),
      'max_head_pitch': ParameterValue(LaunchConfiguration('track_max_head_pitch'), value_type=float),
      'publish_test_traces': ParameterValue(
        LaunchConfiguration('track_publish_test_traces'),
        value_type=bool)
    }]
  )

  body_turn_with_neck_compensation_node = Node(
    package='nao_attention_actuation',
    executable='body_turn_with_neck_compensation_node',
    output='screen',
    parameters=[{
      'reference_frame': LaunchConfiguration('body_reference_frame'),
      'base_frame_id': LaunchConfiguration('body_base_frame_id'),
      'head_frame_id': LaunchConfiguration('body_head_frame_id'),
      'control_period_ms': ParameterValue(
        LaunchConfiguration('body_control_period_ms'),
        value_type=int),
      'body_turn_speed': ParameterValue(LaunchConfiguration('body_turn_speed'), value_type=float),
      'yaw_kp': ParameterValue(LaunchConfiguration('body_yaw_kp'), value_type=float),
      'max_yaw_delta_per_tick': ParameterValue(
        LaunchConfiguration('body_max_yaw_delta_per_tick'),
        value_type=float),
      'yaw_deadband': ParameterValue(LaunchConfiguration('body_yaw_deadband'), value_type=float),
      'min_head_yaw': ParameterValue(LaunchConfiguration('body_min_head_yaw'), value_type=float),
      'max_head_yaw': ParameterValue(LaunchConfiguration('body_max_head_yaw'), value_type=float),
      'tf_lookup_timeout_ms': ParameterValue(
        LaunchConfiguration('body_tf_lookup_timeout_ms'),
        value_type=int),
      'max_tf_failures_before_abort': ParameterValue(
        LaunchConfiguration('body_max_tf_failures_before_abort'),
        value_type=int)
    }]
  )

  return LaunchDescription([
    track_head_frame_id_arg,
    track_yaw_kp_arg,
    track_pitch_kp_arg,
    track_max_yaw_delta_per_tick_arg,
    track_max_pitch_delta_per_tick_arg,
    track_yaw_deadband_arg,
    track_pitch_deadband_arg,
    track_min_head_yaw_arg,
    track_max_head_yaw_arg,
    track_min_head_pitch_arg,
    track_max_head_pitch_arg,
    track_publish_test_traces_arg,
    body_reference_frame_arg,
    body_base_frame_id_arg,
    body_head_frame_id_arg,
    body_control_period_ms_arg,
    body_turn_speed_arg,
    body_yaw_kp_arg,
    body_max_yaw_delta_per_tick_arg,
    body_yaw_deadband_arg,
    body_min_head_yaw_arg,
    body_max_head_yaw_arg,
    body_tf_lookup_timeout_ms_arg,
    body_max_tf_failures_before_abort_arg,
    track_tf_with_neck_node,
    body_turn_with_neck_compensation_node
  ])
