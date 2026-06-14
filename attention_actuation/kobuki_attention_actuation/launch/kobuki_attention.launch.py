import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  use_sim = LaunchConfiguration('use_sim')
  use_sim_time = LaunchConfiguration('use_sim_time')

  use_sim_arg = DeclareLaunchArgument(
    'use_sim',
    default_value='false',
    description='Use Kobuki simulation launch instead of real robot launch'
  )

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )

  base_frame_id_arg = DeclareLaunchArgument(
    'base_frame_id',
    default_value='base_link',
    description='Base frame used by the TF tracking node'
  )

  cmd_vel_topic_arg = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='/cmd_vel',
    description='Velocity command topic used by the TF tracking node'
  )

  control_period_ms_arg = DeclareLaunchArgument(
    'control_period_ms',
    default_value='100',
    description='Control period in milliseconds for the TF tracking node'
  )

  yaw_kp_arg = DeclareLaunchArgument(
    'yaw_kp',
    default_value='4.0',
    description='Yaw proportional gain for the TF tracking node'
  )

  yaw_ki_arg = DeclareLaunchArgument(
    'yaw_ki',
    default_value='0.0',
    description='Yaw integral gain for the TF tracking node'
  )

  yaw_kd_arg = DeclareLaunchArgument(
    'yaw_kd',
    default_value='0.01',
    description='Yaw derivative gain for the TF tracking node'
  )

  max_angular_speed_arg = DeclareLaunchArgument(
    'max_angular_speed',
    default_value='1.5',
    description='Maximum angular speed for the TF tracking node'
  )

  yaw_deadband_arg = DeclareLaunchArgument(
    'yaw_deadband',
    default_value='0.001',
    description='Yaw deadband for the TF tracking node'
  )

  publish_test_traces_arg = DeclareLaunchArgument(
    'publish_test_traces',
    default_value='true',
    description='Publish test trace topics from the TF tracking node'
  )

  kobuki_launch = GroupAction(
    actions=[
      SetRemap(
        dst='/image_rgb',
        src='/color/image_raw'
      ),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(
            get_package_share_directory('kobuki'),
            'launch',
            'kobuki.launch.py'
          )
        ),
        launch_arguments={
          'astra': 'true'
        }.items()
      )
    ],
    condition=UnlessCondition(use_sim)
  )

  kobuki_sim_launch = GroupAction(
    actions=[
      SetRemap(
        dst='/image_rgb',
        src='/rgbd_camera/image'
      ),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(
            get_package_share_directory('kobuki'),
            'launch',
            'simulation.launch.py'
          )
        ),
        launch_arguments={
          'use_integrated_gpu' : 'true'
        }.items()
      )
    ],
    condition=IfCondition(use_sim)
  )

  track_tf_static_node = Node(
    package='kobuki_attention_actuation',
    executable='track_tf_static',
    output='screen',
    parameters=[{
      'use_sim_time' : ParameterValue(use_sim_time, value_type=bool),
      'base_frame_id' : LaunchConfiguration('base_frame_id'),
      'cmd_vel_topic' : LaunchConfiguration('cmd_vel_topic'),
      'control_period_ms' : ParameterValue(
        LaunchConfiguration('control_period_ms'),
        value_type=int),
      'yaw_kp' : ParameterValue(LaunchConfiguration('yaw_kp'), value_type=float),
      'yaw_ki' : ParameterValue(LaunchConfiguration('yaw_ki'), value_type=float),
      'yaw_kd' : ParameterValue(LaunchConfiguration('yaw_kd'), value_type=float),
      'max_angular_speed' : ParameterValue(
        LaunchConfiguration('max_angular_speed'),
        value_type=float),
      'yaw_deadband' : ParameterValue(LaunchConfiguration('yaw_deadband'), value_type=float),
      'publish_test_traces' : ParameterValue(
        LaunchConfiguration('publish_test_traces'),
        value_type=bool)
    }]
  )

  return LaunchDescription([
    use_sim_arg,
    use_sim_time_arg,
    base_frame_id_arg,
    cmd_vel_topic_arg,
    control_period_ms_arg,
    yaw_kp_arg,
    yaw_ki_arg,
    yaw_kd_arg,
    max_angular_speed_arg,
    yaw_deadband_arg,
    publish_test_traces_arg,
    kobuki_launch,
    kobuki_sim_launch,
    track_tf_static_node
  ])
