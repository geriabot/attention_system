import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


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
      'use_sim_time' : use_sim_time,
      'publish_test_traces' : True,
      'yaw_kp' : 4.0,
      'yaw_deadband' : 0.001,
      'yaw_kd' : 0.01,
      "max_angular_speed" : 1.5,
      'max_tf_age_seconds' : 0.5
    }]
  )

  return LaunchDescription([
    use_sim_arg,
    use_sim_time_arg,
    kobuki_launch,
    kobuki_sim_launch,
    track_tf_static_node
  ])
