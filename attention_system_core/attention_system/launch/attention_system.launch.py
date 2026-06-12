import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
  use_sim_time = LaunchConfiguration('use_sim_time')
  pkg_dir = get_package_share_directory('attention_system')

  config_file = os.path.join(pkg_dir, 'config', 'attention_behaviors_config.yaml')
  params_file = os.path.join(pkg_dir, 'config', 'attention_orchestrator_params.yaml')

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )

  action_executor_node = Node(
    package='behavior_architecture',
    executable='mission_executor',
    output='screen',
    emulate_tty=True,
    arguments=[config_file],
    parameters=[
      params_file,
      {
        'use_sim_time': use_sim_time
      }
    ],
    remappings=[
      ('/color/camera_info', '/rgbd_camera/camera_info')
    ]
  )

  attention_subsystems_node = Node(
    package='attention_system',
    executable='attention_subsystems',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time
    }]
  )

  return LaunchDescription([
    use_sim_time_arg,
    action_executor_node,
    attention_subsystems_node
  ])
