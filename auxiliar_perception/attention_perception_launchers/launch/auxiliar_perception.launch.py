import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def create_detection_3d_projector_node(context):
  projector_mode = LaunchConfiguration('projector_mode').perform(context)
  use_sim_time = LaunchConfiguration('use_sim_time')

  if projector_mode == 'no_depth_cam':
    executable = 'detection_3d_projector_no_depht_node'
    parameters = [{
      'use_sim_time': use_sim_time
    }]
    remappings = [
      ('camera_info', '/camera_rgb_info')
    ]
  elif projector_mode == 'astra':
    executable = 'detection_3d_projector_astra_depht_node'
    parameters = [{
      'use_sim_time': use_sim_time,
      'depth_camera_topic': '/depth/image_raw'
    }]
    remappings = [
      ('camera_info', '/color/camera_info')
    ]
  elif projector_mode == 'rgbd':
    executable = 'detection_3d_projector_rgbd_depth_node'
    parameters = [{
      'use_sim_time': use_sim_time,
      'camera_info_topic': '/rgbd_camera/camera_info',
      'depth_camera_topic': '/rgbd_camera/depth_image',
      'detections_2d_topic': '/detections_2d',
      'detections_3d_topic': '/detections_3d'
    }]
    remappings = []
  else:
    raise ValueError(
      'Invalid projector_mode. Valid values are: no_depth_cam, astra, rgbd')

  node = Node(
    package='attention_aux_perception_nodes',
    executable=executable,
    output='screen',
    remappings=remappings,
      parameters=parameters
  )

  return [node]


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )

  projector_mode_arg = DeclareLaunchArgument(
    'projector_mode',
    default_value='no_depth_cam',
    description='3D projector mode: no_depth_cam, astra or rgbd'
  )

  omdet_node = Node(
    package='omdet_node',
    executable='omdet_node',
    name='omdet_node',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time,
      'debug': True,
      'tracking': True,
      'update_tracker_without_detections': False,
      'image_topic': '/image_rgb',
      'max_age': 45,
      'min_hits': 3,
      'min_conf': 0.25,
      'max_cos_dist': 0.25,
      'max_iou_dist': 0.875,
      'nn_budget': 80,
      'mc_lambda': 0.98,
      'ema_alpha': 0.9,
      'reid_weights': 'osnet_x0_25_msmt17.pt'
    }]
  )

  detection_3d_projector_node = OpaqueFunction(
    function=create_detection_3d_projector_node)
  
  pub_single_detection_tf_node = Node(
    package='attention_aux_perception_nodes',
    executable='pub_single_detection_tf_node',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time
    }]
  )

  pub_n_detections_same_class_tf_node = Node(
    package='attention_aux_perception_nodes',
    executable='pub_n_detections_same_class_tf_node',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time,
      'publish_debug_data' : True
    }]
  )

  return LaunchDescription([
    use_sim_time_arg,
    projector_mode_arg,
    omdet_node,
    detection_3d_projector_node,
    pub_single_detection_tf_node,
    pub_n_detections_same_class_tf_node
  ])
