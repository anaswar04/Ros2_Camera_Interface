from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)

    package_dir = get_package_share_directory('camera_switcher')
    yaml_path = os.path.join(package_dir, 'camera_params.yaml')

    with open(yaml_path, 'r') as f:
        camera_map = yaml.safe_load(f)['camera_map']

    if camera_name not in camera_map:
        raise RuntimeError(f"Camera name '{camera_name}' not found in YAML map")

    camera_id = camera_map[camera_name]

    return [
        Node(
            package='camera_switcher',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{'camera_id': camera_id}]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='front',
            description='Name of the camera to use (e.g., front, rear)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
