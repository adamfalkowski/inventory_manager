import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('camera'),
        'config',
        'color_detection_general.yaml'
    )

    color_detection = Node(
        package='camera',
        executable='color_detection',
        name='color_detection',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        color_detection
    ])