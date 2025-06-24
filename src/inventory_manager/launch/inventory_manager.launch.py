import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    config_color_detection = os.path.join(
        get_package_share_directory('camera'),
        'config',
        'color_detection_general.yaml'
    )
    camera_publisher = Node(
        package='camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
    )

    color_detection = Node(
        package='camera',
        executable='color_detection',
        name='color_detection',
        output='screen',
        parameters=[config_color_detection]
    )

    return LaunchDescription([
        camera_publisher,
        color_detection,
        #ExecuteProcess(
        #    cmd=['/home/adam-falkowski/Downloads/ngrok-v3-stable-linux-amd64/ngrok', 'http', '5000'],
        #    output='screen'
        #),
        ExecuteProcess(
            cmd=['/home/adam-falkowski/inventory_manager_ws/src/remote_interface/launch/run_remote_interface.sh'],
            shell=True,
            output='screen'
        )
    ])