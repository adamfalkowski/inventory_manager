import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('inventory_manager'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Publishes fake joint positions via GUI sliders (no hardware/Gazebo needed)
    # Only use this when you want to manually move the robot joints in RViz. If you are using Gazebo, you don't need this node since Gazebo will publish the joint states for you.
    """ node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    ) """

    # RViz itself, pointed at a saved config file
    # When defining a Node() in a launch file you are telling the launch file to start this executable 
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    ) 

    # Launch file for gazebo since it includes many different nodes and configurations
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')
        ),
    launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'inventory_robot'],
        output='screen'
    )

    # Specific node to translate gazebo topics to ROS2 topics.
    node_gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # <topic_name>@<ros2_type>@<gz_type>
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/empty/model/inventory_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/inventory_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
        ('/world/empty/model/inventory_robot/joint_state', '/joint_states'),
        ('/model/inventory_robot/tf', '/tf'),
        ],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        node_robot_state_publisher,
        # node_joint_state_publisher_gui,
        node_rviz,
        gazebo,
        node_spawn_entity,
        node_gazebo_bridge
    ])