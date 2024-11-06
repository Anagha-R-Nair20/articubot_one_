import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Declare use_sim_time and use_ros2_control as launch arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true')
    use_ros2_control = DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true')

    # Path to the xacro file
    pkg_path = get_package_share_directory('articubot_one')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Generate robot description from xacro file, without sim_time passed to xacro
    robot_description_config = Command([
        'xacro ', xacro_file, 
        ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')
    ])

    # Set use_sim_time directly in the node's parameters
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': LaunchConfiguration('use_sim_time')  # Pass use_sim_time directly here
        }]
    )

    return LaunchDescription([
        use_sim_time,
        use_ros2_control,
        node_robot_state_publisher
    ])
