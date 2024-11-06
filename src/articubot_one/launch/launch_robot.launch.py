import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    package_name = 'articubot_one'

    # Include rsp.launch.py with configurations for sim time and ROS2 control
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Include twist_mux configuration
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Pass `robot_description` to controller_manager node as a parameter
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        # Fetching the `robot_description` directly from `robot_state_publisher`
        remappings=[('robot_description', '/robot_state_publisher/robot_description')]
    )

    delayed_controller_manager = TimerAction(period=10.0, actions=[controller_manager])

    # Define spawner nodes with dependencies
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/controller_manager"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad","--controller-manager", "/controller_manager"],
    )
    delayed_diff_drive_spawner = TimerAction(period=30.0, actions=[diff_drive_spawner])
    delayed_joint_broad_spawner = TimerAction(period=30.0, actions=[joint_broad_spawner])

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_spawner],
    #     )
    # )
    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_broad_spawner],
    #     )
    # )

    # Launch all nodes and processes
    return LaunchDescription([
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
