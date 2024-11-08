import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command , FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false','use_ros2_control': 'true'}.items()
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("articubot_one"), "description", "robot.urdf.xacro"]
            ),
            " ",
            "use_ros2_control:=true"

        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("articubot_one"),
            "config",
            "my_controllers.yaml",
        ]
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_publisher': robot_description},
                    robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    delayed_controller_manager = TimerAction(period=10.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_drive_node",
        arguments=["diff_cont", "-c", "/controller_manager"],
    )


    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_board_node",
        arguments=["joint_broad","--controller-manager","/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    return LaunchDescription([
        rsp,    
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])