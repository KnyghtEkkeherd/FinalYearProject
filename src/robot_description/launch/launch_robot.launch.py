import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'robot_description'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',  
            'use_ros2_control': 'true'  
        }.items()
    )

    # ! apparently, im a ghost?
    # controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml') # --> NO WORK
    controller_params_file = "/tmp/my_controllers.yaml" # --> WORKS BTW LMAO
    # controller_params_file = "/home/gyattbot/FinalYearProject/src/robot_description/config/my_controllers.yaml" # --> NO WORK!?!?!?!?

    print(f"ctlr param file: {controller_params_file}")

    unstampedTwistToStampedTwist = Node(
        package="twistHandler",
        executable="twistHandler",
    )

    # !!! I AM SHITTING MYSELF !!!
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],  
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
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
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        unstampedTwistToStampedTwist,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
