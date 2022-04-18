# -*- coding: utf-8 -*-
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    launch_abb_control = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("abb_bringup"),
                        "launch",
                        "abb_control.launch.py",
                    )
                ]
            ),
            launch_arguments={
                "description_package": "abb_irb1200_support",
                "description_file": "irb1200_5_90.xacro",
                "launch_rviz": "false",
                "moveit_config_package": " abb_irb1200_5_90_moveit_config",
                "use_fake_hardware": "true",
            }.items(),
        ),
    )
    print("hello")
    print(launch_abb_control)

    return launch.LaunchDescription(
        launch_abb_control,
    )
