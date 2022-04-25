# View and view the URDF in RViz

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value=TextSubstitution(text="irb4600_60_205.xacro"),
        description="Name of a supported robot description file in package."
    )
    description_file = LaunchConfiguration("description_file")

    # robot_description_path = get_package_share_directory("abb_irb4600_support")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("abb_irb4600_support"), "urdf", description_file]
            ),
        ]
    )

    # robot_description_config = xacro.process_file(
    #     PathJoinSubstitution([
    #         robot_description_path,
    #         "urdf",
    #         description_file,
    #     ])
    # )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("abb_irb4600_support"), "rviz", "urdf_description.rviz"]
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        description_file_arg,
        robot_state_publisher_node,
        joint_state_sliders,
        rviz
    ])
