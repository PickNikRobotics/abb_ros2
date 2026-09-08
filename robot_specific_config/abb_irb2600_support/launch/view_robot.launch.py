# View and view the URDF in RViz

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution, 
    Command, 
    FindExecutable
)
from ament_index_python.packages import get_package_share_directory


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

    # Initialize Arguments
    robot_description_file = LaunchConfiguration("robot_description_file", default="irb2600_12_185.xacro")

    robot_description_file_arg = DeclareLaunchArgument(
        "robot_description_file",
        default_value="irb2600_12_185.xacro"
    )

    robot_description_path = FindPackageShare("abb_irb2600_support")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [robot_description_path, "urdf", robot_description_file]
            ),
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

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

    rviz_config_file = PathJoinSubstitution(
        [robot_description_path, "rviz", "urdf_description.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        output="screen",
    )

    return LaunchDescription([robot_description_file_arg, robot_state_publisher_node, joint_state_sliders, rviz])
