import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "abb_bringup", package_name=f"{moveit_config_package.perform(context)}"
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(f"{support_package.perform(context)}"),
                "urdf",
                f"{robot_xacro_file.perform(context)}",
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                f"{moveit_config_file.perform(context)}",
            )
        )
        .planning_pipelines()
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "kinematics.yaml",
            )
        )
        # MoveIt does not handle controller switching automatically
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "moveit_controllers.yaml",
            ),
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory(
                    f"{moveit_config_package.perform(context)}"
                ),
                "config",
                "joint_limits.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("abb_irb1200_5_90_moveit_config"), "rviz"
    )
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    nodes_to_start = [move_group_node, rviz_node, static_tf_node, robot_state_pub_node]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # TODO(andyz): add other options
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            description="Xacro describing the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            description="Name of the SRDF file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
