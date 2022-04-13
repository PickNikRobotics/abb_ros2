# -*- coding: utf-8 -*-
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import os
import sys
import unittest


def generate_test_description():

    launch_abb_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('abb_bringup'), 'launch', 'abb_control.launch.py')]),
        launch_arguments={
            'description_package': 'abb_irb1200_support',
            'description_file' : 'irb1200_5_90.xacro',
            'launch_rviz':'false',
            'moveit_config_package' :' abb_irb1200_5_90_moveit_config',
            'use_fake_hardware' : 'true',
            }.items(),
    ),
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("abb_irb1200_support"), "urdf", "irb1200_5_90.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            "true",
            " ",
            "fake_sensor_commands:=",
            "false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("abb_bringup"), "config", "abb_controllers.yaml"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    test_command_topics = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "test_command_topics",
            ]
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            # launch_abb_control[0],
            test_command_topics,
            joint_state_broadcaster_spawner,
            initial_joint_controller_spawner,
            control_node,
            robot_state_publisher_node,
            TimerAction(period=2.0, actions=[test_command_topics]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "test_command_topics": test_command_topics,
        "launch_abb_control": launch_abb_control,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, test_command_topics):
        self.proc_info.assertWaitForShutdown(test_command_topics, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, test_command_topics):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_command_topics
        )
