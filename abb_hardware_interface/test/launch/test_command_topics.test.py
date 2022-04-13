# -*- coding: utf-8 -*-
import launch
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch_testing
import os
import sys
import unittest


def generate_test_description():

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
            test_command_topics,
            # TimerAction(period=2.0, actions=[basic_test]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "test_command_topics": test_command_topics,
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
