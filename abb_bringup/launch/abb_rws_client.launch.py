from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    robot_nickname = LaunchConfiguration("robot_nickname")
    polling_rate = LaunchConfiguration("polling_rate")
    no_connection_timeout = LaunchConfiguration("no_connection_timeout")

    return [
        Node(
            package="abb_hardware_interface",
            executable="rws_client",
            name="rws_client",
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "robot_port": int(robot_port.perform(context)),
                    "robot_nickname": robot_nickname,
                    "polling_rate": float(polling_rate.perform(context)),
                    "no_connection_timeout": bool(
                        no_connection_timeout.perform(context)
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("robot_port"),
            DeclareLaunchArgument("robot_nickname"),
            DeclareLaunchArgument("polling_rate"),
            DeclareLaunchArgument("no_connection_timeout"),
            OpaqueFunction(function=launch_setup),
        ]
    )
