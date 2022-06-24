from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    robot_nickname = LaunchConfiguration("robot_nickname")
    polling_rate = LaunchConfiguration("polling_rate")
    no_connection_timeout = LaunchConfiguration("no_connection_timeout")

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="None",
            description="IP address to the robot controller's RWS server",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="80",
            description="Port number of the robot controller's RWS server",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_nickname",
            default_value="",
            description="Arbitrary user nickname/identifier for the robot controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "no_connection_timeout",
            default_value="false",
            description="Specifies whether the node is allowed to wait indefinitely \
            for the robot controller during initialization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "polling_rate",
            default_value="5.0",
            description="The frequency [Hz] at which the controller state is collected.",
        )
    )

    node = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"robot_port": robot_port},
            {"robot_nickname": robot_nickname},
            {"polling_rate": polling_rate},
            {"no_connection_timeout": no_connection_timeout},
        ],
    )
    return LaunchDescription(declared_arguments + [node])
