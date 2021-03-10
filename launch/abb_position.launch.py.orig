import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    package_path = get_package_share_directory('ros2_control_demo_driver')
    robot_description_file = os.path.join(package_path, 'description', 'irb120.urdf')
    robot_controller_file = os.path.join(package_path, 'controllers', 'robot6dof_controller_position.yaml')
    
    with open(robot_description_file, 'r') as infile:
        descr = infile.read()
    robot_description = {'robot_description': descr}


    return LaunchDescription([
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controller_file],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ])
