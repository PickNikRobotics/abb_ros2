This is a meta-package containing everything to run an ABB robot with ROS2.

- `abb_bringup`: Launch files and ros2_control config files that are generic to many types of ABB robots.
- `ros2_control_abb_driver`: A ros2_control hardware interface using abb_libegm.
- `robot_specific_config`: Packages containing robot description and config files that are unique to each type of ABB robot.
- `abb_resources`: A small package containing ABB-related xacro resources.
- `docs`: More detailed documentation.
- `robot_studio_resources`: Code and a pack-and-go solution to begin using RobotStudio easily.

## Limitations:

The IRB1200-5-0.9 is the only robot that has robot description and config files as of March 2022. Pull requests to add additional robot types are welcome.

## Further instructions:

A real robot or RobotStudio compatible with libegm is required. RobotStudio testing setup instructions can be found [here](docs/README.md).
