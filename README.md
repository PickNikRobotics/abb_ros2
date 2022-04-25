This is a meta-package containing everything to run an ABB robot or simulation with ROS2.

- `abb_bringup`: Launch files and ros2_control config files that are generic to many types of ABB robots.
- `abb_hardware_interface`: A ros2_control hardware interface using abb_libegm.
- `robot_specific_config`: Packages containing robot description and config files that are unique to each type of ABB robot.
- `abb_resources`: A small package containing ABB-related xacro resources.
- `docs`: More detailed documentation.
- `robot_studio_resources`: Code and a pack-and-go solution to begin using RobotStudio easily.
- `abb_ros2`: A meta-package that exists to reserve the repo name in rosdistro

## Getting Started:

There are three ways to use this package:

- With an actual, physical ABB robot

- With ROS2 simulating the robot controllers

- With an ABB RobotStudio simulation. Requires 2 PC's (one Windows, one Linux)

Detailed setup instructions can be found [here](docs/README.md).

## Limitations:

The IRB1200-5-0.9 is the only robot that has robot description and config files as of March 2022. Pull requests to add additional robot types are welcome.

## Contributing

### pre-commit Formatting Checks

This package has a pre-commit check that runs in CI. You can use this locally and set it up to run automatically before you commit something. To install, use pip:

    pip3 install pre-commit

To run over all the files in the repo manually:

    pre-commit run -a

To run pre-commit automatically before committing in a local repo, install git hooks:

    pre-commit install
