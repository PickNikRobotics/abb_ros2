# Robot Web Services client

This documentation specified ROS services provided by `abb_rws_client` package for commanding ABB robots via the RWS interface.

## List of core services

Below is a list of core services for commanding the ABB robot via the RWS interface.

| Service name                       | Description                                                                                     |
|------------------------------------|-------------------------------------------------------------------------------------------------|
| `get_robot_controller_description` | Gets a description of the connected robot controller.                                           |
| `pp_to_main`                       | Sets all RAPID program pointers to respective main procedure. Starts all RAPID programs. |
| `start_rapid`                      | Starts all RAPID programs.                                                                      |
| `stop_rapid`                       | Stop all RAPID programs.                                                                        |
| `set_motors_on/off`                | Sets the motors on/off.                                                                         |
| `get/set_file_contents`            | Gets/Sets the contents of a file.                                                               |
| `get/set_io_signal`                | Gets/Sets an IO-signal.                                                                         |
| `get/set_rapid_bool`               | Gets/Sets a RAPID `bool` symbol.                                                                |
| `get/set_rapid_dnum`               | Gets/Sets a RAPID `dnum` symbol.                                                                |
| `get/set_rapid_num`                | Gets/Sets a RAPID `num` symbol.                                                                 |
| `get/set_rapid_string`             | Gets/Sets a RAPID `string` symbol.                                                              |
| `get/set_rapid_symbol `            | Gets/Sets a RAPID symbol (in raw text format).                                                  |
| `get/set_speed_ratio`              | Gets/Sets the controller speed ratio (in the range [0, 100]) for RAPID motions.                 |


## List of RobotWare StateMachine Add-in services

Below is a list of services for operating a robot via the RWS interface with the RobotWare StateMachine Add-in installed.

| Service name           | Description                                                                |
|------------------------|----------------------------------------------------------------------------|
| `run_rapid_routine`    | Signals that custom RAPID routine(s) should be run for all RAPID programs. |
| `set_rapid_routune`    | Sets desired custom RAPID routine for a specific RAPID task.               |
| `get/set_egm_settings` | Gets/Sets EGM settings used by a specific RAPID task.                      |
| `start_egm_joint`      | Starts EGM joint motions for all RAPID programs.                           |
| `start_egm_pose`       | Starts EGM pose motions for all RAPID programs.                            |
| `start_egm_stream`     | Starts EGM position streaming for all RAPID programs.                      |
| `stop_egm`             | Stops EGM motions for all RAPID programs.                                  |
| `stop_egm_stream`      | Stops EGM position streaming for all RAPID programs.                       |
| `run_sg_routine`       | Signals that SmartGripper command(s) should be run for all RAPID programs. |
| `set_sg_command`       | Sets desired SmartGripper command for a specific RAPID task.               |
