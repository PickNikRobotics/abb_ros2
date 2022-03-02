# Robot Studio Simulation

Borrowed from: https://github.com/ros-industrial/abb_libegm/issues/18#issuecomment-473262645

But with some additional details about setting up EGM. Will try to add to this over time to make it a more comprehensive guide on EGM.

## Position Only For Now

# Packaged Sim

You should be able to import `IRB1200_5_90.rspag` into Robot Studio, change the IP of the control computer and try it out.

# Setting Up a New Robot:

1. In Robot Studio, create a new solution

File --> New --> Solution with Station and Virtual Controller

Make sure you have `Customize options` selected

![create solution](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm0.png)

2. Add EGM to the controller

When customizing controller options under `Engineering Tools` choose:
- `689-1 Externally Guided Motion (EGM)`
- (not sure if needed) `623-1 Multitasking`

![customize](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm1.png)

3. Add The ROS2 computer IP and port so EGM can talk to it.

Under the `Controller Tab` --> `Configuration` --> `Communication`

On the right, udner `Transmission Protocol` right click and add new.


![communication](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm2.png)


![add transmission porotocol](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm3.png)

Name it `ROB_1`. Type should be `UDPUC` (if you don't have this you didn't add EGM to the controller).

Set the Remote Address to be that of the ROS2 computer (in the sample solution it's set to `169.254.53.52`, but will likely be different on your setup).

The port should match what ros2 control driver is expecting. This is a hardware parameter set in the robot ros2control xacro. In the sample solution, it is set to `6511`.

Local port should be `0`


![add transmission porotocol](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm4.png)


5. Add the code from `TRob1Main.mod` to the rapid module.

6. Under the `RAPID` tab, select TRob1Main as the task, and press the `Start` icon on the ribbon.

The simulation will then try to connect with the ros2 driver every few seconds. Once the connection is established, you can control the simulated robot from ROS.

Occasionally, when you connect for the first time, you will need to stop the rapid program and start it again...


![add transmission porotocol](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm5.png)