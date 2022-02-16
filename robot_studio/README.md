# Robot Studio Simulation

Borrowed from: https://github.com/ros-industrial/abb_libegm/issues/18#issuecomment-473262645

But with some additional details abotu setting this up.

## Position Only For Now

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

Name it anything you want, for example `ROB_1`. Type should be `UDPUC` (if you don't have this you didn't add EGM to the controller).

Remote Address should be that of the ROS2 computer (in the sample solutin it's set to `192.168.1.210`).

Port should match what ros2 control driver is expecting. Right now it has to be `6511` but it will be made configurable later.

Local port should be `0`


![add transmission porotocol](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm4.png)


5. Add the code from `TRob1Main.mod` to the rapid module.

6. Under the `RAPID` tab, select TRob1Main as the task, and press the `Start` icon on the ribbon.

The simulation will then try to connect with the ros2 driver every few seconds. Once the connection is established youc an control the simulated robot from ros

Occasionally, when you connect for the first time, yuou will need to stop the rapid program and start it again...


![add transmission porotocol](https://raw.githubusercontent.com/dignakov/ros2_control_abb_driver/rolling/docs/images/egm5.png)