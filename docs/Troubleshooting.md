# Troubleshooting

## EGM Connection Issues
 - Occasionally, when connecting for the first time, you will need to stop the RAPID program and start it again
 - The warning message `41822 No data from the UdpUc device[*]` indicates a connection issue between the RobotStudio and the ROS2 computer
    - Check that the communication settings are correct
    - Test the connection between the computers by pinging the ROS2 computer from the RobotStudio computer and vice-versa
    - Check that the firewall settings on the RobotStudio computer allow communication over UDP on the correct port
    - Verify that the ros2_control_node is listening on the correct UDP socket on the ROS2 computer
    - If using a container, verify that the required port is forwarded correctly
    - It may help switch the port by changing both the `robotstudio_port` hardware parameter on the ROS2 control side, and the port number under the `ROB_1` configuration on the RobotStudio side

## RWS Connection Issues
 - Try navigating to `<ROBOTSTUDIO_IP>:80?debug=true` in the browser and logging in with the default credentials (user = `Default User`, pw = `robotics`)
 - If you don't see a login prompt, the ROS2 computer is not communicating with the RWS server.
 - After logging in, if you see `RAPI Unidentified Error`, try [whitelisting the ROS computer's IP](./NetworkingConfiguration.md#adding-the-local-computer-to-the-virtual-controller-whitelist)
 - A workaround to the whitelisting solution is to [use SSH port forwarding](./NetworkingConfiguration.md#configuring-ssh-port-forwarding)