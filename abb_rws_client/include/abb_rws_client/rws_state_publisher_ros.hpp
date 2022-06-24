/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

// This file is a modified copy from
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/include/abb_rws_service_provider/
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/include/abb_rws_state_publisher/

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/rapid_task_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

namespace abb_rws_client
{
class RWSStatePublisherROS
{
public:
  /**
   * \brief Creates a state publisher.
   *
   * \param node ROS2 node.
   * \param robot_ip IP address for the robot controller's RWS server.
   * \param robot_poty Port number for the robot controller's RWS server.
   */
  RWSStatePublisherROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port);

private:
  /**
   * \brief Time callback for receving and publishing of system states from robot.
   */
  void timer_callback();

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * \brief Manager for handling RWS communication with the robot controller.
   */
  abb::robot::RWSManager rws_manager_;

  /**
   * \brief Description of the connected robot controller.
   */
  abb::robot::RobotControllerDescription robot_controller_description_;

  /**
   * \brief Publisher for joint states.
   */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /**
   * \brief Publisher for general system states.
   */
  rclcpp::Publisher<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  /**
   * \brief Publisher for RobotWare StateMachine Add-In runtime states.
   *
   * Note: Only used if the Add-In is present in the system.
   */
  rclcpp::Publisher<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_pub_;

  /**
   * \brief Motion data for each mechanical unit defined in the robot controller.
   */
  abb::robot::MotionData motion_data_;

  /**
   * \brief Data about the robot controller's system state.
   */
  abb::robot::SystemStateData system_state_data_;
};

}  // namespace abb_rws_client
