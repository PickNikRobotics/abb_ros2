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
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/src
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/src

#include "abb_rws_client/rws_state_publisher_ros.hpp"

#include "abb_rws_client/mapping.hpp"
#include "abb_rws_client/utilities.hpp"

namespace
{
/**
 * \brief Time [s] for throttled ROS logging.
 */
constexpr double THROTTLE_TIME{ 10.0 };
}  // namespace

namespace abb_rws_client
{
RWSStatePublisherROS::RWSStatePublisherROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip,
                                           unsigned short robot_port)
  : RWSClient(node, robot_ip, robot_port)
{
  node_->declare_parameter("polling_rate", 5.0);
  abb::robot::initializeMotionData(motion_data_, robot_controller_description_);

  auto sensor_qos =
      rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth),
                  rmw_qos_profile_sensor_data);
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", sensor_qos);

  system_state_pub_ = node_->create_publisher<abb_robot_msgs::msg::SystemState>("~/system_states", 1);

  if (abb::robot::utilities::verify_state_machine_add_in_presence(robot_controller_description_.system_indicators()))
  {
    runtime_state_pub_ =
        node_->create_publisher<abb_rapid_sm_addin_msgs::msg::RuntimeState>("~/sm_addin/runtime_states", 1);
  }
}

void RWSStatePublisherROS::timer_callback()
{
  try
  {
    rws_manager_.collectAndUpdateRuntimeData(system_state_data_, motion_data_);
  }
  catch (const std::runtime_error& exception)
  {
    auto& clk = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clk, THROTTLE_TIME,
                                "Periodic polling of runtime data via RWS failed with '" << exception.what()
                                                                                         << "' (will try again later)");
  }

  sensor_msgs::msg::JointState joint_state_msg;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        joint_state_msg.name.push_back(joint.name);
        joint_state_msg.position.push_back(joint.state.position);
      }
    }
  }

  abb_robot_msgs::msg::SystemState system_state_msg;
  system_state_msg.motors_on = system_state_data_.motors_on.isTrue();
  system_state_msg.auto_mode = system_state_data_.auto_mode.isTrue();
  system_state_msg.rapid_running = system_state_data_.rapid_running.isTrue();

  for (const auto& task : system_state_data_.rapid_tasks)
  {
    abb_robot_msgs::msg::RAPIDTaskState state{};

    state.name = task.name;
    state.activated = task.is_active;
    state.execution_state = abb::robot::utilities::map(task.execution_state);
    state.motion_task = task.is_motion_task;

    system_state_msg.rapid_tasks.push_back(state);
  }

  for (const auto& unit : system_state_data_.mechanical_units)
  {
    abb_robot_msgs::msg::MechanicalUnitState state{};
    state.name = unit.first;
    state.activated = unit.second.active;
    system_state_msg.mechanical_units.push_back(state);
  }

  abb_rapid_sm_addin_msgs::msg::RuntimeState sm_runtime_state_msg;
  const auto& system_indicators{ robot_controller_description_.system_indicators() };
  if (abb::robot::utilities::verify_state_machine_add_in_presence(system_indicators))
  {
    for (const auto& sm : system_state_data_.state_machines)
    {
      abb_rapid_sm_addin_msgs::msg::StateMachineState state;
      state.rapid_task = sm.rapid_task;
      state.sm_state = abb::robot::utilities::mapStateMachineState(sm.sm_state);
      state.egm_action = abb::robot::utilities::mapStateMachineEGMAction(sm.egm_action);
      sm_runtime_state_msg.state_machines.push_back(state);
    }
  }

  auto time = node_->get_clock()->now();
  joint_state_msg.header.stamp = time;
  joint_state_pub_->publish(joint_state_msg);

  system_state_msg.header.stamp = time;
  system_state_pub_->publish(system_state_msg);

  if (abb::robot::utilities::verify_state_machine_add_in_presence(robot_controller_description_.system_indicators()))
  {
    sm_runtime_state_msg.header.stamp = time;
    runtime_state_pub_->publish(sm_runtime_state_msg);
  }
}
}  // namespace abb_rws_client
