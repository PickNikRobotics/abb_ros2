// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_ABB_DRIVER__ABB_SYSTEM_POSITION_ONLY_HPP_
#define ROS2_CONTROL_ABB_DRIVER__ABB_SYSTEM_POSITION_ONLY_HPP_

#include <abb_libegm/egm_controller_interface.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_abb_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_control_abb_driver
{
class ABBSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemPositionOnlyHardware)

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type read() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type write() override;

private:
  // Hardware parameters
  double robotstudio_port_;
  // Store the command for the simulated robot
  std::vector<double> hw_commands_, hw_states_;

  // EGM
  int sequence_number_ = 0;
  boost::asio::io_service io_service_;
  boost::thread_group thread_group_;
  std::unique_ptr<abb::egm::EGMControllerInterface> egm_interface_;

  abb::egm::wrapper::Input input_;
  abb::egm::wrapper::Output output_;
  abb::egm::wrapper::Output output_pos_;
  abb::egm::wrapper::Output output_vel_;
  abb::egm::wrapper::Joints current_positions_;
  abb::egm::wrapper::Joints current_velocities_;
  abb::egm::wrapper::Joints initial_positions_;
};

}  // namespace ros2_control_abb_driver

#endif  // ROS2_CONTROL_ABB_DRIVER__ABB_SYSTEM_POSITION_ONLY_HPP_
