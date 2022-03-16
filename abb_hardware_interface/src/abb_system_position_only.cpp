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

#include "abb_hardware_interface/abb_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "abb_egm_rws_managers/egm_manager.h"
#include "abb_egm_rws_managers/rws_manager.h"
#include "abb_libegm/egm_controller_interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_abb_driver/utilities.hpp"

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static constexpr auto DEG_TO_RAD = 180 / M_PI;

CallbackReturn ABBSystemPositionOnlyHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "on_init()");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  auto robotstudio_port = stoi(info_.hardware_parameters["robotstudio_port"]);
  auto mech_unit_group_name = info_.hardware_parameters["mech_unit_group"];

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
        "expecting exactly 1 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
        "expecting only POSITION command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "expecting exactly 1 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
        "expecting only POSITION state interface");
      return CallbackReturn::ERROR;
    }
  }

  // Configure EGM
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Configuring EGM interface...");

  // Get robot controller description from RWS
  abb::robot::RWSManager rws_manager("127.0.0.1", 8000, "Default User", "robotics");
  auto robot_controller_description = abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", true);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Robot controller description:\n" << abb::robot::summaryText(robot_controller_description));

  // Initialize motion data
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  auto mech_unit_group = abb::robot::findMechanicalUnitGroup("", robot_controller_description);
  auto channel_configuration = abb::robot::EGMManager::ChannelConfiguration{static_cast<uint16_t>(robotstudio_port), mech_unit_group};
  auto channel_configurations = std::vector<abb::robot::EGMManager::ChannelConfiguration>{channel_configuration};
  egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  if (!egm_manager_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
      "Could not create EGMManager");
    return CallbackReturn::ERROR;
  }

  // done
  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "on_configure()");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ABBSystemPositionOnlyHardware::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "export_state_interfaces()");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ABBSystemPositionOnlyHardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "export_command_interfaces()");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn ABBSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "on_activate()");

  thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

  bool wait = true;
  size_t counter = 0;
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Connecting to Robot...");
  while (rclcpp::ok() && wait && ++counter < NUM_CONNECTION_TRIES) {
    if (!egm_manager_->waitForMessage(500)) {
      RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Connected to Robot");
      break;
    // if (egm_interface_->isConnected()) {
    //   RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Connected to Robot");
    //   if (
    //     egm_interface_->getStatus().rapid_execution_state() ==
    //     abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
    //   {
    //     RCLCPP_WARN(
    //       rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
    //       "configure RAPID execution state is UNDEFINED (might happen first time after controller "
    //       "start/restart). Try to restart the RAPID program.");
    //   } else {
    //     wait = egm_interface_->getStatus().rapid_execution_state() !=
    //       abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
    //   }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Not Connected to Robot...");
      if (counter == NUM_CONNECTION_TRIES){
        RCLCPP_ERROR(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Failed to connect to robot");
        return CallbackReturn::ERROR;
      }
    }
    rclcpp::sleep_for(500ms);
  }

  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "CONNECTED!");

  // SET INITIAL STATE TO CURRENT ANGLES, error if something isn't right
  // if (!egm_interface_->waitForMessage(500)) {  // in [ms]
  //   RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "NO Message Received");
  //   return CallbackReturn::ERROR;
  // }

  egm_manager_->read(motion_data_);
  
  // sequence_number_ = input_.header().sequence_number();

  // if (sequence_number_ <= 0) {
  //   RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Sequence number < 0");
  //   return CallbackReturn::ERROR;
  // }

  // output_.Clear();
  // output_pos_.Clear();
  // current_positions_.CopyFrom(input_.feedback().robot().joints().position());
  // current_positions_.JointState = input_.JointState;

  // if (static_cast<size_t>(current_positions_.values_size()) != hw_states_.size()) {
  //   RCLCPP_FATAL(
  //     rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
  //     "Robot does not match expected number of joints");
  //   return CallbackReturn::ERROR;
  // }

  // output_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(current_positions_);
  // output_pos_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(current_positions_);
  for (auto &group : motion_data_.groups)
  {
    for (auto &unit : group.units)
    {
      for (size_t i = 0; i < hw_states_.size(); ++i)
      {
        auto joint = unit.joints[i];
        hw_states_[i] = joint.state.position / DEG_TO_RAD;
        hw_commands_[i] = hw_states_[i];
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "System Sucessfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "on_deactivate()");

  return CallbackReturn::SUCCESS;
}

return_type ABBSystemPositionOnlyHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "read()");

  egm_manager_->read(motion_data_);
  // temp_current_positions.CopyFrom(temp_input.feedback().robot().joints().position());
  for (auto &group : motion_data_.groups)
  {
    for (auto &unit : group.units)
    {
      for (size_t i = 0; i < hw_states_.size(); ++i)
      {
        auto& joint = unit.joints[i];
        hw_states_[i] = joint.state.position;
        RCLCPP_INFO(
          rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got state %.5f for joint %ld!",
          hw_states_[i], i);
      }
    }
  }
  // for (size_t i = 0; i < hw_states_.size(); ++i) {
  //   hw_states_[i] = temp_current_positions.values(i) / DEG_TO_RAD;
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got state %.5f for joint %ld!",
  //     hw_states_[i], i);
  // }

  return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "write()");

  for (auto &group : motion_data_.groups)
  {
    for (auto &unit : group.units)
    {
      for (size_t i = 0; i < hw_commands_.size(); ++i)
      {
        auto& joint = unit.joints[i];
        joint.command.position = hw_commands_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got command %.5f for joint %ld!",
          joint.command.position, i);
      }
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",
          motion_data_.groups[0].units[0].joints[0].command.position, 0);
  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got command %.5f for joint %ld!",
  //     hw_commands_[i], i);
  //   output_pos_.mutable_robot()->mutable_joints()->mutable_position()->set_values(
  //     i, (hw_commands_[i] * 180.0 / 3.14159));
  // }
  egm_manager_->write(motion_data_);

  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  abb_hardware_interface::ABBSystemPositionOnlyHardware, hardware_interface::SystemInterface)
