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

#include <abb_hardware_interface/abb_system_position_only.hpp>
#include <abb_hardware_interface/utilities.hpp>

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemPositionOnlyHardware");

CallbackReturn ABBSystemPositionOnlyHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "on_init()");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // TODO(seng):
  // Rename robotstudio_port to egm_port
  // Create new hardware parameter for:
  //  - rws_port (8000)
  //  - rws_ip (localhost)
  auto robotstudio_port = stoi(info_.hardware_parameters["robotstudio_port"]);
  auto rws_port = 8000;
  auto rws_ip = "127.0.0.1";

  // Get robot controller description from RWS
  // TODO(seng): Do not hardcode RWS port or IP
  abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
  auto robot_controller_description_ =
    abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", true);
  RCLCPP_INFO_STREAM(
    LOGGER, "Robot controller description:\n"
      << abb::robot::summaryText(robot_controller_description_));

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(LOGGER, "Expecting exactly 1 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(LOGGER, "Expecting only POSITION command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(LOGGER, "Expecting exactly 1 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(LOGGER, "Expecting only POSITION state interface");
      return CallbackReturn::ERROR;
    }
  }

  // Configure EGM
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

  // Initialize motion data
  try {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
  } catch (...) {
    RCLCPP_ERROR_STREAM(
      LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto & group : robot_controller_description_.mechanical_units_groups()) {
    auto channel_configuration =
      abb::robot::EGMManager::ChannelConfiguration{static_cast<uint16_t>(robotstudio_port), group};
    channel_configurations.emplace_back(channel_configuration);
  }
  try {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  // done
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ABBSystemPositionOnlyHardware::export_state_interfaces()
{
  RCLCPP_INFO(LOGGER, "export_state_interfaces()");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & group : motion_data_.groups) {
    for (auto & unit : group.units) {
      for (auto & joint : unit.joints) {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        auto pos = joint.name.find("joint");
        auto joint_name = joint.name.substr(pos);
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemPositionOnlyHardware::export_command_interfaces()
{
  RCLCPP_INFO(LOGGER, "export_command_interfaces()");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & group : motion_data_.groups) {
    for (auto & unit : group.units) {
      for (auto & joint : unit.joints) {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        auto pos = joint.name.find("joint");
        auto joint_name = joint.name.substr(pos);
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
      }
    }
  }

  return command_interfaces;
}

CallbackReturn ABBSystemPositionOnlyHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "on_activate()");

  size_t counter = 0;
  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && ++counter < NUM_CONNECTION_TRIES) {
    if (egm_manager_->waitForMessage(500)) {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    } else {
      RCLCPP_INFO(LOGGER, "Not Connected to robot...");
      if (counter == NUM_CONNECTION_TRIES) {
        RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
        return CallbackReturn::ERROR;
      }
    }
    rclcpp::sleep_for(500ms);
  }

  RCLCPP_INFO(LOGGER, "CONNECTED!");

  egm_manager_->read(motion_data_);

  RCLCPP_INFO(LOGGER, "System sucessfully started!");

  return CallbackReturn::SUCCESS;
}

return_type ABBSystemPositionOnlyHardware::read()
{
  RCLCPP_INFO(LOGGER, "read()");

  if (!egm_manager_->read(motion_data_)) {
    RCLCPP_INFO(LOGGER, "Failed to read EGM feedback - is EGM session active?");
  } else {
    for (auto & group : motion_data_.groups) {
      for (auto & unit : group.units) {
        for (auto & joint : unit.joints) {
          RCLCPP_INFO(LOGGER, "Got state %.5f for %s!", joint.state.position, joint.name.c_str());
        }
      }
    }
  }
  return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::write()
{
  RCLCPP_INFO(LOGGER, "write()");

  for (auto & group : motion_data_.groups) {
    for (auto & unit : group.units) {
      for (auto & joint : unit.joints) {
        RCLCPP_INFO(LOGGER, "Got command %.5f for %s!", joint.command.position, joint.name.c_str());
      }
    }
  }
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  abb_hardware_interface::ABBSystemPositionOnlyHardware, hardware_interface::SystemInterface)
