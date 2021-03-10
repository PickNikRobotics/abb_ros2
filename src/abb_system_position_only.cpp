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

//#include "ros2_control_demo_driver/robot6dof_demo_system.hpp"
#include "ros2_control_abb_demo_driver/abb_system_position_only.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "abb_libegm/egm_controller_interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


namespace ros2_control_abb_demo_driver
{

return_type ABBSystemPositionOnlyHardware::configure(const hardware_interface::HardwareInfo &info)
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"configure()");

	if(configure_default(info) != return_type::OK){
		return return_type::ERROR;
	}

	hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

	for (const hardware_interface::ComponentInfo &joint : info_.joints){

		if(joint.command_interfaces.size() != 1){
			RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "expecting exactly 1 command interface");
			return return_type::ERROR;
		}

		if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
			RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "expecting only POSITION command interface");
			return return_type::ERROR;
		}

		if(joint.state_interfaces.size() != 1){
			RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "expecting exactly 1 state interface");
			return return_type::ERROR;
		}

		if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION){
			RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "expecting only POSITION state interface");
			return return_type::ERROR;
		}

	}

	// configure EGM
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
			"configuring EGM interface...");

	  egm_interface_ = std::make_unique<abb::egm::EGMControllerInterface>(io_service_, 6511);
	  if(!egm_interface_){
		  RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
				"Could not create EGMControllerInterface");
		  return return_type::ERROR;
	}

	if(!egm_interface_->isInitialized()){
		  RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
				"EGM UDP Server interface failed to initialize (e.g. due to port already bound)");
		  return return_type::ERROR;
	}

	//done
	status_ = hardware_interface::status::CONFIGURED;
	return return_type::OK;
}

std::vector<hardware_interface::StateInterface> ABBSystemPositionOnlyHardware::export_state_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"export_state_interfaces()");

	std::vector<hardware_interface::StateInterface> state_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++){
		state_interfaces.emplace_back(
				hardware_interface::StateInterface(
					info_.joints[i].name,
					hardware_interface::HW_IF_POSITION,
					&hw_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemPositionOnlyHardware::export_command_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"export_command_interfaces()");

	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++){
		command_interfaces.emplace_back(
				hardware_interface::CommandInterface(
					info_.joints[i].name,
					hardware_interface::HW_IF_POSITION,
					&hw_commands_[i]));
	}
	return command_interfaces;
}

return_type ABBSystemPositionOnlyHardware::start()  // QUESTION: should this be in configure?
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"Starting ...please wait...");

	thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

	bool wait=true;
	int num_tries = 100;
	int counter = 0;
	  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Connecting to Robot...");
	while(wait && counter++ < num_tries){
		if(egm_interface_->isConnected()){
			  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Connected to Robot");
			if(egm_interface_->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED){
				  RCLCPP_WARN(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
						"configure RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
			}else{
				wait = egm_interface_->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
			}
		}else{
			  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Not Connected to Robot...");
		}
		rclcpp::sleep_for(500ms);
	}
	if(wait){  // if still not in the right state, exit
		  RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Could NOT Connect to Robot");
		//exit(EXIT_FAILURE); // HACK FOR NOW
		return return_type::ERROR; // TODO: this does not seem to stop anything, and the code continues to read/write
	}
	  RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "CONNECTED!");


	//SET INITIAL STATE TO CURRENT ANGLES, error if something isn't right
	if(!egm_interface_->waitForMessage(500)){ //in [ms]
		RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "NO Message Received");
		//exit(EXIT_FAILURE); // HACK FOR NOW
		return return_type::ERROR; // TODO: this does not seem to stop anything, and the code continues to read/write
	}

	egm_interface_->read(&input_);
	sequence_number_ = input_.header().sequence_number();

	if(sequence_number_ <= 0){
		RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Sequence number < 0");
		//exit(EXIT_FAILURE); // HACK FOR NOW
		return return_type::ERROR; // TODO: this does not seem to stop anything, and the code continues to read/write
	}

	output_.Clear();
	output_pos_.Clear();
	current_positions_.CopyFrom(input_.feedback().robot().joints().position());

	if((size_t)current_positions_.values_size() != hw_states_.size()){
		RCLCPP_FATAL(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
				"Robot does not match expected number of joints");
		//exit(EXIT_FAILURE); // HACK FOR NOW
		return return_type::ERROR; // TODO: this does not seem to stop anything, and the code continues to read/write
	}

	output_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(current_positions_);
	output_pos_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(current_positions_);

	for(size_t i=0; i<hw_states_.size(); i++){
		hw_states_[i] = current_positions_.values(i)/180.0*3.14159;
		hw_commands_[i] = current_positions_.values(i)/180.0*3.14159; //QUESTION: should I do this here?
	}


	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"System Sucessfully started!");

	status_ = hardware_interface::status::STARTED;
	return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::stop()
{

	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"Stopping ...please wait...");

	//EGM TODO: what's left?
	io_service_.stop();
	thread_group_.join_all();


	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"System sucessfully stopped!");

	status_ = hardware_interface::status::STOPPED;
	return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::read()
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"read()");

  	abb::egm::wrapper::Input tmp_input; //TODO: function no longer const, change this
    abb::egm::wrapper::Joints tmp_current_positions;
	egm_interface_->read(&tmp_input);
    tmp_current_positions.CopyFrom(tmp_input.feedback().robot().joints().position());

  	for (size_t i = 0; i < hw_states_.size(); i++) {
    	hw_states_[i] = tmp_current_positions.values(i)/180.0*3.14159;
    	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",hw_states_[i], i);
  	}

	return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::write()
{
	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"),
	"write()");


  	for (size_t i = 0; i < hw_commands_.size(); i++) {
    	RCLCPP_INFO(rclcpp::get_logger("ABBSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",hw_commands_[i], i);
		output_pos_.mutable_robot()->mutable_joints()->mutable_position()->set_values(i, (hw_commands_[i]*180.0/3.14159));
  	}
	egm_interface_->write(output_pos_);

	return return_type::OK;
}



} //namespace ros2_control_abb_demo_driver


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	ros2_control_abb_demo_driver::ABBSystemPositionOnlyHardware,
	hardware_interface::SystemInterface
)

