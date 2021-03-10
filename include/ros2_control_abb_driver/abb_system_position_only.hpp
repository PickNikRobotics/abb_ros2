#ifndef ROS2_CONTROL_ABB_DRIVER__ABB_SYSTEM_POSITION_HPP_
#define ROS2_CONTROL_ABB_DRIVER__ABB_SYSTEM_POSITION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "ros2_control_abb_driver/visibility_control.h"

#include <abb_libegm/egm_controller_interface.h>

using hardware_interface::return_type;

namespace ros2_control_abb_driver
{

class ABBSystemPositionOnlyHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemPositionOnlyHardware);

	ROS2_CONTROL_DRIVER_PUBLIC
	return_type configure(const hardware_interface::HardwareInfo & info) override;

	ROS2_CONTROL_DRIVER_PUBLIC
	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

	ROS2_CONTROL_DRIVER_PUBLIC
	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	ROS2_CONTROL_DRIVER_PUBLIC
	return_type start() override;

	ROS2_CONTROL_DRIVER_PUBLIC
	return_type stop() override;

	ROS2_CONTROL_DRIVER_PUBLIC
	return_type read() override;

	ROS2_CONTROL_DRIVER_PUBLIC
	return_type write() override;

private:
	// Dummy parameters
	double hw_start_sec_, hw_stop_sec_, hw_slowdown_;
	// Store the command for the simulated robot
	std::vector<double> hw_commands_, hw_states_;


	// EGM things
	const int egm_rate_ = 250.0;
	int sequence_number_ = 0;
	double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).
	double last_time = 0.0;
	bool ready_to_write_ = false;

	//EGM
	boost::asio::io_service io_service_;
	boost::thread_group thread_group_; //NOTE
	std::unique_ptr<abb::egm::EGMControllerInterface> egm_interface_;

	abb::egm::wrapper::Input input_;
	abb::egm::wrapper::Output output_;
	abb::egm::wrapper::Output output_pos_;
	abb::egm::wrapper::Output output_vel_;
	abb::egm::wrapper::Joints current_positions_;
	abb::egm::wrapper::Joints current_velocities_;
	abb::egm::wrapper::Joints initial_positions_;
};

}  // namespace ros2_control_abb_hardware

#endif  // ROS2_CONTROL_DRIVER__RRBOT_SYSTEM_POSITION_ONLY_HPP_
