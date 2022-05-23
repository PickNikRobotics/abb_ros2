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
//
// This file is a modified copy from
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/include/abb_rws_service_provider/
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/include/abb_rws_state_publisher/

#pragma once

#include "abb_rws_client/rws_client.hpp"

// SYSMTEM
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

// ABB MSG
#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

// ABB SRV
#include <abb_rapid_sm_addin_msgs/srv/get_egm_settings.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_egm_settings.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_rapid_routine.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_sg_command.hpp>
#include <abb_robot_msgs/srv/get_file_contents.hpp>
#include <abb_robot_msgs/srv/get_io_signal.hpp>
#include <abb_robot_msgs/srv/get_rapid_bool.hpp>
#include <abb_robot_msgs/srv/get_rapid_dnum.hpp>
#include <abb_robot_msgs/srv/get_rapid_num.hpp>
#include <abb_robot_msgs/srv/get_rapid_string.hpp>
#include <abb_robot_msgs/srv/get_rapid_symbol.hpp>
#include <abb_robot_msgs/srv/get_robot_controller_description.hpp>
#include <abb_robot_msgs/srv/get_speed_ratio.hpp>
#include <abb_robot_msgs/srv/set_file_contents.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <abb_robot_msgs/srv/set_rapid_bool.hpp>
#include <abb_robot_msgs/srv/set_rapid_dnum.hpp>
#include <abb_robot_msgs/srv/set_rapid_num.hpp>
#include <abb_robot_msgs/srv/set_rapid_string.hpp>
#include <abb_robot_msgs/srv/set_rapid_symbol.hpp>
#include <abb_robot_msgs/srv/set_speed_ratio.hpp>
#include <abb_robot_msgs/srv/trigger_with_result_code.hpp>

namespace abb_rws_client {

class RWSServiceProviderROS : RWSClient {
 public:
  RWSServiceProviderROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port);

 private:
  void system_state_callback(const abb_robot_msgs::msg::SystemState& msg);

  void runtime_state_callback(const abb_rapid_sm_addin_msgs::msg::RuntimeState& msg);

  bool get_file_contents(const abb_robot_msgs::srv::GetFileContents::Request::SharedPtr req,
                         abb_robot_msgs::srv::GetFileContents::Response::SharedPtr res);

  bool get_io_signal(const abb_robot_msgs::srv::GetIOSignal::Request::SharedPtr req,
                     abb_robot_msgs::srv::GetIOSignal::Response::SharedPtr res);

  bool get_rapid_bool(const abb_robot_msgs::srv::GetRAPIDBool::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDBool::Response::SharedPtr res);

  bool get_rapid_dnum(const abb_robot_msgs::srv::GetRAPIDDnum::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDDnum::Response::SharedPtr res);

  bool get_rapid_num(const abb_robot_msgs::srv::GetRAPIDNum::Request::SharedPtr req,
                     abb_robot_msgs::srv::GetRAPIDNum::Response::SharedPtr res);

  bool get_rapid_string(const abb_robot_msgs::srv::GetRAPIDString::Request::SharedPtr req,
                        abb_robot_msgs::srv::GetRAPIDString::Response::SharedPtr res);

  bool get_rapid_symbol(const abb_robot_msgs::srv::GetRAPIDSymbol::Request::SharedPtr req,
                        abb_robot_msgs::srv::GetRAPIDSymbol::Response::SharedPtr res);

  bool get_rc_description(const abb_robot_msgs::srv::GetRobotControllerDescription::Request::SharedPtr req,
                          abb_robot_msgs::srv::GetRobotControllerDescription::Response::SharedPtr res);

  bool get_speed_ratio(const abb_robot_msgs::srv::GetSpeedRatio::Request::SharedPtr req,
                       abb_robot_msgs::srv::GetSpeedRatio::Response::SharedPtr res);

  bool pp_to_main(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                  abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool run_rapid_routine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool run_sg_routine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool set_file_contents(const abb_robot_msgs::srv::SetFileContents::Request::SharedPtr req,
                         abb_robot_msgs::srv::SetFileContents::Response::SharedPtr res);

  bool set_io_signal(const abb_robot_msgs::srv::SetIOSignal::Request::SharedPtr req,
                     abb_robot_msgs::srv::SetIOSignal::Response::SharedPtr res);

  bool set_motors_off(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool set_motors_on(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool set_rapid_bool(const abb_robot_msgs::srv::SetRAPIDBool::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDBool::Response::SharedPtr res);

  bool set_rapid_dnum(const abb_robot_msgs::srv::SetRAPIDDnum::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDDnum::Response::SharedPtr res);

  bool set_rapid_num(const abb_robot_msgs::srv::SetRAPIDNum::Request::SharedPtr req,
                     abb_robot_msgs::srv::SetRAPIDNum::Response::SharedPtr res);

  bool set_rapid_string(const abb_robot_msgs::srv::SetRAPIDString::Request::SharedPtr req,
                        abb_robot_msgs::srv::SetRAPIDString::Response::SharedPtr res);

  bool set_rapid_symbol(const abb_robot_msgs::srv::SetRAPIDSymbol::Request::SharedPtr req,
                        abb_robot_msgs::srv::SetRAPIDSymbol::Response::SharedPtr res);

  bool set_speed_ratio(const abb_robot_msgs::srv::SetSpeedRatio::Request::SharedPtr req,
                       abb_robot_msgs::srv::SetSpeedRatio::Response::SharedPtr res);

  bool start_rapid(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                   abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool stop_rapid(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                  abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool get_egm_settings(const abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Request::SharedPtr req,
                        abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Response::SharedPtr res);

  bool set_egm_settings(const abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Request::SharedPtr req,
                        abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Response::SharedPtr res);

  bool set_rapid_routine(const abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Request::SharedPtr req,
                         abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Response::SharedPtr res);

  bool set_sg_command(const abb_rapid_sm_addin_msgs::srv::SetSGCommand::Request::SharedPtr req,
                      abb_rapid_sm_addin_msgs::srv::SetSGCommand::Response::SharedPtr res);

  bool start_egm_joint(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool start_egm_pose(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool start_egm_stream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                        abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool stop_egm(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool stop_egm_stream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  bool verify_auto_mode(uint16_t& result_code, std::string& message);

  bool verify_argument_filename(const std::string& filename, uint16_t& result_code, std::string& message);

  bool verify_argument_rapid_symbol_path(const abb_robot_msgs::msg::RAPIDSymbolPath& path, uint16_t& result_code,
                                         std::string& message);

  bool verify_argument_rapid_task(const std::string& task, uint16_t& result_code, std::string& message);

  bool verify_argument_signal(const std::string& signal, uint16_t& result_code, std::string& message);

  bool verify_motors_off(uint16_t& result_code, std::string& message);

  bool verify_motors_on(uint16_t& result_code, std::string& message);

  bool verify_sm_addin_runtime_states(uint16_t& result_code, std::string& message);

  bool verify_sm_addin_task_exist(const std::string& task, uint16_t& result_code, std::string& message);

  bool verify_sm_addin_task_initialized(const std::string& task, uint16_t& result_code, std::string& message);

  bool verify_rapid_running(uint16_t& result_code, std::string& message);

  bool verify_rapid_stopped(uint16_t& result_code, std::string& message);

  bool verify_rws_manager_ready(uint16_t& result_code, std::string& message);

  rclcpp::Subscription<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_sub_;

  std::vector<rclcpp::ServiceBase::WeakPtr> core_services_;
  std::vector<rclcpp::ServiceBase::WeakPtr> sm_services_;

  abb_robot_msgs::msg::SystemState system_state_;
  abb_rapid_sm_addin_msgs::msg::RuntimeState runtime_state_;
};

}  // namespace abb_rws_client
