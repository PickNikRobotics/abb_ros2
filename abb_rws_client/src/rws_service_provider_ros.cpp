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

#include "abb_rws_client/rws_service_provider_ros.hpp"

#include <abb_robot_msgs/msg/service_responses.hpp>

#include "abb_rws_client/mapping.hpp"
#include "abb_rws_client/utilities.hpp"

using RAPIDSymbols = abb::rws::RWSStateMachineInterface::ResourceIdentifiers::RAPID::Symbols;

namespace abb_rws_client
{
RWSServiceProviderROS::RWSServiceProviderROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip,
                                             unsigned short robot_port)
  : RWSClient(node, robot_ip, robot_port)
{
  system_state_sub_ = node_->create_subscription<abb_robot_msgs::msg::SystemState>(
      "system_states", 10, std::bind(&RWSServiceProviderROS::systemStateCallback, this, std::placeholders::_1));
  runtime_state_sub_ = node_->create_subscription<abb_rapid_sm_addin_msgs::msg::RuntimeState>(
      "sm_addin/runtime_states", 10,
      std::bind(&RWSServiceProviderROS::runtimeStateCallback, this, std::placeholders::_1));

  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRobotControllerDescription>(
      "~/get_robot_controller_description",
      std::bind(&RWSServiceProviderROS::getRCDescription, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetFileContents>(
      "~/get_file_contents",
      std::bind(&RWSServiceProviderROS::getFileContents, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetIOSignal>(
      "~/get_io_signal",
      std::bind(&RWSServiceProviderROS::getIOSignal, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDBool>(
      "~/get_rapid_bool",
      std::bind(&RWSServiceProviderROS::getRapidBool, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDDnum>(
      "~/get_rapid_dnum",
      std::bind(&RWSServiceProviderROS::getRapidDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDNum>(
      "~/get_rapid_num",
      std::bind(&RWSServiceProviderROS::getRapidNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDString>(
      "~/get_rapid_string",
      std::bind(&RWSServiceProviderROS::getRapidString, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDSymbol>(
      "~/get_rapid_symbol",
      std::bind(&RWSServiceProviderROS::getRapidSymbol, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetSpeedRatio>(
      "~/get_speed_ratio",
      std::bind(&RWSServiceProviderROS::getSpeedRatio, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/pp_to_main", std::bind(&RWSServiceProviderROS::ppToMain, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetFileContents>(
      "~/set_file_contents",
      std::bind(&RWSServiceProviderROS::setFileContents, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetIOSignal>(
      "~/set_io_signal",
      std::bind(&RWSServiceProviderROS::setIOSignal, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/set_motors_off",
      std::bind(&RWSServiceProviderROS::setMotorsOff, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/set_motors_on",
      std::bind(&RWSServiceProviderROS::setMotorsOn, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDBool>(
      "~/set_rapid_bool",
      std::bind(&RWSServiceProviderROS::setRapidBool, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDDnum>(
      "~/set_rapid_dnum",
      std::bind(&RWSServiceProviderROS::setRapidDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDNum>(
      "~/set_rapid_num",
      std::bind(&RWSServiceProviderROS::setRapidNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDString>(
      "~/set_rapid_string",
      std::bind(&RWSServiceProviderROS::setRapidString, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDSymbol>(
      "~/set_rapid_symbol",
      std::bind(&RWSServiceProviderROS::setRapidSymbol, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetSpeedRatio>(
      "~/set_speed_ratio",
      std::bind(&RWSServiceProviderROS::setSpeedRatio, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/start_rapid",
      std::bind(&RWSServiceProviderROS::startRapid, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/stop_rapid", std::bind(&RWSServiceProviderROS::stopRapid, this, std::placeholders::_1, std::placeholders::_2)));

  const auto& system_indicators = robot_controller_description_.system_indicators();

  auto has_sm_1_0 = system_indicators.addins().has_state_machine_1_0();
  auto has_sm_1_1 = system_indicators.addins().has_state_machine_1_1();
  if (has_sm_1_0)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "StateMachine Add-In 1.0 detected");
  }
  else if (has_sm_1_1)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "StateMachine Add-In 1.1 detected");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "No StateMachine Add-In detected");
  }

  if (has_sm_1_0 || has_sm_1_1)
  {
    sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
        "~/run_rapid_routine",
        std::bind(&RWSServiceProviderROS::runRapidRoutine, this, std::placeholders::_1, std::placeholders::_2)));
    sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine>(
        "~/set_rapid_routine",
        std::bind(&RWSServiceProviderROS::setRapidRoutine, this, std::placeholders::_1, std::placeholders::_2)));
    if (system_indicators.options().egm())
    {
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::GetEGMSettings>(
          "get_egm_settings",
          std::bind(&RWSServiceProviderROS::getEGMSettings, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetEGMSettings>(
          "set_egm_settings",
          std::bind(&RWSServiceProviderROS::setEGMSettings, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "start_egm_joint",
          std::bind(&RWSServiceProviderROS::startEGMJoint, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "start_egm_pose",
          std::bind(&RWSServiceProviderROS::startEGMPose, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "stop_egm", std::bind(&RWSServiceProviderROS::stopEGM, this, std::placeholders::_1, std::placeholders::_2)));
      if (has_sm_1_1)
      {
        sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
            "start_egm_stream",
            std::bind(&RWSServiceProviderROS::startEGMStream, this, std::placeholders::_1, std::placeholders::_2)));
        sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
            "stop_egm_stream",
            std::bind(&RWSServiceProviderROS::stopEGMStream, this, std::placeholders::_1, std::placeholders::_2)));
      }
    }

    if (system_indicators.addins().smart_gripper())
    {
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "run_sg_routine",
          std::bind(&RWSServiceProviderROS::runSGRoutine, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetSGCommand>(
          "set_sg_command",
          std::bind(&RWSServiceProviderROS::setSGCOmmand, this, std::placeholders::_1, std::placeholders::_2)));
    }
  }
  RCLCPP_INFO(node_->get_logger(), "RWS client services initialized!");
}

void RWSServiceProviderROS::systemStateCallback(const abb_robot_msgs::msg::SystemState& msg)
{
  system_state_ = msg;
}

void RWSServiceProviderROS::runtimeStateCallback(const abb_rapid_sm_addin_msgs::msg::RuntimeState& msg)
{
  runtime_state_ = msg;
}

bool RWSServiceProviderROS::getRCDescription(
    const abb_robot_msgs::srv::GetRobotControllerDescription::Request::SharedPtr req,
    abb_robot_msgs::srv::GetRobotControllerDescription::Response::SharedPtr res)
{
  res->description = robot_controller_description_.DebugString();

  res->message = abb_robot_msgs::msg::ServiceResponses::SUCCESS;
  res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;

  return true;
}

bool RWSServiceProviderROS::getFileContents(const abb_robot_msgs::srv::GetFileContents::Request::SharedPtr req,
                                            abb_robot_msgs::srv::GetFileContents::Response::SharedPtr res)
{
  if (!verify_argument_filename(req->filename, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.getFile(abb::rws::RWSClient::FileResource(req->filename), &res->contents))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getIOSignal(const abb_robot_msgs::srv::GetIOSignal::Request::SharedPtr req,
                                        abb_robot_msgs::srv::GetIOSignal::Response::SharedPtr res)
{
  if (!verify_argument_signal(req->signal, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    res->value = interface.getIOSignal(req->signal);

    if (!res->value.empty())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRapidBool(const abb_robot_msgs::srv::GetRAPIDBool::Request::SharedPtr req,
                                         abb_robot_msgs::srv::GetRAPIDBool::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDBool rapid_bool;
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_bool))
    {
      res->value = rapid_bool.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRapidDNum(const abb_robot_msgs::srv::GetRAPIDDnum::Request::SharedPtr req,
                                         abb_robot_msgs::srv::GetRAPIDDnum::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDDnum rapid_dnum;
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_dnum))
    {
      res->value = rapid_dnum.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRapidNum(const abb_robot_msgs::srv::GetRAPIDNum::Request::SharedPtr req,
                                        abb_robot_msgs::srv::GetRAPIDNum::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum rapid_num{};
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_num))
    {
      res->value = rapid_num.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRapidString(const abb_robot_msgs::srv::GetRAPIDString::Request::SharedPtr req,
                                           abb_robot_msgs::srv::GetRAPIDString::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDString rapid_string{};
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_string))
    {
      res->value = rapid_string.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRapidSymbol(const abb_robot_msgs::srv::GetRAPIDSymbol::Request::SharedPtr req,
                                           abb_robot_msgs::srv::GetRAPIDSymbol::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    res->value = interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol);

    if (!res->value.empty())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getSpeedRatio(const abb_robot_msgs::srv::GetSpeedRatio::Request::SharedPtr,
                                          abb_robot_msgs::srv::GetSpeedRatio::Response::SharedPtr res)
{
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    try
    {
      res->speed_ratio = interface.getSpeedRatio();
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    catch (const std::runtime_error& exception)
    {
      res->message = exception.what();
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::ppToMain(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_stopped(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.resetRAPIDProgramPointer())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}
bool RWSServiceProviderROS::runRapidRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                            abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().rapid().signalRunRAPIDRoutine())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::runSGRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().sg().signalRunSGRoutine())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setFileContents(const abb_robot_msgs::srv::SetFileContents::Request::SharedPtr req,
                                            abb_robot_msgs::srv::SetFileContents::Response::SharedPtr res)
{
  if (!verify_argument_filename(req->filename, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.uploadFile(abb::rws::RWSClient::FileResource(req->filename), req->contents))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setIOSignal(const abb_robot_msgs::srv::SetIOSignal::Request::SharedPtr req,
                                        abb_robot_msgs::srv::SetIOSignal::Response::SharedPtr res)
{
  if (!verify_argument_signal(req->signal, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setIOSignal(req->signal, req->value))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setMotorsOff(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_motors_on(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setMotorsOff())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setMotorsOn(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                        abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_motors_off(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setMotorsOn())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidBool(const abb_robot_msgs::srv::SetRAPIDBool::Request::SharedPtr req,
                                         abb_robot_msgs::srv::SetRAPIDBool::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDBool rapid_bool = static_cast<bool>(req->value);
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_bool))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidDNum(const abb_robot_msgs::srv::SetRAPIDDnum::Request::SharedPtr req,
                                         abb_robot_msgs::srv::SetRAPIDDnum::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDDnum rapid_dnum = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_dnum))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidNum(const abb_robot_msgs::srv::SetRAPIDNum::Request::SharedPtr req,
                                        abb_robot_msgs::srv::SetRAPIDNum::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum rapid_num = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_num))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidString(const abb_robot_msgs::srv::SetRAPIDString::Request::SharedPtr req,
                                           abb_robot_msgs::srv::SetRAPIDString::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDString rapid_string = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_string))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidSymbol(const abb_robot_msgs::srv::SetRAPIDSymbol::Request::SharedPtr req,
                                           abb_robot_msgs::srv::SetRAPIDSymbol::Response::SharedPtr res)
{
  if (!verify_argument_rapid_symbol_path(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, req->value))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setSpeedRatio(const abb_robot_msgs::srv::SetSpeedRatio::Request::SharedPtr req,
                                          abb_robot_msgs::srv::SetSpeedRatio::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    try
    {
      if (interface.setSpeedRatio(req->speed_ratio))
      {
        res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
      }
      else
      {
        res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
        res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      }
    }
    catch (const std::exception& exception)
    {
      res->message = exception.what();
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startRapid(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_motors_on(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.startRAPIDExecution())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopRapid(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.stopRAPIDExecution())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getEGMSettings(const abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Request::SharedPtr req,
                                           abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Response::SharedPtr res)
{
  if (!verify_argument_rapid_task(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_exist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RWSStateMachineInterface::EGMSettings settings;

    if (interface.services().egm().getSettings(req->task, &settings))
    {
      res->settings = abb::robot::utilities::map(settings);
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setEGMSettings(const abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Request::SharedPtr req,
                                           abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Response::SharedPtr res)
{
  if (!verify_argument_rapid_task(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_exist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_initialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RWSStateMachineInterface::EGMSettings settings = abb::robot::utilities::map(req->settings);

    if (interface.services().egm().setSettings(req->task, settings))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRapidRoutine(const abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Request::SharedPtr req,
                                            abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Response::SharedPtr res)
{
  if (!verify_argument_rapid_task(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_exist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_initialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().rapid().setRoutineName(req->task, req->routine))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setSGCOmmand(const abb_rapid_sm_addin_msgs::srv::SetSGCommand::Request::SharedPtr req,
                                         abb_rapid_sm_addin_msgs::srv::SetSGCommand::Response::SharedPtr res)
{
  if (!verify_argument_rapid_task(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_exist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_task_initialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  unsigned int req_command = 0;
  try
  {
    req_command = abb::robot::utilities::mapStateMachineSGCommand(req->command);
  }
  catch (const std::runtime_error& exception)
  {
    res->message = exception.what();
    res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum sg_command_input = static_cast<float>(req_command);
    abb::rws::RAPIDNum sg_target_position_input = req->target_position;

    if (interface.setRAPIDSymbolData(req->task, RAPIDSymbols::SG_COMMAND_INPUT, sg_command_input) &&
        interface.setRAPIDSymbolData(req->task, RAPIDSymbols::SG_TARGET_POSTION_INPUT, sg_target_position_input))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMJoint(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                          abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartJoint())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMPose(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartPose())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                           abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_sm_addin_runtime_states(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartStream())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopEGM(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStop())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                          abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verify_auto_mode(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rapid_running(res->result_code, res->message))
  {
    return true;
  }
  if (!verify_rws_manager_ready(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStopStream())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::verifyAutoMode(uint16_t& result_code, std::string& message)
{
  if (!system_state_.auto_mode)
  {
    message = abb_robot_msgs::msg::ServiceResponses::NOT_IN_AUTO_MODE;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_NOT_IN_AUTO_MODE;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentFilename(const std::string& filename, uint16_t& result_code,
                                                   std::string& message)
{
  if (filename.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_FILENAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_FILENAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentRapidSymbolPath(const abb_robot_msgs::msg::RAPIDSymbolPath& path,
                                                          uint16_t& result_code, std::string& message)
{
  if (path.task.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  if (path.module.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_MODULE_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_MODULE_NAME;
    return false;
  }

  if (path.symbol.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_SYMBOL_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_SYMBOL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentRapidTask(const std::string& task, uint16_t& result_code,
                                                    std::string& message)
{
  if (task.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message)
{
  if (signal.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_SIGNAL_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_SIGNAL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyMotorsOff(uint16_t& result_code, std::string& message)
{
  if (system_state_.motors_on)
  {
    message = abb_robot_msgs::msg::ServiceResponses::MOTORS_ARE_ON;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_MOTORS_ARE_ON;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyMotorsOn(uint16_t& result_code, std::string& message)
{
  if (!system_state_.motors_on)
  {
    message = abb_robot_msgs::msg::ServiceResponses::MOTORS_ARE_OFF;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_MOTORS_ARE_OFF;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddingRuntimeStates(uint16_t& result_code, std::string& message)
{
  if (runtime_state_.state_machines.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_RUNTIME_STATES_MISSING;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_RUNTIME_STATES_MISSING;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddingTaskExist(const std::string& task, uint16_t& result_code,
                                                    std::string& message)
{
  auto it = std::find_if(runtime_state_.state_machines.begin(), runtime_state_.state_machines.end(),
                         [&](const auto& sm) { return sm.rapid_task == task; });

  if (it == runtime_state_.state_machines.end())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_UNKNOWN_RAPID_TASK;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_UNKNOWN_RAPID_TASK;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddingTaskInitialized(const std::string& task, uint16_t& result_code,
                                                          std::string& message)
{
  if (!verifySMAddingTaskExist(task, result_code, message))
    return false;

  auto it = std::find_if(runtime_state_.state_machines.begin(), runtime_state_.state_machines.end(),
                         [&](const auto& sm) { return sm.rapid_task == task; });

  if (it->sm_state == abb_rapid_sm_addin_msgs::msg::StateMachineState::SM_STATE_UNKNOWN ||
      it->sm_state == abb_rapid_sm_addin_msgs::msg::StateMachineState::SM_STATE_INITIALIZE)
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_UNINITIALIZED;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_UNINITIALIZED;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRapidRunning(uint16_t& result_code, std::string& message)
{
  if (!system_state_.rapid_running)
  {
    message = abb_robot_msgs::msg::ServiceResponses::RAPID_NOT_RUNNING;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_RAPID_NOT_RUNNING;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRapidStopped(uint16_t& result_code, std::string& message)
{
  if (system_state_.rapid_running)
  {
    message = abb_robot_msgs::msg::ServiceResponses::RAPID_NOT_STOPPED;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_RAPID_NOT_STOPPED;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRWSManagerReady(uint16_t& result_code, std::string& message)
{
  if (!rws_manager_.isInterfaceReady())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SERVER_IS_BUSY;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SERVER_IS_BUSY;
    return false;
  }

  return true;
}

}  // namespace abb_rws_client
