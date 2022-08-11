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

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

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

namespace abb_rws_client
{
class RWSServiceProviderROS
{
public:
  /**
   * \brief Creates a services server.
   *
   * \param node ROS2 node.
   * \param robot_ip IP address for the robot controller's RWS server.
   * \param robot_poty Port number for the robot controller's RWS server.
   */
  RWSServiceProviderROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port);

  RWSServiceProviderROS() = delete;

  virtual ~RWSServiceProviderROS() = default;

private:
  /**
   * \brief Callback for robot controller system state messages.
   *
   * \param message to process.
   */
  void systemStateCallback(const abb_robot_msgs::msg::SystemState::SharedPtr msg);

  /**
   * \brief Callback for RobotWare StateMachine Add-In runtime state messages.
   *
   * Note: Only used if the Add-In is present in the robot controller's system.
   *
   * \param message to process.
   */
  void runtimeStateCallback(const abb_rapid_sm_addin_msgs::msg::RuntimeState::SharedPtr msg);

  /**
   * \brief Gets the contents of a file.
   *
   * The file must be located in the robot controller's home directory.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getFileContents(const abb_robot_msgs::srv::GetFileContents::Request::SharedPtr req,
                       abb_robot_msgs::srv::GetFileContents::Response::SharedPtr res);
  /**
   * \brief Gets an IO-signal.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getIOSignal(const abb_robot_msgs::srv::GetIOSignal::Request::SharedPtr req,
                   abb_robot_msgs::srv::GetIOSignal::Response::SharedPtr res);

  /**
   * \brief Gets a RAPID 'bool' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDBool(const abb_robot_msgs::srv::GetRAPIDBool::Request::SharedPtr req,
                    abb_robot_msgs::srv::GetRAPIDBool::Response::SharedPtr res);

  /**
   * \brief Gets a RAPID 'dnum' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDDNum(const abb_robot_msgs::srv::GetRAPIDDnum::Request::SharedPtr req,
                    abb_robot_msgs::srv::GetRAPIDDnum::Response::SharedPtr res);

  /**
   * \brief Gets a RAPID 'num' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDNum(const abb_robot_msgs::srv::GetRAPIDNum::Request::SharedPtr req,
                   abb_robot_msgs::srv::GetRAPIDNum::Response::SharedPtr res);

  /**
   * \brief Gets a RAPID 'string' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDString(const abb_robot_msgs::srv::GetRAPIDString::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDString::Response::SharedPtr res);

  /**
   * \brief Gets a RAPID symbol (in raw text format).
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDSymbol(const abb_robot_msgs::srv::GetRAPIDSymbol::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDSymbol::Response::SharedPtr res);

  /**
   * \brief Gets a description of the connected robot controller.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRCDescription(const abb_robot_msgs::srv::GetRobotControllerDescription::Request::SharedPtr req,
                        abb_robot_msgs::srv::GetRobotControllerDescription::Response::SharedPtr res);

  /**
   * \brief Gets the controller speed ratio (in the range [0, 100]) for RAPID motions.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getSpeedRatio(const abb_robot_msgs::srv::GetSpeedRatio::Request::SharedPtr req,
                     abb_robot_msgs::srv::GetSpeedRatio::Response::SharedPtr res);

  /**
   * \brief Sets all RAPID program pointers to respective main procedure.
   * Starts all RAPID programs.
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool ppToMain(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Signals that custom RAPID routine(s) should be run for all RAPID programs.
   *
   * Notes:
   * - Requires the StateMachine Add-In.
   * - The desired RAPID routine(s) needs to be specified beforehand (one per RAPID task).
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool runRAPIDRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Signals that SmartGripper command(s) should be run for all RAPID programs.
   *
   * Notes:
   * - Requires the StateMachine Add-In.
   * - The desired SmartGripper command(s) needs to be specified beforehand (one per RAPID task).
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool runSGRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Sets the contents of a file.
   *
   * The file will be uploaded to the robot controller's home directory.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setFileContents(const abb_robot_msgs::srv::SetFileContents::Request::SharedPtr req,
                       abb_robot_msgs::srv::SetFileContents::Response::SharedPtr res);

  /**
   * \brief Sets an IO-signal.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setIOSignal(const abb_robot_msgs::srv::SetIOSignal::Request::SharedPtr req,
                   abb_robot_msgs::srv::SetIOSignal::Response::SharedPtr res);

  /**
   * \brief Sets the motors off.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setMotorsOff(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Sets the motors on.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setMotorsOn(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                   abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Sets a RAPID 'bool' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDBool(const abb_robot_msgs::srv::SetRAPIDBool::Request::SharedPtr req,
                    abb_robot_msgs::srv::SetRAPIDBool::Response::SharedPtr res);

  /**
   * \brief Sets a RAPID 'dnum' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDDNum(const abb_robot_msgs::srv::SetRAPIDDnum::Request::SharedPtr req,
                    abb_robot_msgs::srv::SetRAPIDDnum::Response::SharedPtr res);

  /**
   * \brief Sets a RAPID 'num' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDNum(const abb_robot_msgs::srv::SetRAPIDNum::Request::SharedPtr req,
                   abb_robot_msgs::srv::SetRAPIDNum::Response::SharedPtr res);

  /**
   * \brief Sets a RAPID 'string' symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDString(const abb_robot_msgs::srv::SetRAPIDString::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDString::Response::SharedPtr res);

  /**
   * \brief Sets a RAPID symbol.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDSymbol(const abb_robot_msgs::srv::SetRAPIDSymbol::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDSymbol::Response::SharedPtr res);

  /**
   * \brief Sets the controller speed ratio (in the range [0, 100]) for RAPID motions.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setSpeedRatio(const abb_robot_msgs::srv::SetSpeedRatio::Request::SharedPtr req,
                     abb_robot_msgs::srv::SetSpeedRatio::Response::SharedPtr res);

  /**
   * \brief Starts all RAPID programs.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                  abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Stop all RAPID programs.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                 abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Gets EGM settings used by a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getEGMSettings(const abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Request::SharedPtr req,
                      abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Response::SharedPtr res);

  /**
   * \brief Sets EGM settings used by a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setEGMSettings(const abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Request::SharedPtr req,
                      abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Response::SharedPtr res);

  /**
   * \brief Sets desired custom RAPID routine for a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDRoutine(const abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Request::SharedPtr req,
                       abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Response::SharedPtr res);

  /**
   * \brief Sets desired SmartGripper command for a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setSGCommand(const abb_rapid_sm_addin_msgs::srv::SetSGCommand::Request::SharedPtr req,
                    abb_rapid_sm_addin_msgs::srv::SetSGCommand::Response::SharedPtr res);

  /**
   * \brief Starts EGM joint motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMJoint(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Starts EGM pose motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMPose(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Starts EGM position streaming for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Stops EGM motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopEGM(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
               abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Stops EGM position streaming for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param req request to process.
   * \param res response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief Verify that auto mode is active.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if auto mode is active.
   */
  bool verifyAutoMode(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a filename is not empty.
   *
   * \param filename to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the filename is not empty.
   */
  bool verifyArgumentFilename(const std::string& filename, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a RAPID symbol path does not contain any empty subcomponents.
   *
   * \param path to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID symbol path does not contain any empty subcomponents..
   */
  bool verifyArgumentRAPIDSymbolPath(const abb_robot_msgs::msg::RAPIDSymbolPath& path, uint16_t& result_code,
                                     std::string& message);

  /**
   * \brief Verify that a RAPID task name is not empty.
   *
   * \param task to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task name is not empty.
   */
  bool verifyArgumentRAPIDTask(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that an IO-signal name is not empty.
   *
   * \param signal to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the IO-signal name is not empty.
   */
  bool verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that motors are off.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the motors are off.
   */
  bool verifyMotorsOff(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that motors are on.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the motors are on.
   */
  bool verifyMotorsOn(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that runtime states has been received for StateMachine Add-In instances.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if runtime states has been received.
   */
  bool verifySMAddinRuntimeStates(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a StateMachine Add-In instance is used by a RAPID task.
   *
   * \param task to check.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task is used by a StateMachine Add-In instance.
   */
  bool verifySMAddinTaskExist(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a StateMachine Add-In instance is used by a RAPID task and that it has been initialized.
   *
   * \param task to check.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task is used by a StateMachine Add-In instance and has been initialized.
   */
  bool verifySMAddinTaskInitialized(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that RAPID is running.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if RAPID is running.
   */
  bool verifyRAPIDRunning(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that RAPID is stopped.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if RAPID is stopped.
   */
  bool verifyRAPIDStopped(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that the RWS communication manager is ready.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RWS communication manager is ready.
   */
  bool verifyRWSManagerReady(uint16_t& result_code, std::string& message);

  rclcpp::Node::SharedPtr node_;

  /**
   * \brief Manager for handling RWS communication with the robot controller.
   */
  abb::robot::RWSManager rws_manager_;

  /**
   * \brief Description of the connected robot controller.
   */
  abb::robot::RobotControllerDescription robot_controller_description_;

  /**
   * \brief Subscriber for robot controller system states.
   */
  rclcpp::Subscription<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_sub_;

  /**
   * \brief Subscriber for RobotWare StateMachine Add-In runtime states.
   */
  rclcpp::Subscription<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_sub_;

  /**
   * \brief List of provided ROS core services.
   */
  std::vector<rclcpp::ServiceBase::SharedPtr> core_services_;

  /**
   * \brief List of provided ROS state machine services.
   */
  std::vector<rclcpp::ServiceBase::SharedPtr> sm_services_;

  /**
   * \brief The latest known robot controller system state.
   */
  abb_robot_msgs::msg::SystemState system_state_;

  /**
   * \brief The latest known RobotWare StateMachine Add-In runtime state.
   */
  abb_rapid_sm_addin_msgs::msg::RuntimeState runtime_state_;
};

}  // namespace abb_rws_client
