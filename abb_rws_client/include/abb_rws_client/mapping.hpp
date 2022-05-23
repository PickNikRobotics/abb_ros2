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
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/mapping.h

#pragma once

#include <string>
#include <vector>

#include <abb_egm_rws_managers/utilities.h>
#include <abb_librws/rws_rapid.h>

#include <abb_rapid_msgs/msg/tool_data.hpp>
#include <abb_rapid_msgs/msg/w_obj_data.hpp>
#include <abb_rapid_sm_addin_msgs/msg/egm_settings.hpp>

namespace abb {
namespace robot {
namespace utilities {

/**
 * \brief Maps RAPID task execution state to ROS representation.
 *
 * \param state to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map(const rws::RWSInterface::RAPIDTaskExecutionState state);

/**
 * \brief Maps RobotWare StateMachine Add-In state to ROS representation.
 *
 * \param state to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map_state_machine_state(const rws::RAPIDNum& state);

/**
 * \brief Maps RobotWare StateMachine Add-In EGM action to ROS representation.
 *
 * \param action to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map_state_machine_egm_action(const rws::RAPIDNum& action);

/**
 * \brief Maps RobotWare StateMachine Add-In SmartGripper command to RWS representation.
 *
 * \param command to map.
 *
 * \return unsigned int containing the mapped command.
 *
 * \throw std::runtime_error if the command is unknown.
 */
unsigned int map_state_machine_sg_command(const unsigned int command);

/**
 * \brief Maps a RAPID 'pos' data type from RWS to ROS representation.
 *
 * \param rws_pos to map.
 *
 * \return abb_rapid_msgs::Pos containing the mapped data.
 */
abb_rapid_msgs::msg::Pos map(const rws::Pos& rws_pos);

/**
 * \brief Maps a RAPID 'orient' data type from RWS to ROS representation.
 *
 * \param rws_orient to map.
 *
 * \return abb_rapid_msgs::Orient containing the mapped data.
 */
abb_rapid_msgs::msg::Orient map(const rws::Orient& rws_orient);

/**
 * \brief Maps a RAPID 'pose' data type from RWS to ROS representation.
 *
 * \param rws_pose to map.
 *
 * \return abb_rapid_msgs::Pose containing the mapped data.
 */
abb_rapid_msgs::msg::Pose map(const rws::Pose& rws_pose);

/**
 * \brief Maps a RAPID 'loaddata' data type from RWS to ROS representation.
 *
 * \param rws_loaddata to map.
 *
 * \return abb_rapid_msgs::LoadData containing the mapped data.
 */
abb_rapid_msgs::msg::LoadData map(const rws::LoadData& rws_loaddata);

/**
 * \brief Maps a RAPID 'tooldata' data type from RWS to ROS representation.
 *
 * \param rws_tooldata to map.
 *
 * \return abb_rapid_msgs::ToolData containing the mapped data.
 */
abb_rapid_msgs::msg::ToolData map(const rws::ToolData& rws_tooldata);

/**
 * \brief Maps a RAPID 'wobjdata' data type from RWS to ROS representation.
 *
 * \param rws_wobjdata to map.
 *
 * \return abb_rapid_msgs::WObjData containing the mapped data.
 */
abb_rapid_msgs::msg::WObjData map(const rws::WObjData& rws_wobjdata);

/**
 * \brief Maps a RobotWare StateMachine Add-In RAPID 'EGMSettings' data type from RWS to ROS
 * representation.
 *
 * \param rws_egm_settings to map.
 *
 * \return abb_rapid_sm_addin_msgs::EGMSettings containing the mapped data.
 */
abb_rapid_sm_addin_msgs::msg::EGMSettings map(const rws::RWSStateMachineInterface::EGMSettings& rws_egm_settings);

/**
 * \brief Maps a RAPID 'pos' data type from ROS to RWS representation.
 *
 * \param ros_pos to map.
 *
 * \return rws::Pos containing the mapped data.
 */
rws::Pos map(const abb_rapid_msgs::msg::Pos& ros_pos);

/**
 * \brief Maps a RAPID 'orient' data type from ROS to RWS representation.
 *
 * \param ros_orient to map.
 *
 * \return rws::Orient containing the mapped data.
 */
rws::Orient map(const abb_rapid_msgs::msg::Orient& ros_orient);

/**
 * \brief Maps a RAPID 'pose' data type from ROS to RWS representation.
 *
 * \param ros_pose to map.
 *
 * \return rws::Pose containing the mapped data.
 */
rws::Pose map(const abb_rapid_msgs::msg::Pose& ros_pose);

/**
 * \brief Maps a RAPID 'loaddata' data type from ROS to RWS representation.
 *
 * \param ros_loaddata to map.
 *
 * \return rws::LoadData containing the mapped data.
 */
rws::LoadData map(const abb_rapid_msgs::msg::LoadData& ros_loaddata);

/**
 * \brief Maps a RAPID 'tooldata' data type from ROS to RWS representation.
 *
 * \param ros_tooldata to map.
 *
 * \return rws::ToolData containing the mapped data.
 */
rws::ToolData map(const abb_rapid_msgs::msg::ToolData& ros_tooldata);

/**
 * \brief Maps a RAPID 'wobjdata' data type from ROS to RWS representation.
 *
 * \param ros_wobjdata to map.
 *
 * \return rws::WObjData containing the mapped data.
 */
rws::WObjData map(const abb_rapid_msgs::msg::WObjData& ros_wobjdata);

/**
 * \brief Maps a RobotWare StateMachine Add-In RAPID 'EGMSettings' data type from ROS to RWS
 * representation.
 *
 * \param ros_egm_settings to map.
 *
 * \return rws::RWSStateMachineInterface::EGMSettings containing the mapped data.
 */
rws::RWSStateMachineInterface::EGMSettings map(const abb_rapid_sm_addin_msgs::msg::EGMSettings& ros_egm_settings);

/**
 * \brief Maps EGM state to ROS representation.
 *
 * \param state to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map(const egm::wrapper::Status::EGMState state);

/**
 * \brief Maps motor state to ROS representation.
 *
 * \param state to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map(const egm::wrapper::Status::MotorState state);

/**
 * \brief Maps RAPID execution state to ROS representation.
 *
 * \param state to map.
 *
 * \return uint8_t containing the mapped state.
 */
uint8_t map(const egm::wrapper::Status::RAPIDExecutionState rapid_execution_state);

/**
 * \brief Maps a vector to a string (e.g. for logging).
 *
 * \param vector to map.
 *
 * \return std::string of the mapped vector.
 *
 * \throw std::runtime if the mapping failed.
 */
template <typename type>
std::string map_vector_to_string(const std::vector<type>& vector);

}  // namespace utilities
}  // namespace robot
}  // namespace abb
