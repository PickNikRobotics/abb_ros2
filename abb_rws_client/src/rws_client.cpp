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

#include <abb_rws_client/rws_client.hpp>

#include <abb_hardware_interface/utilities.hpp>

namespace abb_rws_client
{
RWSClient::RWSClient(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port)
  : node_(node)
  , rws_manager_{ robot_ip, robot_port, abb::rws::SystemConstants::General::DEFAULT_USERNAME,
                  abb::rws::SystemConstants::General::DEFAULT_PASSWORD }
{
  node_->declare_parameter("robot_nickname", std::string{});
  node_->declare_parameter("no_connection_timeout", false);

  connect();
}
void RWSClient::connect()
{
  std::string robot_id;
  bool no_connection_timeout;

  node_->get_parameter("robot_nickname", robot_id);
  node_->get_parameter("no_connection_timeout", no_connection_timeout);
  robot_controller_description_ =
      abb::robot::utilities::establishRWSConnection(rws_manager_, robot_id, no_connection_timeout);
  abb::robot::utilities::verifyRobotWareVersion(robot_controller_description_.header().robot_ware_version());
}
}  // namespace abb_rws_client
