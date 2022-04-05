#ifndef ABB_HARDWARE_INTERFACE__RWS_CLIENT_HPP__
#define ABB_HARDWARE_INTERFACE__RWS_CLIENT_HPP__

// ROS
#include <rclcpp/rclcpp.hpp>

// ABB
#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

namespace abb_rws_client {

class RWSClient {
 public:
  RWSClient(const rclcpp::Node::SharedPtr &node, const std::string &robot_ip, unsigned short robot_port);

 protected:
  rclcpp::Node::SharedPtr node_;
  abb::robot::RWSManager rws_manager_;

  abb::robot::RobotControllerDescription robot_controller_description_;

 private:
  void connect();
};

}  // namespace abb_rws_client

#endif  // ABB_HARDWARE_INTERFACE__RWS_CLIENT_HPP__
