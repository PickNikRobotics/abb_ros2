#include "abb_hardware_interface/rws_client.hpp"

#include "abb_hardware_interface/utilities.hpp"

namespace abb_rws_client {
RWSClient::RWSClient(const rclcpp::Node::SharedPtr &node, const std::string &robot_ip, unsigned short robot_port)
    : node_(node),
      rws_manager_{robot_ip, robot_port, abb::rws::SystemConstants::General::DEFAULT_USERNAME,
                   abb::rws::SystemConstants::General::DEFAULT_PASSWORD} {
  node_->declare_parameter("robot_nickname", std::string{});
  node_->declare_parameter("no_connection_timeout", false);

  connect();
}
void RWSClient::connect() {
  std::string robot_id;
  bool no_connection_timeout;

  node_->get_parameter("robot_nickname", robot_id);
  node_->get_parameter("no_connection_timeout", no_connection_timeout);
  robot_controller_description_ =
      abb::robot::utilities::establish_rws_connection(rws_manager_, robot_id, no_connection_timeout);
  abb::robot::utilities::verify_robotware_version(robot_controller_description_.header().robot_ware_version());
}
}  // namespace abb_rws_client
