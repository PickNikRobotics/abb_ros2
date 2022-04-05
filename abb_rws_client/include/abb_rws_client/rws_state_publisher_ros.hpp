#ifndef ABB_RWS_CLIENT__RWS_STATE_PUBLISHER_ROS_HPP_
#define ABB_RWS_CLIENT__RWS_STATE_PUBLISHER_ROS_HPP_

#include "abb_rws_client/rws_client.hpp"

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS INTERFACES
#include <sensor_msgs/msg/joint_state.hpp>

// ABB INTERFACES
#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/rapid_task_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

namespace abb_rws_client {

class RWSStatePublisherROS : RWSClient {
 public:
  RWSStatePublisherROS(const rclcpp::Node::SharedPtr &node, const std::string &robot_ip, unsigned short robot_port);

 private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_pub_;

  abb::robot::MotionData motion_data_;
  abb::robot::SystemStateData system_state_data_;
};

}  // namespace abb_rws_client

#endif  // ABB_RWS_CLIENT__RWS_STATE_PUBLISHER_ROS_HPP_
