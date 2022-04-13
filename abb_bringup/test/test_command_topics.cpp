#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace minimal_integration_test {
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, ControllerTopicsTest) {

  std::cout << "TEST BEGINNING!!" << std::endl;

  // Sleep so that the other nodes have time to launch
  // TODO: is there a better way to do this?
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  auto topic_names_and_types = node_->get_topic_names_and_types();

  // TODO: Is there a better condition to use here?
  bool test_success = true;
  std::map<std::string, std::string> expected_topic_names_and_types = {
    {
      "/joint_trajectory_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory"
    },
    {
      "/joint_trajectory_controller/state", "control_msgs/msg/JointTrajectoryControllerState"
    }
  };
  for (const auto& [topic_name, topic_type] : expected_topic_names_and_types){
    auto it = topic_names_and_types.find(topic_name);
    if (it == topic_names_and_types.end()){
      test_success = false;
    } else {
      if (it->second.front() != topic_type){
        test_success = false;
      } else {
        std::cout << "Found topic " << it->first << " with type: " << it->second.front()  << std::endl;
      }
    }
  }

  // for (const auto& [topic_name, topic_type] : topic_names_and_types){
  //   // TO DO: Now that we have the topics, we need to actually include the abb_bringup launch
  //   // Find relevant topic names and call map.find(topic_name), if this == map.end(), test fails
  //   // Check map.find(topic)->second, if this is not the expected type, test fails
  //   // Topic name: /joint_trajectory_controller/joint_trajectory
  //   // Topic types: trajectory_msgs/msg/JointTrajectory
  //   // Topic name: /joint_trajectory_controller/state
  //   // Topic types: control_msgs/msg/JointTrajectoryControllerState
  //   std::cout << "Topic name: " << topic_name << std::endl;
  //   std::cout << "Topic types: ";
  //   for (const auto& topic_type : topic_type){
  //     std::cout << topic_type << std::endl;
  //   }
  // }
  // auto publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("topic", 10);

  EXPECT_TRUE(test_success);
}
}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
