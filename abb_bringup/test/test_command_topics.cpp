// TO DO: Move this into abb_bringup - also note that the changes in CMakeLists will also need to be moved

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace minimal_integration_test
{
class TaskPlanningFixture : public testing::Test
{
public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override
  {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, ControllerTopicsTest)
{
  std::cout << "TEST BEGINNING!!" << std::endl;
  auto topics_names_and_types = node_->get_topic_names_and_types();
  for (const auto& [topic_name, topic_types] : topics_names_and_types)
  {
    // TO DO: Now that we have the topics, we need to actually include the abb_bringup launch
    std::cout << "Topic name: " << topic_name << std::endl;
    std::cout << "Topic types: ";
    for (const auto& topic_type : topic_types)
    {
      std::cout << topic_type << std::endl;
    }
  }
  // auto publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("topic", 10);

  EXPECT_TRUE(true);
}
}  // namespace minimal_integration_test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
