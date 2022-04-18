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
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, ControllerTopicsTest)
{
  // Sleep so that the other nodes have time to launch
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  auto topic_names_and_types = node_->get_topic_names_and_types();

  // Define a map of the topics to check and their types
  std::map<std::string, std::string> expected_topic_names_and_types = {
    { "/joint_trajectory_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory" },
    { "/joint_trajectory_controller/state", "control_msgs/msg/JointTrajectoryControllerState" }
  };
  for (const auto& [topic_name, topic_type] : expected_topic_names_and_types)
  {
    auto it = topic_names_and_types.find(topic_name);

    // Check if topic exists, fail if find() returns map.end()
    EXPECT_NE(it, topic_names_and_types.end());

    // Check if topic type is as expected
    EXPECT_EQ(it->second.front(), topic_type);
  }
}
}  // namespace minimal_integration_test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
