// tiago_webots_ros
#include <tiago_webots_ros2/robot_task.h>

// std
#include <signal.h>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // Initialize without sigint handler
  rclcpp::init(argc, argv);

  // Start an asyncronous spinner
  auto node = std::make_shared<tiago_webots_ros::RobotTask>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}