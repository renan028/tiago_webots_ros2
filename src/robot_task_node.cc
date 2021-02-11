// tiago_webots_ros
#include <tiago_webots_ros/robot_task.h>
#include <nav1_behavior_tree/executor_bt.h>

// std
#include <signal.h>
#include <memory>

// ros
#include <ros/ros.h>

static std::atomic_bool killed(false);

void SignalHandler(int signal) {
  (void)signal;
  killed = true;
}

int main(int argc, char **argv) {
  // Initialize without sigint handler
  ros::init(argc, argv, "tiago_ros", ros::init_options::NoSigintHandler);

  // Install custom signal handler
  signal(SIGINT, SignalHandler);

  // Start an asyncronous spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
  tiago_webots_ros::RobotTask robot(*nh);
  nav1_behavior_tree::ExecutorBT tree_loader(nh);
  while (!killed) {
    ros::Duration(0.1).sleep();
  };

  // Clean exit
  spinner.stop();
  ros::shutdown();
}