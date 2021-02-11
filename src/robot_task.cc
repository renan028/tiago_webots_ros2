#include <tiago_webots_ros2/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask() :
    Node("robot_task") {
  enableDevices(true);
}

RobotTask::~RobotTask() {}

void RobotTask::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr 
    scan) {
  sensor_msgs::msg::LaserScan msg = *scan;
  msg.header.frame_id = "laser_frame";
  std::vector<float> ranges = msg.ranges;
  std::reverse(ranges.begin(), ranges.end());
  msg.ranges = ranges;
  laser_pub_->publish(msg);
}

void RobotTask::enableLidar(bool enable) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/Hokuyo_URG_04LX_UG01", 10, 
    std::bind(&RobotTask::updateLaserScan, this, std::placeholders::_1));
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
}

void RobotTask::updateJoints(const sensor_msgs::msg::JointState::SharedPtr 
    joints) {
  wheels_ = *joints;
}

void RobotTask::enableWheel(bool enable) {
  // creating subscriber to encoder
  wheels_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, 
    std::bind(&RobotTask::updateJoints, this, std::placeholders::_1));
}

void RobotTask::enableDevices(bool enable) {
  enableLidar(enable);
  enableWheel(enable);
}

} // end namespace tiago_webots_ros
