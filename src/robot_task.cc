#include <tiago_webots_ros2/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask() :
    Node("robot_task") {
  enableDevices();
}

RobotTask::~RobotTask() {}

void RobotTask::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr 
    scan) {
  RCLCPP_INFO(this->get_logger(), "I heard");
  sensor_msgs::msg::LaserScan msg = *scan;
  msg.header.frame_id = "laser_frame";
  std::vector<float> ranges = msg.ranges;
  std::reverse(ranges.begin(), ranges.end());
  msg.ranges = ranges;
  laser_pub_->publish(msg);
}

void RobotTask::enableCamera() {
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_2D/image_raw", 10, 
    std::bind(&RobotTask::updateImage, this, std::placeholders::_1));
}

void RobotTask::enableLidar() {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/Hokuyo_URG_04LX_UG01", 10, 
    std::bind(&RobotTask::updateLaserScan, this, std::placeholders::_1));
  //laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
}

void RobotTask::enableWheel() {
  wheels_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, 
    std::bind(&RobotTask::updateJoints, this, std::placeholders::_1));
}

void RobotTask::updateJoints(const sensor_msgs::msg::JointState::SharedPtr 
    joints) {
  wheels_ = *joints;
}

void RobotTask::updateImage(const sensor_msgs::msg::Image::SharedPtr image) {
  image_ = *image;
}

void RobotTask::enableDevices() {
  enableCamera();
  enableLidar();
  enableWheel();
}

} // end namespace tiago_webots_ros
