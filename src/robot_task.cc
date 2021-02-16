#include <tiago_webots_ros2/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask() :
    Node("robot_task") {
  enableDevices();
  setTF();
}

RobotTask::~RobotTask() {}

void RobotTask::setTF() {
  static tf2_ros::StaticTransformBroadcaster br(this);
  geometry_msgs::msg::TransformStamped transformStamped;
  
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = robot_model_ + "/Hokuyo_URG_04LX_UG01";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);

  transformStamped.child_frame_id = robot_model_ + "/camera_2D";
  br.sendTransform(transformStamped);

  transformStamped.child_frame_id = "laser_frame";
  br.sendTransform(transformStamped);
}

void RobotTask::enableCamera() {
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_2D/image_raw", rclcpp::SensorDataQoS(), 
    std::bind(&RobotTask::updateImage, this, std::placeholders::_1));
}

void RobotTask::enableRecognition() {
  recognition_sub_ = this->create_subscription<
    webots_ros2_msgs::msg::RecognitionObjects>("/camera_2D/recognition", 
      rclcpp::SensorDataQoS(), std::bind(&RobotTask::updateRecognizedObjects, 
        this, std::placeholders::_1));
}

void RobotTask::enableLidar() {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/Hokuyo_URG_04LX_UG01", rclcpp::SensorDataQoS(),
    std::bind(&RobotTask::updateLaserScan, this, std::placeholders::_1));
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 
    rclcpp::SensorDataQoS());
}

void RobotTask::enableWheel() {
  wheels_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(), 
    std::bind(&RobotTask::updateJoints, this, std::placeholders::_1));
}

void RobotTask::updateJoints(const sensor_msgs::msg::JointState::SharedPtr 
    joints) {
  wheels_ = *joints;
}

void RobotTask::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr 
    scan) {
  sensor_msgs::msg::LaserScan msg = *scan;
  msg.header.stamp = this->now();
  msg.header.frame_id = "laser_frame";
  std::vector<float> ranges = msg.ranges;
  std::reverse(ranges.begin(), ranges.end());
  msg.ranges = ranges;
  laser_pub_->publish(msg);
}

void RobotTask::updateImage(const sensor_msgs::msg::Image::SharedPtr image) {
  image_ = *image;
}

void RobotTask::updateRecognizedObjects(
    const webots_ros2_msgs::msg::RecognitionObjects::SharedPtr objects) {
  rec_objects_ = *objects;
}

void RobotTask::enableDevices() {
  enableCamera();
  enableRecognition();
  enableLidar();
  enableWheel();
}

} // end namespace tiago_webots_ros
