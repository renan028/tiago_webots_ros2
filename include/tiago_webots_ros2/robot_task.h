#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

// std
#include <algorithm>
#include <atomic>
#include <chrono>     
#include <functional>
#include <math.h>   
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// webots_ros
#include <webots_ros2_msgs/msg/recognition_objects.hpp>

namespace tiago_webots_ros {

class RobotTask : public rclcpp::Node {
  // robot
  std::string robot_model_;

  // subscribers and services
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheels_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<webots_ros2_msgs::msg::RecognitionObjects>::SharedPtr 
    recognition_sub_;

  // odom
  sensor_msgs::msg::JointState wheels_;

  // camera
  sensor_msgs::msg::Image image_;
  webots_ros2_msgs::msg::RecognitionObjects rec_objects_;

  // create a TF for lidar and camera
  void setTF();
  
  /** It updates the scan info. A subscription to the Laser sensor topic.
   * @param scan the scan coming from the range sensor
   */
  void updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  
  /** Update the wheels encoder. A subscription to the encoders' sensors topic.
   * @param joints the encoder info
   */
  void updateJoints(const sensor_msgs::msg::JointState::SharedPtr joints);

  /** Update the image. A subscription to the camera 2d topic.
   * @param image the encoder info
   */
  void updateImage(const sensor_msgs::msg::Image::SharedPtr image);

  /** Update the recognized objects. A subscription to the recognition topic.
   * @param image the encoder info
   */
  void updateRecognizedObjects(
    const webots_ros2_msgs::msg::RecognitionObjects::SharedPtr objects);
  
  /** A method to enable all useful devices for autonomous navigation.
   * 
   */ 
  void enableDevices();
  
  public: 
    RobotTask();
    ~RobotTask();

    void enableLidar();
    void enableWheel();    
    void enableCamera();    
    void enableRecognition();    
};

}