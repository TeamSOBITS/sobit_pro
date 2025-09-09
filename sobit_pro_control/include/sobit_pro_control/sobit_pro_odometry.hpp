#ifndef SOBIT_PRO_ODOMETRY_HPP_
#define SOBIT_PRO_ODOMETRY_HPP_

#include <cmath>
#include <memory>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>

class SobitProOdometry{
private:
  rclcpp::Node* node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  SobitProOdometry(rclcpp::Node* node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "SobitProOdometry initialized.");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
  ~SobitProOdometry() {
  RCLCPP_INFO(node_->get_logger(), "SobitProOdometry destroyed.");
  }

  nav_msgs::msg::Odometry odom(
    double steer_fl_curt_pos, double steer_fr_curt_pos,
    double steer_bl_curt_pos, double steer_br_curt_pos,
    double wheel_fl_curt_pos, double wheel_fr_curt_pos,
    double wheel_bl_curt_pos, double wheel_br_curt_pos,
    double wheel_fl_prev_pos, double wheel_fr_prev_pos,
    double wheel_bl_prev_pos, double wheel_br_prev_pos,
    nav_msgs::msg::Odometry prev_odom);

  double distance_calculation(double wheel_delta_pos);
  void pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom);
};

#endif // SOBIT_PRO_ODOMETRY_HPP_