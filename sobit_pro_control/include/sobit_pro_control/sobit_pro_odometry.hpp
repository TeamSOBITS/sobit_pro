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
  enum MODE {
    NONE = -1,
    STOP_MOTION_MODE,
    TRANSLATIONAL_MOTION_MODE,
    ROTATIONAL_MOTION_MODE,
    SWIVEL_MOTION_MODE // Motion can be added
  } motion_mode;

  rclcpp::Node* node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  SobitProOdometry(rclcpp::Node* node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "SobitProOdometry initialized.");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
  // Check how much of odometry is accepted
  ~SobitProOdometry() {
  RCLCPP_INFO(node_->get_logger(), "SobitProOdometry destroyed.");
  }

  bool odom(
    double steer_fl_curt_pos, double steer_fr_curt_pos,
    double steer_bl_curt_pos, double steer_br_curt_pos,
    double wheel_fl_curt_pos, double wheel_fr_curt_pos,
    double wheel_bl_curt_pos, double wheel_br_curt_pos,
    double wheel_fl_init_pos, double wheel_fr_init_pos,
    double wheel_bl_init_pos, double wheel_br_init_pos,
    double prev_motion,
    nav_msgs::msg::Odometry prev_odom, nav_msgs::msg::Odometry* result_odom);

  double distance_calculation(double wheel_curt_pos);
  double position_calculation(double steer_curt_pos);
  void pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom);

  MODE getMotion(int motion) {
    switch (motion) {
      case STOP_MOTION_MODE:          motion_mode = STOP_MOTION_MODE;          break;
      case TRANSLATIONAL_MOTION_MODE: motion_mode = TRANSLATIONAL_MOTION_MODE; break;
      case ROTATIONAL_MOTION_MODE:    motion_mode = ROTATIONAL_MOTION_MODE;    break;
      case SWIVEL_MOTION_MODE:        motion_mode = SWIVEL_MOTION_MODE;        break;
      default:                        motion_mode = NONE;                      break;
    }
    return motion_mode;
  }
};

#endif // SOBIT_PRO_ODOMETRY_HPP_