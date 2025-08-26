#include "sobits_interfaces/action/move_wheel_linear.hpp"
#include "sobits_interfaces/action/move_wheel_rotate.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace sobit_pro
{

class WheelActionServer : public rclcpp::Node
{
public:
  using MoveWheelLinear = sobits_interfaces::action::MoveWheelLinear;
  using MoveWheelRotate = sobits_interfaces::action::MoveWheelRotate;

  using GoalHandleMoveWheelLinear = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveWheelLinear>;
  using GoalHandleMoveWheelRotate = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveWheelRotate>;

  explicit WheelActionServer(const rclcpp::NodeOptions & options);
  ~WheelActionServer();

  geometry_msgs::msg::Vector3 get_euler_from_quat(
    const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(
    const geometry_msgs::msg::Vector3& rpy);

private:
  nav_msgs::msg::Odometry init_odom_;
  nav_msgs::msg::Odometry curt_odom_;
  geometry_msgs::msg::Twist zero_vel_;

  rclcpp_action::Server<MoveWheelLinear>::SharedPtr action_server_move_wheel_linear_;
  rclcpp_action::Server<MoveWheelRotate>::SharedPtr action_server_move_wheel_rotate_;

  rclcpp_action::GoalResponse handle_move_wheel_linear_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveWheelLinear::Goal> goal);
  rclcpp_action::GoalResponse handle_move_wheel_rotate_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveWheelRotate::Goal> goal);

  rclcpp_action::CancelResponse handle_move_wheel_linear_cancel(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle);
  rclcpp_action::CancelResponse handle_move_wheel_rotate_cancel(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle);

  void handle_move_wheel_linear_accepted(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle);
  void handle_move_wheel_rotate_accepted(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle);

  void exe_move_wheel_linear(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle);
  void exe_move_wheel_rotate(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle);


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
}; // class WheelActionServer

inline geometry_msgs::msg::Vector3 WheelActionServer::get_euler_from_quat(
  const geometry_msgs::msg::Quaternion& msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;  
}

inline geometry_msgs::msg::Quaternion WheelActionServer::get_quat_from_euler(
  const geometry_msgs::msg::Vector3& euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_pro

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_pro::WheelActionServer)