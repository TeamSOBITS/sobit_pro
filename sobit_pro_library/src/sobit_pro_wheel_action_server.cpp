#include "sobit_pro_library/sobit_pro_wheel_action_server.hpp"

namespace sobit_pro{

WheelActionServer::WheelActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("wheel_action_server", options)
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_wheel_linear_ = rclcpp_action::create_server<MoveWheelLinear>(
      this,
      "move_wheel_linear",
      std::bind(&WheelActionServer::handle_move_wheel_linear_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_linear_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_linear_accepted, this, std::placeholders::_1));
  this->action_server_move_wheel_rotate_ = rclcpp_action::create_server<MoveWheelRotate>(
      this,
      "move_wheel_rotate",
      std::bind(&WheelActionServer::handle_move_wheel_rotate_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_accepted, this, std::placeholders::_1));


  this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", qos_profile);
  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_profile, std::bind(&WheelActionServer::odom_callback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "WheelActionServer has been initialized.");
}
WheelActionServer::~WheelActionServer()
{
  this->action_server_move_wheel_linear_.reset();
  this->action_server_move_wheel_rotate_.reset();

  this->pub_cmd_vel_.reset();
  this->sub_odom_.reset();

  RCLCPP_INFO(this->get_logger(), "WheelActionServer has been terminated.");
}


rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_linear_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveWheelLinear::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_rotate_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveWheelRotate::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_linear_cancel(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_rotate_cancel(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void WheelActionServer::handle_move_wheel_linear_accepted(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_linear, this, std::placeholders::_1), goal_handle}.detach();
}
void WheelActionServer::handle_move_wheel_rotate_accepted(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_rotate, this, std::placeholders::_1), goal_handle}.detach();
}


// TODO: goal time allowance is not considered
void WheelActionServer::exe_move_wheel_linear(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelLinear::Result>();

  // Check if the target point is valid (only x is considered)
  if (goal->target_point.z != 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid target point: (%f, %f, %f)",
        goal->target_point.x, goal->target_point.y, goal->target_point.z);
    result->success = false;
    result->message = "[FAIL] Invalid target point";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Initialize values
  geometry_msgs::msg::Twist out_vel;
  double goal_dist = std::sqrt(std::pow(goal->target_point.x, 2) + std::pow(goal->target_point.y, 2));
  double curt_dist=0.0;
  // double integral_dist = 0.0;
  // double prev_error_dist = goal_dist - curt_dist;


  // Set PID parameters
  // TODO: Get the parameters from the action goal
  double kp, ki, kd;
  kp = 0.1;
  ki = 0.4;
  kd = 0.8;

  double vel_diff = kp * goal_dist;

  // Set current time
  auto start_time = this->now();
  this->init_odom_ = this->curt_odom_;
  rclcpp::Rate loop_rate(10);

  while (curt_dist < goal_dist) {
    // Check if the goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      this->pub_cmd_vel_->publish(zero_vel_);

      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

      goal_handle->canceled(result);
      return;
    }

    // Get the current time
    auto curt_time = this->now();

    // Calculate the elapsed time
    rclcpp::Duration dur_elapsed_time = curt_time - start_time;
    double elapsed_time = dur_elapsed_time.nanoseconds() / 1e9; 

    double vel_linear = 0.0;

    // PID
    if ( goal_dist <= 0.1 ) {
      vel_linear =  kp * ( goal_dist + 0.001 - curt_dist )
                  - kd * vel_diff
                  + ki * ( goal_dist + 0.001 - curt_dist ) * std::pow( elapsed_time, 2 );
    } else {
      vel_linear =  kp * ( goal_dist + 0.001 - curt_dist )
                  - kd * vel_diff
                  + ki * ( goal_dist + 0.001 - curt_dist ) * std::pow( elapsed_time, 2 ) / 8.0 * goal_dist;
    }


    // // Calculate the current distance
    // double error_dist = goal_dist - curt_dist;
    // integral_dist += error_dist;
    // double derivative_dist = error_dist - prev_error_dist;

    // // Calculate the output velocity
    // out_vel.linear.x = 
    //     kp * error_dist +
    //     ki * integral_dist +
    //     kd * derivative_dist;

    // out_vel.linear.x = goal->target_point.x > 0 ? vel_linear : -vel_linear;
    out_vel.linear.x = vel_linear * std::cos(std::atan2(goal->target_point.y, goal->target_point.x));
    out_vel.linear.y = vel_linear * std::sin(std::atan2(goal->target_point.y, goal->target_point.x));

    // Publish the velocity
    this->pub_cmd_vel_->publish(out_vel);

    // Update the previous error
    curt_dist = std::sqrt(
        std::pow(this->curt_odom_.pose.pose.position.x - this->init_odom_.pose.pose.position.x, 2) +
        std::pow(this->curt_odom_.pose.pose.position.y - this->init_odom_.pose.pose.position.y, 2));
    // prev_error_dist = error_dist;

    // Publish feedback
    auto feedback = std::make_shared<MoveWheelLinear::Feedback>();
    feedback->current_point.x = curt_dist * std::cos(std::atan2(goal->target_point.y, goal->target_point.x));
    feedback->current_point.y = curt_dist * std::sin(std::atan2(goal->target_point.y, goal->target_point.x));
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
    goal_handle->publish_feedback(feedback);

    // Spin the node
    // rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();

  }

  this->pub_cmd_vel_->publish(zero_vel_);

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}


// TODO: goal time allowance is not considered
void WheelActionServer::exe_move_wheel_rotate(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelRotate::Result>();

  // Initialize values
  this->init_odom_ = this->curt_odom_;
  double init_real_angle = this->get_euler_from_quat(this->init_odom_.pose.pose.orientation).z;
  double curt_real_angle = this->get_euler_from_quat(this->curt_odom_.pose.pose.orientation).z;
  double prev_real_angle = init_real_angle;

  geometry_msgs::msg::Twist out_vel;
  double moved_angle = 0.0;
  double goal_angle = std::abs(goal->target_yaw);
  double goal_angle_deg = goal_angle * 180.0 / M_PI;

  // Set PID parameters
  // TODO: Get the parameters from the action goal
  double kp, ki, kd;
  kp = 0.1;
  ki = 0.4;
  kd = 0.8;

  double vel_diff = kp * goal->target_yaw;
  double max_angular_speed = 0.7;

  // Set current time
  auto start_time = this->now();
  rclcpp::Rate loop_rate(10);

  while (moved_angle < goal_angle) {
    // Check if the goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      this->pub_cmd_vel_->publish(zero_vel_);

      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      return;
    }

    // Get the current time
    auto curt_time = this->now();

    // Calculate the elapsed time
    rclcpp::Duration dur_elapsed_time = curt_time - start_time;
    double elapsed_time = dur_elapsed_time.nanoseconds() / 1e9; 

    double vel_angular = 0.0;

    if (goal_angle_deg < 30) {
      vel_angular = kp * (goal_angle + 0.001 - moved_angle)
                  - kd * vel_diff
                  + ki * (goal_angle + 0.001 - moved_angle) * pow(elapsed_time, 2);
    } else {
      vel_angular = kp * (goal_angle + 0.001 - moved_angle)
                  - kd * vel_diff
                  + ki * (goal_angle + 0.001 - moved_angle) * pow(elapsed_time, 2) * (22.5 / goal_angle);
    }

    // Apply the maximum speed limit
    vel_angular = (goal->target_yaw > 0) ? std::min(vel_angular, max_angular_speed) : -std::min(std::abs(vel_angular), max_angular_speed);
    out_vel.angular.z = vel_angular;
    vel_diff = vel_angular;

    // Publish the velocity
    this->pub_cmd_vel_->publish(out_vel);

    // Calculate the moved distance
    curt_real_angle = this->get_euler_from_quat(this->curt_odom_.pose.pose.orientation).z;

    double delta_angle = curt_real_angle - prev_real_angle;

    if (delta_angle > M_PI)       delta_angle -= 2 * M_PI;
    else if (delta_angle < -M_PI) delta_angle += 2 * M_PI;

    moved_angle += std::abs(delta_angle);
    prev_real_angle = curt_real_angle;

    // Publish feedback
    auto feedback = std::make_shared<MoveWheelRotate::Feedback>();
    feedback->current_yaw = moved_angle * (goal->target_yaw/std::abs(goal->target_yaw));
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
    goal_handle->publish_feedback(feedback);

    // Spin the node
    // rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();

  }

  this->pub_cmd_vel_->publish(zero_vel_);

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}


void WheelActionServer::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->curt_odom_ = *msg;
}

} // namespace sobit_pro
