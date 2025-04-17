#include "sobit_pro_library/sobit_pro_joint_action_server.hpp"

namespace sobit_pro{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));
  this->service_server_move_hand_to_coord_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord",
      std::bind(&JointActionServer::serve_move_hand_to_coord, this, std::placeholders::_1, std::placeholders::_2));
  this->service_server_move_hand_to_tf_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf",
      std::bind(&JointActionServer::serve_move_hand_to_tf, this, std::placeholders::_1, std::placeholders::_2));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
  this->pub_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory_controller/joint_trajectory", qos_profile);


  //Declare the pose parameters

  this->declare_parameter("poses", std::vector<std::string>());
  auto pose_names = this->get_parameter("poses").as_string_array();

  poses_.clear();
  for (auto pose_name : pose_names) {
    // Declare parameters for each pose
    this->declare_parameter(pose_name + ".arm_shoulder_1_tilt_joint"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_upper_1_tilt_joint", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_lower_tilt_joint"  , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_lower_pan_joint"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_tilt_joint"        , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_joint"                  , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_pan_joint"              , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_tilt_joint"             , rclcpp::PARAMETER_DOUBLE);

    // Read parameters for each pose
    PoseParams params;
    params.pose_name                    = pose_name;
    params.arm_shoulder_1_tilt_joint    = this->get_parameter(pose_name + ".arm_shoulder_1_tilt_joint").as_double();
    params.arm_elbow_upper_1_tilt_joint = this->get_parameter(pose_name + ".arm_elbow_upper_1_tilt_joint").as_double();
    params.arm_elbow_lower_tilt_joint   = this->get_parameter(pose_name + ".arm_elbow_lower_tilt_joint").as_double();
    params.arm_elbow_lower_pan_joint    = this->get_parameter(pose_name + ".arm_elbow_lower_pan_joint").as_double();
    params.arm_wrist_tilt_joint         = this->get_parameter(pose_name + ".arm_wrist_tilt_joint").as_double();
    params.hand_joint                   = this->get_parameter(pose_name + ".hand_joint").as_double();
    params.head_pan_joint               = this->get_parameter(pose_name + ".head_pan_joint").as_double();
    params.head_tilt_joint              = this->get_parameter(pose_name + ".head_tilt_joint").as_double();
    
    poses_.push_back(params);
  }

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
}
JointActionServer::~JointActionServer()
{
  this->action_server_move_joints_.reset();
  this->action_server_move_to_pose_.reset();

  this->sub_joint_state_.reset();
  this->pub_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been terminated.");
}


rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void JointActionServer::handle_move_joints_accepted(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
}

void JointActionServer::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}


void JointActionServer::exe_move_joints(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request. The number of joint names and joint rad are different");
    result->success = false;
    result->message = "Invalid goal request. The number of joint names and joint rad are different";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(JointNames.begin(), JointNames.end(), goal->target_joint_names[i]) == JointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "The joint name does not exist: %s", goal->target_joint_names[i].c_str());
      result->success = false;
      result->message = "The joint name does not exist: " + goal->target_joint_names[i];
      result->total_elapsed_time.sec = 0;
      result->total_elapsed_time.nanosec = 0;
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      this->pub_joint_control_->publish(set_joints({}, {}, dt));

      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();

  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::exe_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  // Check if the pose name is valid
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose) { return pose.pose_name == goal->pose_name; }) == poses_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    result->success = false;
    result->message = "Invalid pose name: " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad;
  for (const auto &pose : poses_) {
    if (pose.pose_name == goal->pose_name) {
      target_joint_rad.push_back(pose.arm_shoulder_1_tilt_joint);
      target_joint_rad.push_back(pose.arm_elbow_upper_1_tilt_joint);
      target_joint_rad.push_back(pose.arm_elbow_lower_tilt_joint);
      target_joint_rad.push_back(pose.arm_elbow_lower_pan_joint);
      target_joint_rad.push_back(pose.arm_wrist_tilt_joint);
      target_joint_rad.push_back(pose.hand_joint);
      target_joint_rad.push_back(pose.head_pan_joint);
      target_joint_rad.push_back(pose.head_tilt_joint);
      break;
    }
  }

  if (target_joint_rad.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to not find the pose name : %s", goal->pose_name.c_str());

    result->success = false;
    result->message = "[FAIL] Failed to not find the pose name : " +  goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
  }


  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      this->pub_joint_control_->publish(set_joints({}, {}, dt));
  
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = JointNames;
    for (const auto &joint_name : JointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < JointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[JointNames[i]] - target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->message = "[SUCCESS] Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::serve_move_hand_to_coord(
  const std::shared_ptr<MoveHandToTargetCoord::Request> request,
  std::shared_ptr<MoveHandToTargetCoord::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/arm_base_link";

  // Transform to robot base from 'sobit_pro/arm_base_link'
  try{
    goal_coord = tf_buffer_->transform(
      request->target_coord, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // double target_linear;

  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_1_tilt_joint","arm_elbow_upper_1_tilt_joint","arm_elbow_lower_tilt_joint","arm_elbow_lower_pan_joint","arm_wrist_tilt_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad);

  // if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
  //   target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  // } else {
  //   target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  // }

  response->move_pose.position.x = goal_coord.transform.translation.x - hand_pose.transform.translation.x;
  response->move_pose.position.y = goal_coord.transform.translation.y;
  response->move_pose.position.z = 0.0;

  RCLCPP_INFO(this->get_logger(), "move_pose.position.x (%.2f)",goal_coord.transform.translation.x - hand_pose.transform.translation.x);
  RCLCPP_INFO(this->get_logger(), "move_pose.position.y (%.2f)",goal_coord.transform.translation.y);
  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = 0.0;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::serve_move_hand_to_tf(
  const std::shared_ptr<MoveHandToTargetTF::Request> request,
  std::shared_ptr<MoveHandToTargetTF::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/arm_base_link";

  geometry_msgs::msg::TransformStamped goal_coord_shift;


  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->tf_differential.header.frame_id, request->target_frame,
      tf2::TimePointZero);

    geometry_msgs::msg::Vector3 euler_target, euler_shift;
    euler_target = get_euler_from_quat(goal_coord_shift.transform.rotation);
    euler_shift = get_euler_from_quat(request->tf_differential.transform.rotation);
    euler_target.x += euler_shift.x;
    euler_target.y += euler_shift.y;
    euler_target.z += euler_shift.z;

    goal_coord_shift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_coord_shift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_coord_shift.transform.translation.z += request->tf_differential.transform.translation.z;
    goal_coord_shift.transform.rotation = get_quat_from_euler(euler_target);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s to %s: %s", request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(),ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform: " + request->target_frame + " to: " + request->tf_differential.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // Transform to robot base from 'sobit_pro/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      goal_coord_shift, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s", goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // double target_linear;

  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_1_tilt_joint","arm_elbow_upper_1_tilt_joint","arm_elbow_lower_tilt_joint","arm_elbow_lower_pan_joint","arm_wrist_tilt_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (z: Height of Arm0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad);

  response->move_pose.position.x = goal_coord.transform.translation.x - hand_pose.transform.translation.x;
  response->move_pose.position.y = goal_coord.transform.translation.y - hand_pose.transform.translation.y;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = 0.0;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "arm_shoulder_2_tilt_joint") continue;  // Skip sub joints
    if (msg->name[i] == "arm_elbow_upper_2_tilt_joint") continue;  // Skip sub joints
    if (msg->name[i] == "wheel_f_r_steer_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_f_l_steer_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_b_r_steer_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_b_l_steer_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_f_r_drive_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_f_l_drive_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_b_r_drive_joint") continue;  // Skip wheel joints
    if (msg->name[i] == "wheel_b_l_drive_joint") continue;  // Skip wheel joints


    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }
  // RCLCPP_INFO(this->get_logger(), "Received joint state__");
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
  const std::vector<std::string> &target_joint_names,
  const std::vector<double> &target_joint_rad,
  const builtin_interfaces::msg::Duration &time_allowance)
{
  // Get current joint state from kCurrentJointState
  std::vector<double> full_target_joint_rad;
  for (size_t i = 0; i < JointNames.size(); i++) {
    full_target_joint_rad.push_back(this->curt_joint_state_[JointNames[i]]);
  }
  
  // Update the target joint rad
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(JointNames.begin(), JointNames.end(), target_joint_names[i]);
    full_target_joint_rad[std::distance(JointNames.begin(), it)] = target_joint_rad[i];
  }

  auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  joint_trajectory.header.stamp = this->now();
  joint_trajectory.points.resize(1);
  joint_trajectory.points[0].time_from_start = time_allowance;
  for (size_t i = 0; i < JointNames.size(); i++) {
    joint_trajectory.points[0].positions.push_back(full_target_joint_rad[i]);
    joint_trajectory.joint_names.push_back(JointNames[i]);

    // Add sub joints
    if (JointNames[i] == JointNames[JointIds::ARM_SHOULDER_1_TILT_JOINT]) {
      joint_trajectory.points[0].positions.push_back(-full_target_joint_rad[i]);
      joint_trajectory.joint_names.push_back("arm_shoulder_2_tilt_joint");
    }

    if (JointNames[i] == JointNames[JointIds::ARM_ELBOW_UPPER_1_TILT_JOINT]) {
      joint_trajectory.points[0].positions.push_back(-full_target_joint_rad[i]);
      joint_trajectory.joint_names.push_back("arm_elbow_upper_2_tilt_joint");
    }
  }

  return joint_trajectory;
}

geometry_msgs::msg::TransformStamped JointActionServer::forward_kinematics(
  const std::vector<double> &target_joint_rad)
{
  geometry_msgs::msg::TransformStamped hand_pose;

  hand_pose.transform.translation.x =   ARM_UPPER * cosf(target_joint_rad[0] )
                                      + ARM_INNER * cosf(target_joint_rad[0]  + target_joint_rad[1])
                                      + ARM_LOWER * cosf(target_joint_rad[0]  + target_joint_rad[1] + target_joint_rad[2]);
  hand_pose.transform.translation.y = 0.0;
  hand_pose.transform.translation.z =   ARM_UPPER * sinf(target_joint_rad[0] ) 
                                      + ARM_INNER * sinf(target_joint_rad[0]  + target_joint_rad[1])
                                      + ARM_LOWER * sinf(target_joint_rad[0]  + target_joint_rad[1] + target_joint_rad[2]);

  RCLCPP_INFO(this->get_logger(), "Hand_pose(x,y,z) = (%.5f, %.5f, %.5f)",hand_pose.transform.translation.x,hand_pose.transform.translation.y,hand_pose.transform.translation.z);
  return hand_pose;
}

std::vector<double> JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord)
{
  std::vector<double> target_joint_rad;
  double arm_to_target_x = goal_coord.transform.translation.x;
  // double arm_to_target_y = goal_coord.transform.translation.y;
  double arm_to_target_z = goal_coord.transform.translation.z;


   // Check if the arm can reach the target
  //  if( (arm_to_target_z < -(ARM_LENTGH+ARM_GRIPPER)) || (ARM_LENTGH < arm_to_target_z) ){
  //   return target_joint_rad;
  // }

  // Check if the arm can reach the target
  if( (arm_to_target_z < -ARM_LENTGH) || (ARM_LENTGH < arm_to_target_z) ){
    return target_joint_rad;
  }


  // Arm_shoulder_tilt_joint is set based on arm_to_target_z 
  double arm_to_arm_elbow_upper_tilt_joint_x = sqrtf(powf(ARM_UPPER, 2.) - powf(arm_to_target_z / 3., 2.));
  double arm_shoulder_tilt_joint_angle;

  if ( arm_to_target_z < 0 ) arm_shoulder_tilt_joint_angle = -acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);
  else                       arm_shoulder_tilt_joint_angle =  acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);

  double arm_elbow_upper_tilt_joint_to_target_x = arm_to_target_x - arm_to_arm_elbow_upper_tilt_joint_x;
  // double arm_elbow_upper_tilt_joint_to_target_y = arm_to_target_y;
  double arm_elbow_upper_tilt_joint_to_target_z = arm_to_target_z - (arm_to_target_z / 3.0);
  //std::cout << "arm_elbow_upper_tilt_joint_to_target_x: " << arm_elbow_upper_tilt_joint_to_target_x << ", arm_elbow_upper_tilt_joint_to_target_z: " << arm_elbow_upper_tilt_joint_to_target_z << std::endl;


  // Calculate the distance to move the wheels
  // double move_wheel_x = 0.0;
  // double move_wheel_y = arm_elbow_upper_tilt_joint_to_target_y;

  double diagonal_length = sqrtf(powf(arm_elbow_upper_tilt_joint_to_target_x, 2.) + powf(arm_elbow_upper_tilt_joint_to_target_z, 2));
  //std::cout << "diagonal_length: " << diagonal_length << std::endl;

  // The point reference is arm_elbow_upper_tilt_joint_to_target_z
  // diagonal_length is fixed to 30cm to calculate the distance to move the wheels
  if ( (ARM_INNER + ARM_LOWER) < diagonal_length || diagonal_length < ARM_INNER * sqrtf(2.) || arm_elbow_upper_tilt_joint_to_target_x <= 0 ) {
      double x     = sqrtf(powf(0.30, 2) - powf(arm_elbow_upper_tilt_joint_to_target_z, 2.));
      // move_wheel_x = arm_elbow_upper_tilt_joint_to_target_x - x;
      arm_elbow_upper_tilt_joint_to_target_x = x;
  }

  diagonal_length     = sqrtf(powf(arm_elbow_upper_tilt_joint_to_target_x, 2.) + powf(arm_elbow_upper_tilt_joint_to_target_z, 2.));
  double diagonal_angle      = atanf(arm_elbow_upper_tilt_joint_to_target_z / arm_elbow_upper_tilt_joint_to_target_x);
  double cos_arm_elbow_upper_tilt_joint_joint = (powf(diagonal_length, 2) + powf(ARM_INNER, 2.) - powf(ARM_LOWER, 2.)) / (2. * diagonal_length * ARM_INNER);

  if     ( cos_arm_elbow_upper_tilt_joint_joint >  1. ) cos_arm_elbow_upper_tilt_joint_joint =  1.;
  else if( cos_arm_elbow_upper_tilt_joint_joint < -1. ) cos_arm_elbow_upper_tilt_joint_joint = -1.;
  
  double arm_elbow_upper_tilt_joint_angle1 = -((arm_shoulder_tilt_joint_angle - diagonal_angle) + acosf(cos_arm_elbow_upper_tilt_joint_joint));
  double arm_elbow_upper_tilt_joint_angle2 = -((arm_shoulder_tilt_joint_angle - diagonal_angle) - acosf(cos_arm_elbow_upper_tilt_joint_joint));

  // As the lenght of other links are similar, we set the angle the same
  double external_angle = acosf(cos_arm_elbow_upper_tilt_joint_joint) + acosf(cos_arm_elbow_upper_tilt_joint_joint);
  double arm_elbow_lower_tilt_joint_angle1 =  external_angle;
  double arm_elbow_lower_tilt_joint_angle2 = -external_angle;

  double arm_wrist_tilt_joint_angle1 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle1 + arm_elbow_lower_tilt_joint_angle1);
  double arm_wrist_tilt_joint_angle2 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle2 + arm_elbow_lower_tilt_joint_angle2);

  RCLCPP_INFO(this->get_logger(), "inverseKinematics() Result 1: (%.2f, %.2f, %.2f, %.2f)",arm_shoulder_tilt_joint_angle,arm_elbow_upper_tilt_joint_angle1,arm_elbow_lower_tilt_joint_angle1,arm_wrist_tilt_joint_angle1);
  RCLCPP_INFO(this->get_logger(), "inverseKinematics() Result 2: (%.2f, %.2f, %.2f, %.2f)",arm_shoulder_tilt_joint_angle,arm_elbow_upper_tilt_joint_angle2,arm_elbow_lower_tilt_joint_angle2,arm_wrist_tilt_joint_angle2);

  std::vector<double> target_joint_rad_1{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle1, arm_elbow_lower_tilt_joint_angle1, 0, arm_wrist_tilt_joint_angle1};
  std::vector<double> target_joint_rad_2{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle2, arm_elbow_lower_tilt_joint_angle2, 0, arm_wrist_tilt_joint_angle2};

  // if(arm_to_target_z >= 0){
  //   if(target_joint_rad_1[1] >= 0) target_joint_rad = target_joint_rad_1;
  //   else                            target_joint_rad = target_joint_rad_2;
  // }
  // else{
  //   if(target_joint_rad_1[1] <= 0) target_joint_rad = target_joint_rad_1;
  //   else                          target_joint_rad = target_joint_rad_2;
  // }             
  
  target_joint_rad = target_joint_rad_1;

  RCLCPP_INFO(this->get_logger(), "arm_elbow_upper_tilt_joint (%.2f)",arm_elbow_upper_tilt_joint_angle1);

  return target_joint_rad;
}

} // namespace sobit_pro
