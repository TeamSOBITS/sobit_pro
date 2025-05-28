#include "sobit_pro_control/sobit_pro_main.hpp"

namespace sobit_pro{

SobitProMain::SobitProMain(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("sobit_pro_main", options)
{
  // Start up sound
  this->start_up_sound();

  // Initialize the control and odometry classes
  sobit_pro_control_ = std::make_unique<SobitProControl>(this);
  sobit_pro_odometry_ = std::make_unique<SobitProOdometry>(this);

  // Configure the QoS profile
  rclcpp::QoS qos_profile(1);
  // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  this->sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos_profile, std::bind(&SobitProMain::callback, this, std::placeholders::_1));
  this->sub_joint_info_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&SobitProMain::joint_callback, this, std::placeholders::_1));

  this->pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos_profile);
  this->pub_steer_joint_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "steer_joint_trajectory_controller/joint_trajectory", qos_profile);
  this->pub_wheel_joint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "velocity_controller/commands", qos_profile);
  this->pub_wheels_error_ = this->create_publisher<std_msgs::msg::Bool>(
      "wheels_error", qos_profile);

  this->control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SobitProMain::control_callback, this));


  // Initialize the wheel and steer positions
  steer_joint_trajectory.joint_names.resize(4);
  steer_joint_trajectory.points.resize(1);
  steer_joint_trajectory.points[0].positions.resize(4);
  wheel_joint_vel.data.resize(4);

  is_steer_movable = true;


  // [SIM] Set the initial position of the wheel
  wheel_fl_init_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint");
  wheel_fr_init_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint");
  wheel_bl_init_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint");
  wheel_br_init_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint");

  // Get the robot namespace
  std::string robot_name = (std::strcmp(this->get_namespace(), "/") != 0)
                          ? std::string(this->get_namespace()).substr(1) + "/"
                          : "";
  
  // Initilize Odometry // [NOT for Isaac Sim!]
  prev_odom.header.stamp            = this->get_clock()->now();
  prev_odom.header.frame_id         = robot_name + "odom";
  prev_odom.child_frame_id          = robot_name + "base_footprint";
  prev_odom.pose.pose.position.x    = 0.0;
  prev_odom.pose.pose.position.y    = 0.0;
  prev_odom.pose.pose.position.z    = 0.0;
  prev_odom.pose.pose.orientation.x = 0.0;
  prev_odom.pose.pose.orientation.y = 0.0;
  prev_odom.pose.pose.orientation.z = 0.0;
  prev_odom.pose.pose.orientation.w = 1.0;
  prev_odom.twist.twist.linear.x    = 0.0;
  prev_odom.twist.twist.linear.y    = 0.0;
  prev_odom.twist.twist.linear.z    = 0.0;
  prev_odom.twist.twist.angular.x   = 0.0;
  prev_odom.twist.twist.angular.y   = 0.0;
  prev_odom.twist.twist.angular.z   = 0.0;

  result_odom.header.stamp            = this->get_clock()->now();
  result_odom.header.frame_id         = robot_name + "odom";
  result_odom.child_frame_id          = robot_name + "base_footprint";
  result_odom.pose.pose.position.x    = 0.0;
  result_odom.pose.pose.position.y    = 0.0;
  result_odom.pose.pose.position.z    = 0.0;
  result_odom.pose.pose.orientation.x = 0.0;
  result_odom.pose.pose.orientation.y = 0.0;
  result_odom.pose.pose.orientation.z = 0.0;
  result_odom.pose.pose.orientation.w = 1.0;
  result_odom.twist.twist.linear.x    = 0.0;
  result_odom.twist.twist.linear.y    = 0.0;
  result_odom.twist.twist.linear.z    = 0.0;
  result_odom.twist.twist.angular.x   = 0.0;
  result_odom.twist.twist.angular.y   = 0.0;
  result_odom.twist.twist.angular.z   = 0.0;

  RCLCPP_INFO(this->get_logger(), "SobitProMain initialized.");
}

SobitProMain::~SobitProMain()
{
  // Shut down sound
  this->shut_down_sound();
}

// Twist callback
void SobitProMain::callback(const geometry_msgs::msg::Twist::SharedPtr vel_twist)
{
  curt_vel_twist = *vel_twist;

  // Translational
  if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000))
      && (std::fabs(vel_twist->angular.z) <= 0.001)) {
    motion = SobitProControl::TRANSLATIONAL_MOTION;
    wheels_error.data = false;
    pub_wheels_error_->publish(wheels_error);
  }
  // Rotational
  else if (((std::fabs(vel_twist->linear.x) <= 0.001) && (std::fabs(vel_twist->linear.y) <= 0.001))
      && (std::fabs(vel_twist->angular.z) > 0.000)) {
    motion = SobitProControl::ROTATIONAL_MOTION;
    wheels_error.data = false;
    pub_wheels_error_->publish(wheels_error);
  }
  // Swivel
  else if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000))
      && (std::fabs(vel_twist->angular.z) > 0.000)) {
    if (std::fabs(2.0 * std::sqrt(std::pow(vel_twist->linear.x, 2.0) + std::pow(vel_twist->linear.y, 2.0))) >
      std::fabs(vel_twist->angular.z * SobitProControl::TRACK)) {
      motion = SobitProControl::SWIVEL_MOTION;
    }
    else {
      motion = SobitProControl::ROTATIONAL_MOTION;
    }
    wheels_error.data = false;
    pub_wheels_error_->publish(wheels_error);
  }
  // Stop
  else {
    motion = SobitProControl::STOP_MOTION;
    wheels_error.data = false;
    pub_wheels_error_->publish(wheels_error);
  }

  sobit_pro_control_->getMotion(motion);
  sobit_pro_odometry_->getMotion(motion);
  sobit_pro_control_->setParams(*vel_twist);
}

void SobitProMain::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_info)
{
  for (size_t i = 0; i < joint_info->name.size(); ++i) {
    joints_pos[joint_info->name[i]] = joint_info->position[i];
    joints_vel[joint_info->name[i]] = joint_info->velocity[i];
  }
}

// Start up sound
bool SobitProMain::start_up_sound()
{
  bool is_sound = false;

  // Generate a random number
  std::random_device rnd;
  std::mt19937 gen(rnd());
  std::uniform_int_distribution<int> distribution(1, 100);
  int rand_sound = distribution(gen);

  // Obtain the parameter value
  this->declare_parameter("sound_param", int(95));
  auto sound_param = this->get_parameter("sound_param").as_int();

  // Determine the sound to play
  std::string sound = rand_sound <= sound_param ? "start_up" : "soka_univ_gakuseika";

  // Get the package path
  std::string pack_path = ament_index_cpp::get_package_share_directory("sobit_pro_control");
  std::string sound_path = pack_path + "/mp3/" + sound + ".mp3";

  // Log output
  std::cout << std::endl;
  std::cout << "rand_sound: " << rand_sound << std::endl;
  std::cout << "sound_param: " << sound_param << std::endl;
  std::cout << "Start Up: " << sound << ".mp3" << std::endl;
  std::cout << std::endl;

  // Play sound
  is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());
  // rclcpp::sleep_for(std::chrono::seconds(2));

  if (is_sound) {
    RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the start up sound.");
  }

  return is_sound;
}

// Shutdown sound
bool SobitProMain::shut_down_sound()
{
  bool is_sound = false;

  // Get the package path
  std::string package_path = ament_index_cpp::get_package_share_directory("sobit_pro_control");
  std::string sound_path   = package_path + "/mp3/shut_down.mp3";

  // Log output
  std::cout << std::endl;
  std::cout << "Shutdown Sound" << std::endl;

  // Play sound
  is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());
  // rclcpp::sleep_for(std::chrono::seconds(2));

  if (is_sound) {
    RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the shutdown sound.");
  }

  return is_sound;
}

// Control wheel
void SobitProMain::control_callback()
{
  // [SIM] Wait for the joint_states to be published
  if (joints_pos.empty() || joints_vel.empty()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for joint_states to be published...");
    return;
  }

  set_steer_pos = sobit_pro_control_->setSteerPos();

  steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint");
  steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint");
  steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint");
  steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint");

  steer_joint_trajectory.joint_names.clear();
  steer_joint_trajectory.points.clear();

  setPosJointTrajectory("wheel_f_l_steer_joint", set_steer_pos[0], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_f_r_steer_joint", set_steer_pos[1], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_b_l_steer_joint", set_steer_pos[2], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_b_r_steer_joint", set_steer_pos[3], 0.1, &steer_joint_trajectory);

  // TODO: find a better way to check if the steer joint is movable
  // if (is_steer_movable) {
  //   if (checkPublishersConnection("cmd_vel")
  //       || checkPublishersConnection("navigate_to_pose/goal")) { // TODO: check the topic name
  //     if (std::fabs(curt_vel_twist.linear.x) > 0.001
  //         || std::fabs(curt_vel_twist.linear.y) > 0.001
  //         || std::fabs(curt_vel_twist.angular.z) > 0.001) {
  //       RCLCPP_DEBUG(this->get_logger(), "Publishing steer joint trajectory...");
  //       pub_steer_joint_->publish(steer_joint_trajectory);
  //     }
  //   }
  // }
  if (is_steer_movable) {
    RCLCPP_DEBUG(this->get_logger(), "Publishing steer joint trajectory...");
    pub_steer_joint_->publish(steer_joint_trajectory);
  }
  else {
    RCLCPP_DEBUG(this->get_logger(), "Steer joint is not movable.");
  }

  steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint");
  steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint");
  steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint");
  steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint");

  if ((SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[0] - steer_fl_curt_pos))
  || (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[1] - steer_fr_curt_pos))
  || (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[2] - steer_bl_curt_pos))
  || (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[3] - steer_br_curt_pos))){
    RCLCPP_INFO(this->get_logger(), "Waiting for the steering to reach the target position...");
    RCLCPP_INFO(this->get_logger(), "set_steer_pos:%.5f,%.5f,%.5f,%.5f", set_steer_pos[0],set_steer_pos[1],set_steer_pos[2],set_steer_pos[3]);
    RCLCPP_INFO(this->get_logger(), "set_steer_curt_pos:%.5f,%.5f,%.5f,%.5f", steer_fl_curt_pos, steer_fr_curt_pos, steer_bl_curt_pos, steer_br_curt_pos);
    RCLCPP_INFO(this->get_logger(), "\n0.174533 to position result1: %.3f", fabs(set_steer_pos[0] - steer_fl_curt_pos));
    RCLCPP_INFO(this->get_logger(), "0.174533 to position result2: %.3f", fabs(set_steer_pos[1] - steer_fl_curt_pos));
    RCLCPP_INFO(this->get_logger(), "0.174533 to position result3: %.3f", fabs(set_steer_pos[2] - steer_fl_curt_pos));
    RCLCPP_INFO(this->get_logger(), "0.174533 to position result4: %.3f", fabs(set_steer_pos[3] - steer_fl_curt_pos));
    set_wheel_vel[0] = set_wheel_vel[1] = set_wheel_vel[2] = set_wheel_vel[3] = 0.;
    is_steer_movable = false;
  }
  else {
    set_wheel_vel = sobit_pro_control_->setWheelVel();
    is_steer_movable = true;
  }


  // [SIM] Publish Float64MultiArray [rad/s]
  wheel_joint_vel.data.clear();

  wheel_joint_vel.data.push_back(set_wheel_vel[0]);
  wheel_joint_vel.data.push_back(set_wheel_vel[1]);
  wheel_joint_vel.data.push_back(set_wheel_vel[2]);
  wheel_joint_vel.data.push_back(set_wheel_vel[3]);

  // checkPublishersConnection("velocity_controller/commands");
  pub_wheel_joint_->publish(wheel_joint_vel);

  // [SIM] Update the current wheel position
  wheel_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint");
  wheel_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint");
  wheel_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint");
  wheel_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint");

  // Calculate Odometry based on motion mode (check!) // [NOT for Isaac Sim!]
  sobit_pro_odometry_->odom(steer_fl_curt_pos, steer_fr_curt_pos,
                            steer_bl_curt_pos, steer_br_curt_pos,
                            wheel_fl_curt_pos, wheel_fr_curt_pos,
                            wheel_bl_curt_pos, wheel_br_curt_pos,
                            wheel_fl_init_pos, wheel_fr_init_pos,
                            wheel_bl_init_pos, wheel_br_init_pos,
                            prev_motion,
                            prev_odom, &result_odom);

  // Update the initial wheel position value for next loop calculation
  wheel_fl_init_pos = wheel_fl_curt_pos;
  wheel_fr_init_pos = wheel_fr_curt_pos;
  wheel_bl_init_pos = wheel_bl_curt_pos;
  wheel_br_init_pos = wheel_br_curt_pos;

  // Update odom for next loop calculation // [NOT for Isaac Sim!]
  prev_odom.pose.pose.position.x    = result_odom.pose.pose.position.x;
  prev_odom.pose.pose.position.y    = result_odom.pose.pose.position.y;
  prev_odom.pose.pose.position.z    = result_odom.pose.pose.position.z;
  prev_odom.pose.pose.orientation.x = result_odom.pose.pose.orientation.x;
  prev_odom.pose.pose.orientation.y = result_odom.pose.pose.orientation.y;
  prev_odom.pose.pose.orientation.z = result_odom.pose.pose.orientation.z;
  prev_odom.pose.pose.orientation.w = result_odom.pose.pose.orientation.w;
  prev_odom.twist.twist.linear.x    = result_odom.twist.twist.linear.x;
  prev_odom.twist.twist.linear.y    = result_odom.twist.twist.linear.y;
  prev_odom.twist.twist.linear.z    = result_odom.twist.twist.linear.z;
  prev_odom.twist.twist.angular.x   = result_odom.twist.twist.angular.x;
  prev_odom.twist.twist.angular.y   = result_odom.twist.twist.angular.y;
  prev_odom.twist.twist.angular.z   = result_odom.twist.twist.angular.z;


  // Publish Odometry
  sobit_pro_odometry_->pose_broadcaster(result_odom);
  pub_odometry_->publish(result_odom);
}

} // namespace sobit_pro
