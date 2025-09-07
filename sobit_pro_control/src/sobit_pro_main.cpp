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

  // Set the initial position of the wheel
  // rclcpp::Rate rate(50);
  joints_pos.clear();
  while (joints_pos.empty()) rclcpp::spin_some(this->get_node_base_interface());/* rate.sleep();*/
  sobit_pro_control_->steer_fl_goal_pos = ( 1./4.) * M_PI;
  sobit_pro_control_->steer_fr_goal_pos = (-1./4.) * M_PI;
  sobit_pro_control_->steer_bl_goal_pos = (-1./4.) * M_PI;
  sobit_pro_control_->steer_br_goal_pos = ( 1./4.) * M_PI;
  wheel_fl_prev_pos = joints_pos["wheel_f_l_drive_joint"];
  wheel_fr_prev_pos = joints_pos["wheel_f_r_drive_joint"];
  wheel_bl_prev_pos = joints_pos["wheel_b_l_drive_joint"];
  wheel_br_prev_pos = joints_pos["wheel_b_r_drive_joint"];

  // create looped function of 50hz
  this->control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SobitProMain::control_callback, this));

  // Initialize the wheel and steer positions
  // steer_joint_trajectory.joint_names.resize(4);
  // steer_joint_trajectory.points.resize(1);
  // steer_joint_trajectory.points[0].positions.resize(4);
  // wheel_joint_vel.data.resize(4);

  // Get the robot namespace
  robot_name = (std::strcmp(this->get_namespace(), "/") != 0)
              ? std::string(this->get_namespace()).substr(1) + "/"
              : "";
  
  // Initilize Odometry
  prev_odom.header.stamp            = this->get_clock()->now();
  prev_odom.header.frame_id         = robot_name + "odom";
  prev_odom.child_frame_id          = robot_name + "base_footprint";

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
  // Translational
  if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000))
      && (std::fabs(vel_twist->angular.z) == 0.000))
    sobit_pro_control_->motion_mode = SobitProControl::MODE::TRANSLATIONAL_MOTION_MODE;
  // Rotational
  else if (((std::fabs(vel_twist->linear.x) <= 0.001) && (std::fabs(vel_twist->linear.y) <= 0.001))
      && (std::fabs(vel_twist->angular.z) > 0.000))
    sobit_pro_control_->motion_mode = SobitProControl::MODE::ROTATIONAL_MOTION_MODE;
  // Swivel
  else if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000))
      && (std::fabs(vel_twist->angular.z) > 0.000)) {
    if (std::fabs(2.0 * std::sqrt(std::pow(vel_twist->linear.x, 2.0) + std::pow(vel_twist->linear.y, 2.0))) >
      std::fabs(vel_twist->angular.z * SobitProControl::TRACK))
         sobit_pro_control_->motion_mode = SobitProControl::MODE::SWIVEL_MOTION_MODE;
    else sobit_pro_control_->motion_mode = SobitProControl::MODE::ROTATIONAL_MOTION_MODE; // base_circle point is inner of robot
  }
  // Stop
  else sobit_pro_control_->motion_mode = SobitProControl::MODE::STOP_MOTION_MODE;

  sobit_pro_control_->setParams(*vel_twist);
}

void SobitProMain::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_info)
{
  for (size_t i = 0; i < joint_info->name.size(); ++i) 
    joints_pos[joint_info->name[i]] = joint_info->position[i];
}

// Start up sound
bool SobitProMain::start_up_sound()
{
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

  // Play sound
  bool is_sound = false;
  is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());

  // sound debug
  if (is_sound) RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the start up sound.");

  return is_sound;
}

// Shutdown sound
bool SobitProMain::shut_down_sound()
{
  // Get the package path
  std::string package_path = ament_index_cpp::get_package_share_directory("sobit_pro_control");
  std::string sound_path   = package_path + "/mp3/shut_down.mp3";

  // Log output
  std::cout << std::endl;
  std::cout << "Shutdown Sound" << std::endl;

  // Play sound
  bool is_sound = false;
  is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());

  // sound debug
  if (is_sound) RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the shutdown sound.");

  return is_sound;
}

// Control wheel
void SobitProMain::control_callback()
{
  /**
  * @brief Main control loop for SOBIT PRO wheel and steering motion.
  * 
  * - Computes desired steering angles and wheel velocities.
  * - Runs a state machine to detect stuck conditions and trigger recovery.
  * - Publishes wheel and steering commands.
  * - Computes and publishes odometry from joint feedback.
  */

  // Tracks the last sent steer command to avoid unnecessary republishing
  static std::array<double, 4> last_sent_steer_pos = {0.0, 0.0, 0.0, 0.0};

  // Wait until joint_states topic is populated (needed for steering and odometry calculations)
  if (joints_pos.empty()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for joint_states to be published...");
    return;
  }

  // Calculate desired steer positions and update current ones
  set_steer_pos = sobit_pro_control_->setSteerPos();

  // Update current steer positions
  steer_fl_curt_pos = joints_pos["wheel_f_l_steer_joint"];
  steer_fr_curt_pos = joints_pos["wheel_f_r_steer_joint"];
  steer_bl_curt_pos = joints_pos["wheel_b_l_steer_joint"];
  steer_br_curt_pos = joints_pos["wheel_b_r_steer_joint"];

  // Check if all current steering joint positions are within acceptable threshold of their targets
  bool all_aligned = 
      fabs(set_steer_pos[0] - steer_fl_curt_pos) <= SobitProControl::DXL_MOVING_STATUS_THRESHOLD &&
      fabs(set_steer_pos[1] - steer_fr_curt_pos) <= SobitProControl::DXL_MOVING_STATUS_THRESHOLD &&
      fabs(set_steer_pos[2] - steer_bl_curt_pos) <= SobitProControl::DXL_MOVING_STATUS_THRESHOLD &&
      fabs(set_steer_pos[3] - steer_br_curt_pos) <= SobitProControl::DXL_MOVING_STATUS_THRESHOLD;


  // Prepare steer trajectory (always, for republishing)
  steer_joint_trajectory.joint_names.clear();
  steer_joint_trajectory.points.clear();
  setPosJointTrajectory("wheel_f_l_steer_joint", set_steer_pos[0], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_f_r_steer_joint", set_steer_pos[1], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_b_l_steer_joint", set_steer_pos[2], 0.1, &steer_joint_trajectory);
  addPosJointTrajectory("wheel_b_r_steer_joint", set_steer_pos[3], 0.1, &steer_joint_trajectory);

  set_wheel_vel[0] = set_wheel_vel[1] = set_wheel_vel[2] = set_wheel_vel[3] = 0.0;
  bool should_publish_steer = false; // Whether to publish steer trajectory

  // Drive State Machine
  switch (drive_state) {
    case DriveState::DRIVE: {
      // Only publish new steer trajectory if setpoints changed meaningfully
      should_publish_steer = false;
      for (int i = 0; i < 4; ++i) {
        if (std::abs(set_steer_pos[i] - last_sent_steer_pos[i]) > SobitProControl::STEER_PUBLISH_EPSILON) {
          should_publish_steer = true;
          break;
        }
      }
        
      // Check if the robot is stuck
      if (all_aligned) {
        set_wheel_vel = sobit_pro_control_->setWheelVel();
        stuck_counter = 0; // Reset the blocked counter

      } else {
        stuck_counter++;
        // If the robot is stuck for too long, apply attenuation to wheel speeds
        // ATTENUATION_FACTOR(sobit_pro_main.hpp): how many cycles to wait before starting to reduce speed
        // MAX_STUCK_CYCLES(sobit_pro_main.hpp): max allowed stuck duration before triggering recovery mode
        if (stuck_counter > ATTENUATION_FACTOR) {
          // Compute attenuation factor (0.0 to 1.0)
          double factor = 1.0 - double(stuck_counter - ATTENUATION_FACTOR) / double(MAX_STUCK_CYCLES - ATTENUATION_FACTOR);
          factor = std::max(factor, 0.1); // Ensure factor is not negative

          // Apply attenuation to wheel velocities
          for (int i = 0; i < 4; ++i) set_wheel_vel[i] *= factor; // Reduce speed based on factor
        }
        // Check if the robot is stuck for too long
        if (stuck_counter >= MAX_STUCK_CYCLES) {
          drive_state = DriveState::RECOVERY; // Switch to recovery state
          stabilize_counter = 0; // Reset stabilize counter
        }
      }
      break;
    }

    case DriveState::RECOVERY: {
      // Force steer alignment
      should_publish_steer = false;
      for (int i = 0; i < 4; ++i) {
        if (std::abs(set_steer_pos[i] - last_sent_steer_pos[i]) > SobitProControl::STEER_PUBLISH_EPSILON) {
          should_publish_steer = true;
          break;
        }
      }

      recovery_publish_counter++;
      // Force steer republishing every N cycles if alignment still not achieved
      if (!should_publish_steer && (recovery_publish_counter % 10) == 0) should_publish_steer = true;
      if (should_publish_steer) recovery_publish_counter = 0; // Reset counter

      // Check if the robot is still stuck
      if (all_aligned){
        stabilize_counter = 0; // Reset the stabilize counter
        drive_state = DriveState::STABILIZE; // Switch to stabilize state
      }
      break;
    }

    case DriveState::STABILIZE: {
      should_publish_steer = true; // Always publish steer trajectory in stabilize state
      stabilize_counter++;

      // Check if stabilization is complete  
      if (stabilize_counter >= 30) { // 1.5 seconds at 50ms cycle
        drive_state = DriveState::DRIVE; // Switch back to drive state
        stuck_counter = 0; // Reset stuck counter
      }
      
      break;
    }
  }

  // Publish Float64MultiArray [rad/s]
  wheel_joint_vel.data.clear();
  wheel_joint_vel.data.push_back(set_wheel_vel[0]);
  wheel_joint_vel.data.push_back(set_wheel_vel[1]);
  wheel_joint_vel.data.push_back(set_wheel_vel[2]);
  wheel_joint_vel.data.push_back(set_wheel_vel[3]);
  pub_wheel_joint_->publish(wheel_joint_vel);

  // Publish new steer command only when necessary:
  // - In DRIVE mode: only if target steer positions changed (avoid redundant publishes)
  // - In RECOVERY/STABILIZE: may be forced at intervals to ensure alignment
  // After publishing, update last_sent_steer_pos to track what was sent
  if (should_publish_steer) {
    pub_steer_joint_->publish(steer_joint_trajectory);
    last_sent_steer_pos = set_steer_pos; // Remember last sent
  }

  // Update the current wheel position
  wheel_fl_curt_pos = joints_pos["wheel_f_l_drive_joint"];
  wheel_fr_curt_pos = joints_pos["wheel_f_r_drive_joint"];
  wheel_bl_curt_pos = joints_pos["wheel_b_l_drive_joint"];
  wheel_br_curt_pos = joints_pos["wheel_b_r_drive_joint"];

  // Calculate Odometry
  result_odom = sobit_pro_odometry_->odom(steer_fl_curt_pos, steer_fr_curt_pos,
                                          steer_bl_curt_pos, steer_br_curt_pos,
                                          wheel_fl_curt_pos, wheel_fr_curt_pos,
                                          wheel_bl_curt_pos, wheel_br_curt_pos,
                                          wheel_fl_prev_pos, wheel_fr_prev_pos,
                                          wheel_bl_prev_pos, wheel_br_prev_pos,
                                          prev_odom);

  // Publish Odometry
  result_odom.header.stamp    = this->get_clock()->now();
  result_odom.header.frame_id = robot_name + "odom";
  result_odom.child_frame_id  = robot_name + "base_footprint";
  sobit_pro_odometry_->pose_broadcaster(result_odom);
  pub_odometry_->publish(result_odom);

  // Update wheel position value for next loop calculation
  wheel_fl_prev_pos = wheel_fl_curt_pos;
  wheel_fr_prev_pos = wheel_fr_curt_pos;
  wheel_bl_prev_pos = wheel_bl_curt_pos;
  wheel_br_prev_pos = wheel_br_curt_pos;

  // Update odom for next loop calculation
  prev_odom = result_odom;
}

} // namespace sobit_pro