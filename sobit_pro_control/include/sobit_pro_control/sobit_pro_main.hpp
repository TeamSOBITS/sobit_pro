#include <iostream>
#include <random>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

namespace sobit_pro
{
class SobitProMain : public rclcpp::Node
{
public:
  explicit SobitProMain(const rclcpp::NodeOptions & options);
  ~SobitProMain();

  bool start_up_sound();
  bool shut_down_sound();
  void control_wheel();

private:    
  // ROS2 I/F
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_info_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               pub_odometry_;
          rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_steer_joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      pub_wheel_joint_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   pub_wheels_error_;

  // ==== Control & Sensing Callbacks ====
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void control_callback();

  // ==== Publishing Helpers ====
  double getJointPos(const std::string& joint_name);
  double getJointVel(const std::string& joint_name);
  void setPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt);
  void addPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt);
  bool checkPublishersConnection(std::string pub_name);

  // ==== Control Variables ====
  trajectory_msgs::msg::JointTrajectory steer_joint_trajectory;
  std_msgs::msg::Float64MultiArray      wheel_joint_vel;

  // Wheel positions (initial and current)
  double wheel_fl_init_pos, wheel_fr_init_pos, wheel_bl_init_pos, wheel_br_init_pos;
  double wheel_fl_curt_pos, wheel_fr_curt_pos, wheel_bl_curt_pos, wheel_br_curt_pos;

  // Steering joint positions (current)
  double steer_fl_curt_pos, steer_fr_curt_pos, steer_bl_curt_pos, steer_br_curt_pos;

  // Desired (target) steering positions and wheel velocities
  std::array<double, 4> set_steer_pos;
  std::array<double, 4> set_wheel_vel;

  bool is_steer_movable;

  int32_t motion;
  int32_t prev_motion = -1;

  std::map<std::string, double> joints_pos;
  std::map<std::string, double> joints_vel;

  nav_msgs::msg::Odometry result_odom;
  nav_msgs::msg::Odometry prev_odom;

  std_msgs::msg::Bool wheels_error;
  geometry_msgs::msg::Twist curt_vel_twist;

  std::unique_ptr<SobitProControl> sobit_pro_control_;
  std::unique_ptr<SobitProOdometry> sobit_pro_odometry_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // ==== State Machine for Safe Swerve Control ====
  // Used by control_callback() to manage motion safety:
  // Defines the robot's current drive state:
enum class DriveState { 
  DRIVE,       // Normal driving mode: move using wheel and steer commands
  RECOVERY,    // Robot detected as stuck — stop wheels, force steer alignment
  STABILIZE    // Post-recovery cooldown phase — ensure robot is stable before resuming
};
  DriveState drive_state = DriveState::RECOVERY; // Current drive state (initialized in RECOVERY to ensure stability on startup)
  // DriveState prev_drive_state = DriveState::RECOVERY; // Previous drive state (used to detect state transitions and for logging)

  int stuck_counter = 0;        // Counts consecutive control cycles where the robot remains unaligned (used to detect if robot is stuck)
  int stabilize_counter = 0;    // Counter for STABILIZE phase — delays transition back to DRIVE to allow full recovery
  int MAX_STUCK_CYCLES = 40;    // Max number of cycles before triggering recovery mode (≈2s if control loop is 50ms) //TO DO: make static constexpr
  static constexpr int ATTENUATION_FACTOR = 10;  // Number of cycles before attenuation of wheel speed begins during misalignment //TO DO: make static constexpr
  static int recovery_publish_counter; // Counter to occasionally force republishing steer trajectory during RECOVERY
};
    
// 実装部
inline double SobitProMain::getJointPos(const std::string& joint_name)
{
  return joints_pos[joint_name];
}

inline double SobitProMain::getJointVel(const std::string& joint_name)
{
  return joints_vel[joint_name];
}

inline void SobitProMain::setPosJointTrajectory(
  const std::string& joint_name,
  double rad, double sec,
  trajectory_msgs::msg::JointTrajectory* jt)
{
  trajectory_msgs::msg::JointTrajectory      joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;

  joint_trajectory.joint_names.push_back( joint_name );
  joint_trajectory_point.positions.push_back( rad );
  // joint_trajectory_point.velocities.push_back( 0.0 );
  // joint_trajectory_point.accelerations.push_back( 0.0 );
  // joint_trajectory_point.effort.push_back( 0.0 );
  joint_trajectory_point.time_from_start = rclcpp::Duration::from_seconds(sec);
  joint_trajectory.points.push_back( joint_trajectory_point );

  *jt = joint_trajectory;
}

inline void SobitProMain::addPosJointTrajectory(
  const std::string& joint_name,
  double rad, double sec,
  trajectory_msgs::msg::JointTrajectory* jt)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory = *jt;

  joint_trajectory.joint_names.push_back( joint_name );
  joint_trajectory.points[0].positions.push_back( rad );
  // joint_trajectory.points[0].velocities.push_back( 0.0 );
  // joint_trajectory.points[0].accelerations.push_back( 0.0 );
  // joint_trajectory.points[0].effort.push_back( 0.0 );
  joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(sec);

  *jt = joint_trajectory;
}

inline bool SobitProMain::checkPublishersConnection(std::string pub_name) {
  RCLCPP_DEBUG(this->get_logger(), "Number of publishers: %ld", this->count_publishers(pub_name));
  return this->count_publishers(pub_name) > 0 ? true : false;
}

} // namespace sobit_pro

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_pro::SobitProMain)