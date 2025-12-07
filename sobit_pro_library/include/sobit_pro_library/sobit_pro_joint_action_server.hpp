#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_coord.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_tf.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/float64.hpp>
#include <rclcpp/publisher.hpp>



namespace sobit_pro
{

struct PoseParams 
{
  std::string pose_name;
  double arm_shoulder_1_tilt_joint;
  double arm_elbow_upper_1_tilt_joint;
  double arm_elbow_lower_tilt_joint;
  double arm_elbow_lower_pan_joint;
  double arm_wrist_tilt_joint;
  double hand_joint;
  double head_pan_joint;
  double head_tilt_joint;

  // double wheel_f_l_steer_joint;
  // double wheel_f_r_steer_joint;
  // double wheel_b_l_steer_joint;
  // double wheel_b_r_steer_joint;
  // double wheel_f_l_drive_joint;
  // double wheel_f_r_drive_joint;
  // double wheel_b_l_drive_joint;
  // double wheel_b_r_drive_joint;
};

enum JointIds
{
  ARM_SHOULDER_1_TILT_JOINT,
  // ARM_SHOULDER_2_TILT_JOINT,
  ARM_ELBOW_UPPER_1_TILT_JOINT,
  // ARM_ELBOW_UPPER_2_TILT_JOINT,
  ARM_ELBOW_LOWER_TILT_JOINT,
  ARM_ELBOW_LOWER_PAN_JOINT,
  ARM_WRIST_TILT_JOINT,
  HAND_JOINT,
  HEAD_PAN_JOINT,
  HEAD_TILT_JOINT,

  // WHEEL_F_L_STEER_JOINT,
  // WHEEL_F_R_STEER_JOINT,
  // WHEEL_B_L_STEER_JOINT,
  // WHEEL_B_R_STEER_JOINT,
  // WHEEL_F_L_DRIVE_JOINT,
  // WHEEL_F_R_DRIVE_JOINT,
  // WHEEL_B_L_DRIVE_JOINT,
  // WHEEL_B_R_DRIVE_JOINT,

  JOINT_NUM
};

class JointActionServer : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  // using MoveHandToTargetCoord = sobits_interfaces::action::MoveHandToTargetCoord;
  // using MoveHandToTargetTF = sobits_interfaces::action::MoveHandToTargetTF;
  using GetHandToTargetCoord = sobits_interfaces::srv::GetHandToTargetCoord;
  using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;

  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;
  // using GoalHandleMoveHandToCoord = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetCoord>;
  // using GoalHandleMoveHandToTf = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetTF>;


  explicit JointActionServer(const rclcpp::NodeOptions & options);
  ~JointActionServer();

  bool detect_user_open_intent(
      const std::vector<std::string>& joint_names,
      const std::vector<double>& joint_values);

  geometry_msgs::msg::Vector3 get_euler_from_quat(
    const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(
    const geometry_msgs::msg::Vector3& rpy);
  geometry_msgs::msg::TransformStamped forward_kinematics(
    const std::vector<double> &target_joint_rad);  // target_yaw should be eliminated in the future.
  std::vector<double> inverse_kinematics(
    const geometry_msgs::msg::TransformStamped &goal_coord);  // target_yaw should be eliminated in the future.
  trajectory_msgs::msg::JointTrajectory set_joints(
    const std::vector<std::string> &target_joint_names,
    const std::vector<double> &target_joint_rad,
    const builtin_interfaces::msg::Duration &time_allowance);

private:
  const std::vector<std::string> JointNames = {
    "arm_shoulder_1_tilt_joint", 
    // "arm_shoulder_2_tilt_joint",
    "arm_elbow_upper_1_tilt_joint",
    // "arm_elbow_upper_2_tilt_joint",
    "arm_elbow_lower_tilt_joint",
    "arm_elbow_lower_pan_joint",
    "arm_wrist_tilt_joint",
    "hand_joint",
    "head_pan_joint",
    "head_tilt_joint",

    // "wheel_f_l_steer_joint",
    // "wheel_f_r_steer_joint",
    // "wheel_b_l_steer_joint",
    // "wheel_b_r_steer_joint",
    // "wheel_f_l_drive_joint",
    // "wheel_f_r_drive_joint",
    // "wheel_b_l_drive_joint",
    // "wheel_b_r_drive_joint"
  };

  static constexpr double ARM_UPPER = 0.15;
  static constexpr double ARM_INNER = 0.15;
  static constexpr double ARM_LOWER = 0.15;
  static constexpr double ARM_GRIPPER = 0.25;
  static constexpr double ARM_LENTGH = ARM_UPPER+ARM_INNER+ARM_LOWER;

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;

  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_;

  rclcpp_action::GoalResponse handle_move_joints_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_move_to_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void serve_get_hand_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response);
  void serve_get_hand_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request, std::shared_ptr<GetHandToTargetTF::Response> response);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_joint_control_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;


  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hand_goal_raw_;
  double latest_hand_goal_ = std::numeric_limits<double>::quiet_NaN();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::map<std::string, double> initial_joint_state_;


  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
}; // class JointActionServer

inline geometry_msgs::msg::Vector3 JointActionServer::get_euler_from_quat(
  const geometry_msgs::msg::Quaternion& msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;  
}

inline geometry_msgs::msg::Quaternion JointActionServer::get_quat_from_euler(
  const geometry_msgs::msg::Vector3& euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_pro

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_pro::JointActionServer)

