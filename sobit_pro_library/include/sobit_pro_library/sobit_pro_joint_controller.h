#ifndef _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_
#define _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sobit_pro_library/sobit_pro_library.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cmath>
#include <cstring>

namespace sobit {

enum Joint { ARM1_1_JOINT = 0, ARM1_2_JOINT, ARM2_1_JOINT, ARM2_2_JOINT, ARM3_JOINT, ARM4_JOINT, GRIPPER_JOINT, HEAD_CAMERA_PAN_JOINT, HEAD_CAMERA_TILT_JOINT, JOINT_NUM };

typedef struct {
  std::string         pose_name;
  std::vector<double> joint_val;
} Pose;

class SobitProJointController : private ROSCommonNode {
 private:
  ros::NodeHandle       nh_;
  ros::NodeHandle       pnh_;
  tf::TransformListener listener_;
  ros::Publisher        pub_arm_joint_;
  ros::Publisher        pub_head_camera_joint_;

  const std::vector<std::string> joint_names_
      = {"arm1_1_joint", "arm1_2_joint", "arm2_1_joint", "arm2_2_joint", "arm3_joint", "arm4_joint", "gripper_joint", "head_camera_pan_joint", "head_camera_tilt_joint"};
  std::vector<Pose> pose_list_;

  static const double arm1_link_length;
  static const double arm2_link_length;
  static const double arm3_link_length;

  void                 setJointTrajectory(const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt);
  void                 addJointTrajectory(const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt);
  void                 checkPublishersConnection(const ros::Publisher& pub);
  void                 loadPose();
  bool                 moveAllJoint(const double arm1,
                                    const double arm2,
                                    const double arm3,
                                    const double arm4,
                                    const double gripper,
                                    const double head_camera_pan,
                                    const double head_camera_tilt,
                                    const double sec,
                                    bool         is_sleep = true);
  geometry_msgs::Point forwardKinematics(double arm1_joint_angle, double arm2_joint_angle, double arm3_joint_angle);
  std::vector<std::vector<double>>  inverseKinematics(double arm2_joint_to_object_x, double arm2_joint_to_object_z, double arm1_joint_angle);

 public:
  SobitProJointController(const std::string& name);
  SobitProJointController();

  bool moveJoint(const Joint joint_num, const double rad, const double sec, bool is_sleep = true);
  bool moveArm(const double arm1, const double arm2, const double arm3, const double arm4);
  bool moveHeadPanTilt(const double head_camera_pan, const double head_camera_tilt, const double sec, bool is_sleep = true);
  bool moveToRegisterdMotion(const std::string& pose_name);
  bool moveGripperToTarget(const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z);
};
}  // namespace sobit

inline void sobit::SobitProJointController::setJointTrajectory(const std::string&                joint_name,
                                                               const double                      rad,
                                                               const double                      sec,
                                                               trajectory_msgs::JointTrajectory* jt) {
  trajectory_msgs::JointTrajectory      joint_trajectory;
  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  joint_trajectory.joint_names.push_back(joint_name);
  joint_trajectory_point.positions.push_back(rad);
  joint_trajectory_point.velocities.push_back(0.0);
  joint_trajectory_point.accelerations.push_back(0.0);
  joint_trajectory_point.effort.push_back(0.0);
  joint_trajectory_point.time_from_start = ros::Duration(sec);
  joint_trajectory.points.push_back(joint_trajectory_point);
  *jt = joint_trajectory;
  return;
}

inline void sobit::SobitProJointController::addJointTrajectory(const std::string&                joint_name,
                                                               const double                      rad,
                                                               const double                      sec,
                                                               trajectory_msgs::JointTrajectory* jt) {
  trajectory_msgs::JointTrajectory joint_trajectory = *jt;
  joint_trajectory.joint_names.push_back(joint_name);
  joint_trajectory.points[0].positions.push_back(rad);
  joint_trajectory.points[0].velocities.push_back(0.0);
  joint_trajectory.points[0].accelerations.push_back(0.0);
  joint_trajectory.points[0].effort.push_back(0.0);
  joint_trajectory.points[0].time_from_start = ros::Duration(sec);
  *jt                                        = joint_trajectory;
  return;
}

inline void sobit::SobitProJointController::checkPublishersConnection(const ros::Publisher& pub) {
  ros::Rate loop_rate(10);
  while (pub.getNumSubscribers() == 0 && ros::ok()) {
    try {
      loop_rate.sleep();
    } catch (const std::exception& ex) {
      break;
    }
  }
  return;
}

#endif /* _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_ */