#ifndef _SOBIT_PRO_LIBRARY_H_
#define _SOBIT_PRO_LIBRARY_H_

#include <pybind11/pybind11.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cmath>
#include <cstring>

class ROSCommonNode {
 public:
  ROSCommonNode(const std::string& name) {
    char* cstr = new char[name.size() + 1];
    std::strcpy(cstr, name.c_str());
    char** argv = &cstr;
    int    argc = 0;
    delete[] cstr;
    ros::init(argc, argv, "sobit_pro_library_node");
  }
  ROSCommonNode() {}
};

namespace sobit {

enum Joint { ARM1_JOINT = 0, ARM2_JOINT, ARM3_JOINT, ARM4_JOINT, GRIPPER_JOINT, HEAD_CAMERA_PAN_JOINT, HEAD_CAMERA_TILT_JOINT, JOINT_NUM };

typedef struct {
  std::string         pose_name;
  std::vector<double> joint_val;
} Pose;

class SobitProLibrary : private ROSCommonNode {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  void            setJointTrajectory(const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt);
  void            addJointTrajectory(const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt);
  void            checkPublishersConnection(const ros::Publisher& pub);

  ros::Publisher                 pub_arm_joint_;
  ros::Publisher                 pub_head_camera_joint_;
  const std::vector<std::string> joint_names_
      = {"arm1_joint", "arm2_joint", "arm3_joint", "arm4_joint", "gripper_joint", "head_camera_pan_joint", "head_camera_tilt_joint"};
  std::vector<Pose> pose_list_;
  void              loadPose();
  bool              moveAllJoint(const double arm1,
                                 const double arm2,
                                 const double arm3,
                                 const double arm4,
                                 const double gripper,
                                 const double head_camera_pan,
                                 const double head_camera_tilt,
                                 const double sec,
                                 bool         is_sleep = true);

 public:
  SobitProLibrary(const std::string& name);
  SobitProLibrary();

  bool moveJoint(const Joint joint_num, const double rad, const double sec, bool is_sleep = true);
  bool moveArm(const double arm1, const double arm2, const double arm3, const double arm4, const double gripper);
  bool moveHeadPanTilt(const double head_camera_pan, const double head_camera_tilt, const double sec, bool is_sleep = true);
  bool moveToRegisterdMotion(const std::string& pose_name);
  bool moveGripperToTarget();
};
}  // namespace sobit

inline void sobit::SobitProLibrary::setJointTrajectory(const std::string&                joint_name,
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

inline void sobit::SobitProLibrary::addJointTrajectory(const std::string&                joint_name,
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

inline void sobit::SobitProLibrary::checkPublishersConnection(const ros::Publisher& pub) {
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


#endif /* _SOBIT_PRO_LIBRARY_ */