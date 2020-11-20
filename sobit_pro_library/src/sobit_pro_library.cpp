#include <sobit_pro_library/sobit_pro_library.h>

using namespace sobit;
namespace py = pybind11;

SobitProLibrary::SobitProLibrary(const std::string& name) : ROSCommonNode(name), nh_(), pnh_("~") {
  pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
  pub_head_camera_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
  loadPose();
}

SobitProLibrary::SobitProLibrary() : ROSCommonNode(), nh_(), pnh_("~") {
  pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
  pub_head_camera_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
  loadPose();
}

void SobitProLibrary::loadPose() {
  XmlRpc::XmlRpcValue pose_val;
  if (!nh_.hasParam("/sobit_pro_pose")) return;
  nh_.getParam("/sobit_pro_pose", pose_val);
  int pose_num = pose_val.size();
  pose_list_.clear();
  for (int i = 0; i < pose_num; i++) {
    Pose                pose;
    std::vector<double> joint_val(7, 0.0);
    pose.pose_name                           = static_cast<std::string>(pose_val[i]["pose_name"]);
    joint_val[Joint::ARM1_JOINT]             = static_cast<double>(pose_val[i][joint_names_[Joint::ARM1_JOINT]]);
    joint_val[Joint::ARM2_JOINT]             = static_cast<double>(pose_val[i][joint_names_[Joint::ARM2_JOINT]]);
    joint_val[Joint::ARM3_JOINT]             = static_cast<double>(pose_val[i][joint_names_[Joint::ARM3_JOINT]]);
    joint_val[Joint::ARM4_JOINT]             = static_cast<double>(pose_val[i][joint_names_[Joint::ARM4_JOINT]]);
    joint_val[Joint::GRIPPER_JOINT]          = static_cast<double>(pose_val[i][joint_names_[Joint::GRIPPER_JOINT]]);
    joint_val[Joint::HEAD_CAMERA_PAN_JOINT]  = static_cast<double>(pose_val[i][joint_names_[Joint::HEAD_CAMERA_PAN_JOINT]]);
    joint_val[Joint::HEAD_CAMERA_TILT_JOINT] = static_cast<double>(pose_val[i][joint_names_[Joint::HEAD_CAMERA_TILT_JOINT]]);
    pose.joint_val                           = joint_val;
    pose_list_.push_back(pose);
  }
  return;
}

bool SobitProLibrary::moveAllJoint(const double arm1,
                                   const double arm2,
                                   const double arm3,
                                   const double arm4,
                                   const double gripper,
                                   const double head_camera_pan,
                                   const double head_camera_tilt,
                                   const double sec,
                                   bool         is_sleep) {
  try {
    trajectory_msgs::JointTrajectory arm_joint_trajectory;
    trajectory_msgs::JointTrajectory head_joint_trajectory;
    setJointTrajectory(joint_names_[Joint::ARM1_JOINT], arm1, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_JOINT], arm2, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM3_JOINT], arm3, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM4_JOINT], arm4, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::GRIPPER_JOINT], gripper, sec, &arm_joint_trajectory);
    setJointTrajectory(joint_names_[Joint::HEAD_CAMERA_PAN_JOINT], head_camera_pan, sec, &head_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::HEAD_CAMERA_TILT_JOINT], head_camera_tilt, sec, &head_joint_trajectory);
    checkPublishersConnection(pub_arm_joint_);
    checkPublishersConnection(pub_head_camera_joint_);
    pub_arm_joint_.publish(arm_joint_trajectory);
    pub_head_camera_joint_.publish(head_joint_trajectory);
    if (is_sleep) {
      ros::Duration(sec).sleep();
    }
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SobitProLibrary::moveJoint(const Joint joint_num, const double rad, const double sec, bool is_sleep) {
  try {
    trajectory_msgs::JointTrajectory joint_trajectory;
    setJointTrajectory(joint_names_[joint_num], rad, sec, &joint_trajectory);
    if (joint_num < Joint::HEAD_CAMERA_PAN_JOINT) {
      checkPublishersConnection(pub_arm_joint_);
      pub_arm_joint_.publish(joint_trajectory);
    } else {
      checkPublishersConnection(pub_head_camera_joint_);
      pub_head_camera_joint_.publish(joint_trajectory);
    }
    if (is_sleep) {
      ros::Duration(sec).sleep();
    }
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SobitProLibrary::moveHeadPanTilt(const double head_camera_pan, const double head_camera_tilt, const double sec, bool is_sleep) {
  try {
    trajectory_msgs::JointTrajectory joint_trajectory;
    setJointTrajectory(joint_names_[Joint::HEAD_CAMERA_PAN_JOINT], head_camera_pan, sec, &joint_trajectory);
    addJointTrajectory(joint_names_[Joint::HEAD_CAMERA_TILT_JOINT], head_camera_tilt, sec, &joint_trajectory);
    checkPublishersConnection(pub_head_camera_joint_);
    pub_head_camera_joint_.publish(joint_trajectory);
    if (is_sleep) {
      ros::Duration(sec).sleep();
    }
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

// TODO: 引数をmoveHeadPanTiltと同じに
bool SobitProLibrary::moveArm(const double arm1, const double arm2, const double arm3, const double arm4, const double gripper) {
  try {
    trajectory_msgs::JointTrajectory arm_joint_trajectory;
    setJointTrajectory(joint_names_[Joint::ARM1_JOINT], arm1, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_JOINT], arm2, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM3_JOINT], arm3, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM4_JOINT], arm4, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::GRIPPER_JOINT], gripper, 5.0, &arm_joint_trajectory);
    checkPublishersConnection(pub_arm_joint_);
    pub_arm_joint_.publish(arm_joint_trajectory);
    ros::Duration(5.0).sleep();
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SobitProLibrary::moveToRegisterdMotion(const std::string& pose_name) {
  bool                is_find = false;
  std::vector<double> joint_val;
  for (auto& pose : pose_list_) {
    if (pose_name != pose.pose_name) continue;
    is_find   = true;
    joint_val = pose.joint_val;
    break;
  }
  if (is_find) {
    ROS_INFO("I found a '%s'", pose_name.c_str());
    return moveAllJoint(joint_val[Joint::ARM1_JOINT],
                        joint_val[Joint::ARM2_JOINT],
                        joint_val[Joint::ARM3_JOINT],
                        joint_val[Joint::ARM4_JOINT],
                        joint_val[Joint::GRIPPER_JOINT],
                        joint_val[Joint::HEAD_CAMERA_PAN_JOINT],
                        joint_val[Joint::HEAD_CAMERA_TILT_JOINT],
                        5.0);
  } else {
    ROS_ERROR("'%s' doesn't exist.", pose_name.c_str());
    return false;
  }
}

PYBIND11_MODULE(sobit_pro_module, m) {
  py::enum_<Joint>(m, "Joint")
      .value("ARM1_JOINT", Joint::ARM1_JOINT)
      .value("ARM2_JOINT", Joint::ARM2_JOINT)
      .value("ARM3_JOINT", Joint::ARM3_JOINT)
      .value("ARM4_JOINT", Joint::ARM4_JOINT)
      .value("GRIPPER_JOINT", Joint::GRIPPER_JOINT)
      .value("HEAD_CAMERA_PAN_JOINT", Joint::HEAD_CAMERA_PAN_JOINT)
      .value("HEAD_CAMERA_TILT_JOINT", Joint::HEAD_CAMERA_TILT_JOINT)
      .value("JOINT_NUM", Joint::JOINT_NUM)
      .export_values();

  py::class_<SobitProLibrary>(m, "SobitProLibrary")
      .def(py::init<const std::string&>())
      .def("moveJoint", &SobitProLibrary::moveJoint, "move Joint", py::arg("joint_num"), py::arg("rad"), py::arg("sec"), py::arg("is_sleep") = true)
      .def("moveHeadPanTilt",
           &SobitProLibrary::moveHeadPanTilt,
           "move Head PanTilt",
           py::arg("head_camera_pan"),
           py::arg("head_camera_tilt"),
           py::arg("sec"),
           py::arg("is_sleep") = true)
      .def("moveArm", &SobitProLibrary::moveArm, "move Arm")
      .def("moveToRegisterdMotion", &SobitProLibrary::moveToRegisterdMotion, "move to Registerd Motion", py::arg("pose_name"));
}