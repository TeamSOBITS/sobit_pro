#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

#include <cmath>

using namespace sobit;

const double sobit::SobitProJointController::arm1_link_length = 0.15;
const double sobit::SobitProJointController::arm2_link_length = 0.15;
const double sobit::SobitProJointController::arm3_link_length = 0.15;

SobitProJointController::SobitProJointController(const std::string& name) : ROSCommonNode(name), nh_(), pnh_("~") {
  pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
  pub_head_camera_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
  loadPose();
}

SobitProJointController::SobitProJointController() : ROSCommonNode(), nh_(), pnh_("~") {
  pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
  pub_head_camera_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
  loadPose();
}

void SobitProJointController::loadPose() {
  XmlRpc::XmlRpcValue pose_val;
  if (!nh_.hasParam("/sobit_pro_pose")) return;
  nh_.getParam("/sobit_pro_pose", pose_val);
  int pose_num = pose_val.size();
  pose_list_.clear();
  for (int i = 0; i < pose_num; i++) {
    Pose                pose;
    std::vector<double> joint_val(9, 0.0);
    pose.pose_name                           = static_cast<std::string>(pose_val[i]["pose_name"]);
    joint_val[Joint::ARM1_1_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::ARM1_1_JOINT]]);
    joint_val[Joint::ARM1_2_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::ARM1_2_JOINT]]);
    joint_val[Joint::ARM2_1_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::ARM2_1_JOINT]]);
    joint_val[Joint::ARM2_2_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::ARM2_2_JOINT]]);
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

bool SobitProJointController::moveAllJoint(const double arm1,
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
    setJointTrajectory(joint_names_[Joint::ARM1_1_JOINT], arm1, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM1_2_JOINT], -arm1, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_1_JOINT], arm2, sec, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_2_JOINT], -arm2, sec, &arm_joint_trajectory);
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

bool SobitProJointController::moveJoint(const Joint joint_num, const double rad, const double sec, bool is_sleep) {
  try {
    trajectory_msgs::JointTrajectory joint_trajectory;
  
    // arm1_1_joint : joint_num = 0
    // arm2_1_joint : joint_num = 2
    if (joint_num == 0 || joint_num == 2 ){
      setJointTrajectory(joint_names_[joint_num], rad, sec, &joint_trajectory);
      addJointTrajectory(joint_names_[joint_num + 1], -rad, sec, &joint_trajectory);
    } else{
      setJointTrajectory(joint_names_[joint_num], rad, sec, &joint_trajectory);
    }

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

bool SobitProJointController::moveHeadPanTilt(const double head_camera_pan, const double head_camera_tilt, const double sec, bool is_sleep) {
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
bool SobitProJointController::moveArm(const double arm1, const double arm2, const double arm3, const double arm4) {
  try {
    trajectory_msgs::JointTrajectory arm_joint_trajectory;
    setJointTrajectory(joint_names_[Joint::ARM1_1_JOINT], arm1, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM1_2_JOINT], -arm1, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_1_JOINT], arm2, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM2_2_JOINT], -arm2, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM3_JOINT], arm3, 5.0, &arm_joint_trajectory);
    addJointTrajectory(joint_names_[Joint::ARM4_JOINT], arm4, 5.0, &arm_joint_trajectory);
    checkPublishersConnection(pub_arm_joint_);
    pub_arm_joint_.publish(arm_joint_trajectory);
    ros::Duration(5.0).sleep();
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SobitProJointController::moveToRegisterdMotion(const std::string& pose_name) {
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
    return moveAllJoint(joint_val[Joint::ARM1_1_JOINT],
                        joint_val[Joint::ARM2_1_JOINT],
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

geometry_msgs::Point SobitProJointController::forwardKinematics(double arm1_joint_angle, double arm2_joint_angle, double arm3_joint_angle) {
  geometry_msgs::Point res_point;

  res_point.x = arm1_link_length * std::cos(arm1_joint_angle) + arm2_link_length * std::cos(arm1_joint_angle + arm2_joint_angle)
                + arm3_link_length * std::cos(arm1_joint_angle + arm2_joint_angle + arm3_joint_angle);
  res_point.z = arm1_link_length * std::sin(arm1_joint_angle) + arm2_link_length * std::sin(arm1_joint_angle + arm2_joint_angle)
                + arm3_link_length * std::sin(arm1_joint_angle + arm2_joint_angle + arm3_joint_angle);

  std::cout << "\n=====================================================" << std::endl;
  std::cout << __func__ << std::endl;
  std::cout << "x: " << res_point.x << ", z: " << res_point.z << std::endl;
  std::cout << "======================================================\n" << std::endl;
  return res_point;
}

std::vector<std::vector<double>> SobitProJointController::inverseKinematics(double arm2_joint_to_object_x,
                                                                            double arm2_joint_to_object_z,
                                                                            double arm1_joint_angle) {
  double diagonal_length = std::sqrt(std::pow(arm2_joint_to_object_x, 2) + std::pow(arm2_joint_to_object_z, 2));
  double cos_arm2_joint
      = (std::pow(diagonal_length, 2) + std::pow(arm2_link_length, 2) - std::pow(arm3_link_length, 2)) / (2.0 * diagonal_length * arm2_link_length);
  double diagonal_angle    = std::atan(arm2_joint_to_object_z / arm2_joint_to_object_x);

  if (cos_arm2_joint > 1.0) {
    cos_arm2_joint = 1.0;
  } else if (cos_arm2_joint < -1.0) {
    cos_arm2_joint = -1.0;
  }
  double arm2_joint_angle1 = -((arm1_joint_angle - diagonal_angle) + std::acos(cos_arm2_joint));  // XXX: なぜ-を掛けるか分からないが、動く
  double arm2_joint_angle2 = -((arm1_joint_angle - diagonal_angle) - std::acos(cos_arm2_joint));  // XXX: なぜ-を掛けるか分からないが、動く
  double external_angle    = std::acos(cos_arm2_joint) + std::acos(cos_arm2_joint);               // NOTE: 辺の長さが同じため、同じ角度にした
  double arm3_joint_angle1 = external_angle;
  double arm3_joint_angle2 = -external_angle;

  double arm4_joint_angle1 = -(arm1_joint_angle + arm2_joint_angle1 + arm3_joint_angle1);
  double arm4_joint_angle2 = -(arm1_joint_angle + arm2_joint_angle2 + arm3_joint_angle2);
  std::cout << "pair1: (" << arm1_joint_angle << ", " << arm2_joint_angle1 << ", " << arm3_joint_angle1 << ", " << arm4_joint_angle1 << ")" << std::endl;
  std::cout << "pair2: (" << arm1_joint_angle << ", " << arm2_joint_angle2 << ", " << arm3_joint_angle2 << ", " << arm4_joint_angle2 << ")" << std::endl;

  std::vector<double> result_angles1{arm1_joint_angle, arm2_joint_angle1, arm3_joint_angle1, arm4_joint_angle1};
  std::vector<double> result_angles2{arm1_joint_angle, arm2_joint_angle2, arm3_joint_angle2, arm4_joint_angle2};

  std::vector<std::vector<double>> result_angles_pairs{result_angles1, result_angles2};

  return result_angles_pairs;
}

bool SobitProJointController::moveGripperToTarget(const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z) {
  geometry_msgs::Point shift;

  tf::StampedTransform transform_base_to_object;
  tf::StampedTransform transform_arm_to_object;
  try {
    listener_.waitForTransform("base_footprint", target_name, ros::Time(0), ros::Duration(2.0));
    listener_.lookupTransform("base_footprint", target_name, ros::Time(0), transform_base_to_object);
    listener_.waitForTransform("arm_fixed_link", target_name, ros::Time(0), ros::Duration(2.0));
    listener_.lookupTransform("arm_fixed_link", target_name, ros::Time(0), transform_arm_to_object);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  double arm_to_object_x = transform_arm_to_object.getOrigin().x() + shift.x + diff_goal_position_x;
  double arm_to_object_y = transform_arm_to_object.getOrigin().y() + shift.y + diff_goal_position_y;
  double arm_to_object_z = transform_arm_to_object.getOrigin().z() + shift.z + diff_goal_position_z;

  double sum_arm123_link_length = arm1_link_length + arm2_link_length + arm3_link_length;
  if ((arm_to_object_z < -(sum_arm123_link_length)) || (sum_arm123_link_length < arm_to_object_z)) {
    std::cout << "Armが届きません。" << std::endl;
    return false;
  }

  // 先に、arm1_jointで目標値の3分の1の高さに調整
  double arm_to_arm2_joint_x = std::sqrt(std::pow(arm1_link_length, 2) - std::pow(arm_to_object_z / 3.0, 2));
  double arm1_joint_angle;
  if (arm_to_object_z < 0) {
    arm1_joint_angle = -std::acos(arm_to_arm2_joint_x / arm1_link_length);
  } else {
    arm1_joint_angle = std::acos(arm_to_arm2_joint_x / arm1_link_length);
  }

  double arm2_joint_to_object_x = arm_to_object_x - arm_to_arm2_joint_x;
  double arm2_joint_to_object_y = arm_to_object_y;
  double arm2_joint_to_object_z = arm_to_object_z - (arm_to_object_z / 3.0);
  std::cout << "arm2_joint_to_object_x: " << arm2_joint_to_object_x << ", arm2_joint_to_object_z: " << arm2_joint_to_object_z << std::endl;

  // 車輪の移動量の計算
  double move_wheel_y = arm2_joint_to_object_y;
  // xの移動量は、while文で増減して算出
  // HACK: while文がボトルネックになっているため、計算できるなら最適化したほうが良い
  double       move_wheel_x = 0.0;
  const double step         = 0.01;

  double diagonal_length = std::sqrt(std::pow(arm2_joint_to_object_x, 2) + std::pow(arm2_joint_to_object_z, 2));
  // arm2_joint_to_object_z を基準に移動量を算出
  // diagonal_lengthを30に固定して計算
  std::cout << "diagonal_length: " << diagonal_length << std::endl;
  if ((arm2_link_length + arm3_link_length) < diagonal_length || diagonal_length < arm2_link_length * std::sqrt(2) || arm2_joint_to_object_x <= 0) {
    double x               = std::sqrt(std::pow(0.30, 2) - std::pow(arm2_joint_to_object_z, 2));
    move_wheel_x           = arm2_joint_to_object_x - x;
    arm2_joint_to_object_x = x;
  }
  /*
  while ((arm2_link_length + arm3_link_length) < diagonal_length || diagonal_length < arm2_link_length * std::sqrt(2)) {  // armが届かない場合
    if (arm2_joint_to_object_x < 0) { // -の場合は無条件でバック
      arm2_joint_to_object_x += step;
      move_wheel_x -= step;
    } else if (diagonal_length < arm2_link_length  * std::sqrt(2)) { // +の場合で、目標値が内側で届かない場合は、バック
      arm2_joint_to_object_x += step;
      move_wheel_x -= step;
    } else if ((arm2_link_length + arm3_link_length) < diagonal_length) { // +の場合で、目標値が遠い場合は、直進
      arm2_joint_to_object_x -= step;
      move_wheel_x += step;
    } else {
      std::cout << "ERROR" << std::endl;
    }

    // std::cout << "arm2_joint_to_object_x: " << arm2_joint_to_object_x << std::endl;
    diagonal_length = std::sqrt(std::pow(arm2_joint_to_object_x, 2) + std::pow(arm2_joint_to_object_z, 2));
    // std::cout << "diagonal_length: " << diagonal_length << std::endl;
  }
  */
  std::cout << "move_wheel_x: " << move_wheel_x << ", move_whell_y: " << move_wheel_y << std::endl;
  std::cout << "arm2_joint_to_object_x: " << arm2_joint_to_object_x << std::endl;

  std::vector<std::vector<double>> result         = inverseKinematics(arm2_joint_to_object_x, arm2_joint_to_object_z, arm1_joint_angle);
  std::vector<double>              result_angles1 = result.at(0);
  std::vector<double>              result_angles2 = result.at(1);

  /** 順運動学で確認 **/
  geometry_msgs::Point result_xz1 = forwardKinematics(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2));
  geometry_msgs::Point result_xz2 = forwardKinematics(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2));

  /** 車輪で最適な把持位置まで移動 **/
  std::cout << "(move_x, move_y): (" << move_wheel_x << ", " << move_wheel_y << ")" << std::endl;
  sobit::SobitProWheelController wheel_ctr;
  wheel_ctr.controlWheelLinear(move_wheel_x, move_wheel_y);

  /** アームを物体のところまで移動 **/
  std::cout << "(joint1, joint2, joint3, joint4): (" << result_angles1.at(0) << ", " << result_angles1.at(1) << ", " << result_angles1.at(2) << ", "
            << result_angles1.at(3) << std::endl;
  std::cout << "(joint1, joint2, joint3, joint4): (" << result_angles2.at(0) << ", " << result_angles2.at(1) << ", " << result_angles2.at(2) << ", "
            << result_angles2.at(3) << std::endl;
  
  moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), result_angles1.at(3));
  //moveArm(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2), result_angles2.at(3));

  std::cout << "order : (x, y, z): (" << transform_arm_to_object.getOrigin().x() << ", " << transform_arm_to_object.getOrigin().y() << ", "
            << transform_arm_to_object.getOrigin().z() << ")" << std::endl;
  std::cout << "result: (x, y, z): (" << result_xz1.x + move_wheel_x << ", " << move_wheel_y << ", " << result_xz1.z << ")" << std::endl;

  return true;
}