#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

namespace sobit_pro{

// why both required?
SobitProJointController::SobitProJointController(const std::string& name) : ROSCommonNode(name), nh_(), pnh_("~") {
    pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    loadPose();
}

SobitProJointController::SobitProJointController() : ROSCommonNode(), nh_(), pnh_("~") {
    pub_arm_joint_         = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    loadPose();
}

void SobitProJointController::loadPose() {
    XmlRpc::XmlRpcValue pose_param;

    if ( !nh_.hasParam("/sobit_pro_pose") ) return;
    nh_.getParam( "/sobit_pro_pose", pose_param );

    int pose_num = pose_param.size();
    pose_list_.clear();

    for( int i = 0; i < pose_num; i++ ){
        Pose                pose;
        std::vector<double> joint_val(Joint::JOINT_NUM, 0.0);

        pose.pose_name                                 =  static_cast<std::string>(pose_param[i]["pose_name"]);
        joint_val[Joint::ARM_SHOULDER_1_TILT_JOINT]    =  static_cast<double>(pose_param[i][joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]]);
        joint_val[Joint::ARM_SHOULDER_2_TILT_JOINT]    = -static_cast<double>(pose_param[i][joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]]);
        joint_val[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT] =  static_cast<double>(pose_param[i][joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT]]);
        joint_val[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT] = -static_cast<double>(pose_param[i][joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT]]);
        joint_val[Joint::ARM_ELBOW_LOWER_TILT_JOINT]   =  static_cast<double>(pose_param[i][joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT]]);
        joint_val[Joint::ARM_ELBOW_LOWER_PAN_JOINT]    =  static_cast<double>(pose_param[i][joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT]]);
        joint_val[Joint::ARM_WRIST_TILT_JOINT]         =  static_cast<double>(pose_param[i][joint_names_[Joint::ARM_WRIST_TILT_JOINT]]);
        joint_val[Joint::HAND_JOINT]                   =  static_cast<double>(pose_param[i][joint_names_[Joint::HAND_JOINT]]);
        joint_val[Joint::HEAD_CAMERA_PAN_JOINT]        =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_CAMERA_PAN_JOINT]]);
        joint_val[Joint::HEAD_CAMERA_TILT_JOINT]       =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_CAMERA_TILT_JOINT]]);
        pose.joint_val                                 =  joint_val;

        pose_list_.push_back( pose );
    }

    return;
}

bool SobitProJointController::moveToPose(const std::string& pose_name, const double sec, bool is_sleep) {
    bool                is_find = false;
    std::vector<double> joint_val;

    // Check if selected pose exists
    for( auto& pose : pose_list_ ){
        if ( pose_name != pose.pose_name ) continue;
        is_find   = true;
        joint_val = pose.joint_val;
        break;
    }

    if( is_find ){
        ROS_INFO("Pose '%s' was found successfully", pose_name.c_str());
        return moveAllJoint( joint_val[Joint::ARM_SHOULDER_1_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_LOWER_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_LOWER_PAN_JOINT],
                             joint_val[Joint::ARM_WRIST_TILT_JOINT],
                             joint_val[Joint::HAND_JOINT],
                             joint_val[Joint::HEAD_CAMERA_PAN_JOINT],
                             joint_val[Joint::HEAD_CAMERA_TILT_JOINT],
                             sec, is_sleep);
    } else{
        ROS_ERROR("Pose '%s' does not exist.", pose_name.c_str());
        return false;
    }
}

bool SobitProJointController::moveAllJoint( const double arm_upper,
                                            const double arm_inner,
                                            const double arm_lower,
                                            const double arm_lower_pan,
                                            const double arm_wrist,
                                            const double gripper,
                                            const double head_pan,
                                            const double head_tilt,
                                            const double sec,
                                            bool         is_sleep) {
    try{
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        trajectory_msgs::JointTrajectory head_joint_trajectory;

        // Set Joint Trajectory for Arm
        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]   ,  arm_upper     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT]   , -arm_upper     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],  arm_inner     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT], -arm_inner     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT]  ,  arm_lower     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT]   ,  arm_lower_pan , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT]        ,  arm_wrist     , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_JOINT]                  ,  gripper  , sec, &arm_joint_trajectory );

        // Set Joint Trajectory for Head
        setJointTrajectory( joint_names_[Joint::HEAD_CAMERA_PAN_JOINT]       ,  head_pan , sec, &head_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_CAMERA_TILT_JOINT]      ,  head_tilt, sec, &head_joint_trajectory );

        // Publish Joint Trajectory
        checkPublishersConnection( pub_arm_joint_ );
        checkPublishersConnection( pub_head_joint_ );
        pub_arm_joint_.publish( arm_joint_trajectory );
        pub_head_joint_.publish( head_joint_trajectory );

        if ( is_sleep ) ros::Duration( sec ).sleep();

        return true;
    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveJoint( const Joint joint_num, const double rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
      
        // ARM_SHOULDER_1_TILT_JOINT    : joint_num = 0
        // ARM_SHOULDER_2_TILT_JOINT    : joint_num = 1
        // ARM_ELBOW_UPPER_1_TILT_JOINT : joint_num = 2
        // ARM_ELBOW_UPPER_2_TILT_JOINT : joint_num = 3
        if ( joint_num == 0 || joint_num == 2 ) {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num + 1], -rad, sec, &joint_trajectory );
        } else if ( joint_num == 1 || joint_num == 3 ) {
            setJointTrajectory( joint_names_[joint_num -1 ], rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num], -rad, sec, &joint_trajectory );
        } else {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        }

        if ( joint_num < Joint::HEAD_CAMERA_PAN_JOINT)  {
            checkPublishersConnection( pub_arm_joint_ );
            pub_arm_joint_.publish( joint_trajectory );
        } else {
            checkPublishersConnection( pub_head_joint_ );
            pub_head_joint_.publish( joint_trajectory );
        }
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveHeadPanTilt( const double head_pan, const double head_tilt, const double sec, bool is_sleep )  {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::HEAD_CAMERA_PAN_JOINT], head_pan, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_CAMERA_TILT_JOINT], head_tilt, sec, &joint_trajectory );
        checkPublishersConnection( pub_head_joint_ );
        pub_head_joint_.publish( joint_trajectory );
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveArm( const double arm_upper, const double arm_inner, const double arm_lower, const double arm_lower_pan, const double arm_wrist, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT], arm_upper, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT], -arm_upper, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT], arm_inner, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT], -arm_inner, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT], arm_lower, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT], arm_lower_pan, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT], arm_wrist, sec, &arm_joint_trajectory );
        checkPublishersConnection( pub_arm_joint_ );
        pub_arm_joint_.publish( arm_joint_trajectory );
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

geometry_msgs::Point SobitProJointController::forwardKinematics( double arm_upper_joint_angle, double arm_inner_joint_angle, double arm_elbow_lower_tilt_joint_angle ) {
    geometry_msgs::Point res_point;

    res_point.x = ARM_UPPER * std::cos(arm_upper_joint_angle) + ARM_INNER * std::cos(arm_upper_joint_angle + arm_inner_joint_angle)
                  + ARM_LOWER * std::cos(arm_upper_joint_angle + arm_inner_joint_angle + arm_elbow_lower_tilt_joint_angle);
    res_point.z = ARM_UPPER * std::sin(arm_upper_joint_angle) + ARM_INNER * std::sin(arm_upper_joint_angle + arm_inner_joint_angle)
                  + ARM_LOWER * std::sin(arm_upper_joint_angle + arm_inner_joint_angle + arm_elbow_lower_tilt_joint_angle);

    std::cout << "\n=====================================================" << std::endl;
    std::cout << __func__ << std::endl;
    std::cout << "x: " << res_point.x << ", z: " << res_point.z << std::endl;
    std::cout << "======================================================\n" << std::endl;
    return res_point;
}

std::vector<std::vector<double>> SobitProJointController::inverseKinematics( double arm_inner_joint_to_object_x,
                                                                             double arm_inner_joint_to_object_z,
                                                                             double arm_upper_joint_angle ) {
    double diagonal_length = std::sqrt(std::pow(arm_inner_joint_to_object_x, 2) + std::pow(arm_inner_joint_to_object_z, 2));
    double cos_arm_inner_joint
        = (std::pow(diagonal_length, 2) + std::pow(ARM_INNER, 2) - std::pow(ARM_LOWER, 2)) / (2.0 * diagonal_length * ARM_INNER);
    double diagonal_angle    = std::atan(arm_inner_joint_to_object_z / arm_inner_joint_to_object_x);

    if ( cos_arm_inner_joint > 1.0 ) {
        cos_arm_inner_joint = 1.0;
    } else if ( cos_arm_inner_joint < -1.0 ) {
        cos_arm_inner_joint = -1.0;
    }
    double arm_inner_joint_angle1 = -((arm_upper_joint_angle - diagonal_angle) + std::acos(cos_arm_inner_joint));  // XXX: なぜ-を掛けるか分からないが、動く
    double arm_inner_joint_angle2 = -((arm_upper_joint_angle - diagonal_angle) - std::acos(cos_arm_inner_joint));  // XXX: なぜ-を掛けるか分からないが、動く
    double external_angle    = std::acos(cos_arm_inner_joint) + std::acos(cos_arm_inner_joint);               // NOTE: 辺の長さが同じため、同じ角度にした
    double arm_elbow_lower_tilt_joint_angle1 = external_angle;
    double arm_elbow_lower_tilt_joint_angle2 = -external_angle;

    double arm_wrist_tilt_joint_angle1 = -(arm_upper_joint_angle + arm_inner_joint_angle1 + arm_elbow_lower_tilt_joint_angle1);
    double arm_wrist_tilt_joint_angle2 = -(arm_upper_joint_angle + arm_inner_joint_angle2 + arm_elbow_lower_tilt_joint_angle2);
    std::cout << "pair1: (" << arm_upper_joint_angle << ", " << arm_inner_joint_angle1 << ", " << arm_elbow_lower_tilt_joint_angle1 << ", " << arm_wrist_tilt_joint_angle1 << ")" << std::endl;
    std::cout << "pair2: (" << arm_upper_joint_angle << ", " << arm_inner_joint_angle2 << ", " << arm_elbow_lower_tilt_joint_angle2 << ", " << arm_wrist_tilt_joint_angle2 << ")" << std::endl;

    std::vector<double> result_angles1{arm_upper_joint_angle, arm_inner_joint_angle1, arm_elbow_lower_tilt_joint_angle1, arm_wrist_tilt_joint_angle1};
    std::vector<double> result_angles2{arm_upper_joint_angle, arm_inner_joint_angle2, arm_elbow_lower_tilt_joint_angle2, arm_wrist_tilt_joint_angle2};

    std::vector<std::vector<double>> result_angles_pairs{result_angles1, result_angles2};

    return result_angles_pairs;
}

bool SobitProJointController::moveGripperToTargetCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    double arm_to_object_x = goal_position_x + shift.x + diff_goal_position_x;
    double arm_to_object_y = goal_position_y + shift.y + diff_goal_position_y;
    double arm_to_object_z = goal_position_z + shift.z + diff_goal_position_z;

    if ( (arm_to_object_z < -(ARM_LENTGH+ARM_HAND)) || (ARM_LENTGH < arm_to_object_z) ) {
        std::cout << "Armが届きません。" << std::endl;
        return false;
    }

    // 先に、arm_upper_jointで目標値の3分の1の高さに調整
    double arm_to_arm_inner_joint_x = std::sqrt(std::pow(ARM_UPPER, 2) - std::pow(arm_to_object_z / 3.0, 2));
    double arm_upper_joint_angle;
    if ( arm_to_object_z < 0 ) {
        arm_upper_joint_angle = -std::acos(arm_to_arm_inner_joint_x / ARM_UPPER);
    } else {
        arm_upper_joint_angle = std::acos(arm_to_arm_inner_joint_x / ARM_UPPER);
    }

    double arm_inner_joint_to_object_x = arm_to_object_x - arm_to_arm_inner_joint_x;
    double arm_inner_joint_to_object_y = arm_to_object_y;
    double arm_inner_joint_to_object_z = arm_to_object_z - (arm_to_object_z / 3.0);
    //std::cout << "arm_inner_joint_to_object_x: " << arm_inner_joint_to_object_x << ", arm_inner_joint_to_object_z: " << arm_inner_joint_to_object_z << std::endl;

    // 車輪の移動量の計算
    double move_wheel_y = arm_inner_joint_to_object_y;
    // xの移動量は、while文で増減して算出
    // HACK: while文がボトルネックになっているため、計算できるなら最適化したほうが良い
    double       move_wheel_x = 0.0;
    const double step         = 0.01;

    double diagonal_length = std::sqrt(std::pow(arm_inner_joint_to_object_x, 2) + std::pow(arm_inner_joint_to_object_z, 2));
    // arm_inner_joint_to_object_z を基準に移動量を算出
    // diagonal_lengthを30に固定して計算
    //std::cout << "diagonal_length: " << diagonal_length << std::endl;
    if ( (ARM_INNER + ARM_LOWER) < diagonal_length || diagonal_length < ARM_INNER * std::sqrt(2) || arm_inner_joint_to_object_x <= 0 ) {
        double x               = std::sqrt(std::pow(0.30, 2) - std::pow(arm_inner_joint_to_object_z, 2));
        move_wheel_x           = arm_inner_joint_to_object_x - x;
        arm_inner_joint_to_object_x = x;
    }
    /*
    while ((ARM_INNER + ARM_LOWER) < diagonal_length || diagonal_length < ARM_INNER * std::sqrt(2)) {  // armが届かない場合
      if (arm_inner_joint_to_object_x < 0) { // -の場合は無条件でバック
        arm_inner_joint_to_object_x += step;
        move_wheel_x -= step;
      } else if (diagonal_length < ARM_INNER  * std::sqrt(2)) { // +の場合で、目標値が内側で届かない場合は、バック
        arm_inner_joint_to_object_x += step;
        move_wheel_x -= step;
      } else if ((ARM_INNER + ARM_LOWER) < diagonal_length) { // +の場合で、目標値が遠い場合は、直進
        arm_inner_joint_to_object_x -= step;
        move_wheel_x += step;
      } else {
        std::cout << "ERROR" << std::endl;
      }

      // std::cout << "arm_inner_joint_to_object_x: " << arm_inner_joint_to_object_x << std::endl;
      diagonal_length = std::sqrt(std::pow(arm_inner_joint_to_object_x, 2) + std::pow(arm_inner_joint_to_object_z, 2));
      // std::cout << "diagonal_length: " << diagonal_length << std::endl;
    }
    */
    //std::cout << "move_wheel_x: " << move_wheel_x << ", move_whell_y: " << move_wheel_y << std::endl;
    //std::cout << "arm_inner_joint_to_object_x: " << arm_inner_joint_to_object_x << std::endl;

    std::vector<std::vector<double>> result         = inverseKinematics(arm_inner_joint_to_object_x, arm_inner_joint_to_object_z, arm_upper_joint_angle);
    std::vector<double>              result_angles1 = result.at(0);
    std::vector<double>              result_angles2 = result.at(1);

    /** 順運動学で確認 **/
    geometry_msgs::Point result_xz1 = forwardKinematics(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2));
    geometry_msgs::Point result_xz2 = forwardKinematics(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2));

    /** 車輪で最適な把持位置まで移動 **/
    std::cout << "(move_x, move_y): (" << move_wheel_x << ", " << move_wheel_y << ")" << std::endl;
    sobit_pro::SobitProWheelController wheel_ctr;
    wheel_ctr.controlWheelLinear(move_wheel_x, move_wheel_y);

    /** アームを物体のところまで移動 **/
    std::cout << "(joint1, joint2, joint3, joint4): (" << result_angles1.at(0) << ", " << result_angles1.at(1) << ", " << result_angles1.at(2) << ", "
              << result_angles1.at(3) << std::endl;
    //std::cout << "(joint1, joint2, joint3, joint4): (" << result_angles2.at(0) << ", " << result_angles2.at(1) << ", " << result_angles2.at(2) << ", "
    //          << result_angles2.at(3) << std::endl;
    
    /** 床の物体を把持するための判定 **/
    bool is_reached;
    if ( arm_to_object_z < -ARM_LENTGH ) {
        is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3)-1.57); 
    }
    else {
        is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3));
    }
    //moveArm(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2), result_angles2.at(3));

    std::cout << "order : (x, y, z): (" <<goal_position_x << ", " << goal_position_y << ", "<< goal_position_z << ")" << std::endl;
    std::cout << "result: (x, y, z): (" << result_xz1.x + move_wheel_x << ", " << move_wheel_y << ", " << result_xz1.z << ")" << std::endl;

    return is_reached;
}

bool SobitProJointController::moveGripperToTargetTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    //tf::StampedTransform transform_base_to_object;
    tf::StampedTransform transform_arm_to_object;
    try {
        //listener_.waitForTransform("base_footprint", target_name, ros::Time(0), ros::Duration(2.0));
        //listener_.lookupTransform("base_footprint", target_name, ros::Time(0), transform_base_to_object);
        listener_.waitForTransform("arm_base_link", target_name, ros::Time(0), ros::Duration(2.0));
        listener_.lookupTransform("arm_base_link", target_name, ros::Time(0), transform_arm_to_object);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    bool is_reached = moveGripperToTargetCoord( transform_arm_to_object.getOrigin().x(), transform_arm_to_object.getOrigin().y(), transform_arm_to_object.getOrigin().z(),
                              diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
    
    return is_reached;
}

bool SobitProJointController::moveGripperToPlaceCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    // 作成中
    double target_z         = 0.;

    /** 目標値から0.1[m]程下げた位置までアームを移動 **/
    /**  ハンドに負荷がかかった場合はそこで停止する  **/
    while( -target_z < diff_goal_position_z ) {
        moveGripperToTargetCoord( goal_position_x, goal_position_y, goal_position_z, 
                                  diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
        
        // ハンドのジョイントに負荷がかかった場合、そこで停止する
        if ( 500 < arm_wrist_tilt_joint_current_ && arm_wrist_tilt_joint_current_ < 1000 ) {
            break;
        }

        // 目標値からの差分を追加
        target_z -= 0.05;
    }

    return true;
}

bool SobitProJointController::moveGripperToPlaceTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    //debug
    geometry_msgs::Point shift;

    //tf::StampedTransform transform_base_to_object;
    tf::StampedTransform transform_arm_to_object;
    try {
        //listener_.waitForTransform("base_footprint", target_name, ros::Time(0), ros::Duration(2.0));
        //listener_.lookupTransform("base_footprint", target_name, ros::Time(0), transform_base_to_object);
        listener_.waitForTransform("arm_base_link", target_name, ros::Time(0), ros::Duration(2.0));
        listener_.lookupTransform("arm_base_link", target_name, ros::Time(0), transform_arm_to_object);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    moveGripperToPlaceCoord( transform_arm_to_object.getOrigin().x(), transform_arm_to_object.getOrigin().y(), transform_arm_to_object.getOrigin().z(),
                                        diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
    return true;
}

bool SobitProJointController::graspDecision() {
    while ( hand_joint_current_ == 0. ) {
        ros::spinOnce();
    }
    ros::spinOnce();
    std::cout << "hand_joint_current_ :" << hand_joint_current_ << std::endl;
    if ( 300 <= hand_joint_current_ && hand_joint_current_ <= 1000 ) {
        return true;
    } else {
        return false;
    }
}

void SobitProJointController::callbackCurrentStateArray( const sobits_msgs::current_state_array msg ) {
    ros::spinOnce();

    for ( const auto current_state : msg.current_state_array ) {
        if ( current_state.joint_name == "arm_wrist_tilt_joint" ) {
            //std::cout << "\njoint_name:" << current_state.joint_name << std::endl;
            //std::cout << "\njoint_current:" << current_state.current_ma << std::endl;
            arm_wrist_tilt_joint_current_ = current_state.current_ma;
        }

        if ( current_state.joint_name == "hand_joint" ) {
            //std::cout << "\njoint_name:" << current_state.joint_name << std::endl;
            //std::cout << "\njoint_current:" << current_state.current_ma << std::endl;
            hand_joint_current_ = current_state.current_ma;
        }
    }
}

}  // namespace sobit_pro