#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

namespace sobit_pro{

SobitProJointController::SobitProJointController( const std::string& name ) : ROSCommonNode(name), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_){
    pub_arm_joint_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    loadPose();
}

SobitProJointController::SobitProJointController() : ROSCommonNode(), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_){
    pub_arm_joint_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    loadPose();
}

void SobitProJointController::loadPose(){
    XmlRpc::XmlRpcValue pose_param;

    if( !nh_.hasParam("/sobit_pro_pose") ) return;
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
        joint_val[Joint::GRIPPER_JOINT]                   =  static_cast<double>(pose_param[i][joint_names_[Joint::GRIPPER_JOINT]]);
        joint_val[Joint::HEAD_PAN_JOINT]        =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_PAN_JOINT]]);
        joint_val[Joint::HEAD_TILT_JOINT]       =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_TILT_JOINT]]);
        pose.joint_val                                 =  joint_val;

        pose_list_.push_back( pose );
    }

    return;
}

bool SobitProJointController::moveToPose(const std::string& pose_name,
                                         const double sec, bool is_sleep){
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
                             joint_val[Joint::GRIPPER_JOINT],
                             joint_val[Joint::HEAD_PAN_JOINT],
                             joint_val[Joint::HEAD_TILT_JOINT],
                             sec, is_sleep);
    } else{
        ROS_ERROR("Pose '%s' does not exist.", pose_name.c_str());
        return false;
    }
}

bool SobitProJointController::moveAllJoint( const double arm_shoulder_tilt_joint,
                                            const double arm_elbow_upper_tilt_joint,
                                            const double arm_elbow_lower_tilt_joint,
                                            const double arm_elbow_lower_pan_joint,
                                            const double arm_wrist_tilt_joint,
                                            const double gripper_joint,
                                            const double head_pan_joint,
                                            const double head_tilt_joint,
                                            const double sec, bool is_sleep){
    try{
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        trajectory_msgs::JointTrajectory head_joint_trajectory;

        // Set Joint Trajectory for Arm
        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]   ,  arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT]   , -arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],  arm_elbow_upper_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT], -arm_elbow_upper_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT]  ,  arm_elbow_lower_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT]   ,  arm_elbow_lower_pan_joint  , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT]        ,  arm_wrist_tilt_joint       , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::GRIPPER_JOINT]               ,  gripper_joint              , sec, &arm_joint_trajectory );

        // Set Joint Trajectory for Head
        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT]              ,  head_pan_joint             , sec, &head_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT]             ,  head_tilt_joint            , sec, &head_joint_trajectory );

        // Publish Joint Trajectory
        checkPublishersConnection( pub_arm_joint_ );
        checkPublishersConnection( pub_head_joint_ );
        pub_arm_joint_.publish( arm_joint_trajectory );
        pub_head_joint_.publish( head_joint_trajectory );

        if( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveJoint( const Joint joint_num,
                                         const double rad,
                                         const double sec, bool is_sleep ){
    try{
        trajectory_msgs::JointTrajectory joint_trajectory;
      
        // Parallel Joint (avoid lock of opposite joint)
        if( joint_num == ARM_SHOULDER_1_TILT_JOINT || joint_num == ARM_ELBOW_UPPER_1_TILT_JOINT ) {
            setJointTrajectory( joint_names_[joint_num]    ,  rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num + 1], -rad, sec, &joint_trajectory );

        } else if( joint_num == ARM_SHOULDER_2_TILT_JOINT || joint_num == ARM_ELBOW_UPPER_2_TILT_JOINT ) {
            setJointTrajectory( joint_names_[joint_num -1 ],  rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num]    , -rad, sec, &joint_trajectory );

        } else setJointTrajectory( joint_names_[joint_num] ,  rad, sec, &joint_trajectory );


        if( joint_num < Joint::HEAD_PAN_JOINT)  {
            checkPublishersConnection( pub_arm_joint_ );
            pub_arm_joint_.publish( joint_trajectory );
        } else{
            checkPublishersConnection( pub_head_joint_ );
            pub_head_joint_.publish( joint_trajectory );
        }

        if( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveHeadPanTilt( const double head_pan_joint,
                                               const double head_tilt_joint,
                                               const double sec, bool is_sleep ){
    try{
        trajectory_msgs::JointTrajectory joint_trajectory;

        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT] , head_pan_joint , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], head_tilt_joint, sec, &joint_trajectory );

        checkPublishersConnection( pub_head_joint_ );
        pub_head_joint_.publish( joint_trajectory );

        if( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveArm( const double arm_shoulder_tilt_joint,
                                       const double arm_elbow_upper_tilt_joint,
                                       const double arm_elbow_lower_tilt_joint,
                                       const double arm_elbow_lower_pan_joint,
                                       const double arm_wrist,
                                       const double sec, bool is_sleep ){
    try{
        trajectory_msgs::JointTrajectory arm_joint_trajectory;

        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]   ,  arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT]   , -arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],  arm_elbow_upper_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT], -arm_elbow_upper_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT]  ,  arm_elbow_lower_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT]   ,  arm_elbow_lower_pan_joint, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT]        ,  arm_wrist    , sec, &arm_joint_trajectory );

        checkPublishersConnection( pub_arm_joint_ );
        pub_arm_joint_.publish( arm_joint_trajectory );

        if( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

geometry_msgs::Point SobitProJointController::forwardKinematics( const double arm_shoulder_tilt_joint_angle,
                                                                 const double arm_elbow_upper_tilt_joint_angle,
                                                                 const double arm_elbow_lower_tilt_joint_angle ){
    geometry_msgs::Point res_point;

    res_point.x =   ARM_UPPER * cosf(arm_shoulder_tilt_joint_angle)
                  + ARM_INNER * cosf(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle)
                  + ARM_LOWER * cosf(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle + arm_elbow_lower_tilt_joint_angle);
    res_point.y = 0.0;
    res_point.z =   ARM_UPPER * sinf(arm_shoulder_tilt_joint_angle) 
                  + ARM_INNER * sinf(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle)
                  + ARM_LOWER * sinf(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle + arm_elbow_lower_tilt_joint_angle);

    std::cout << "\n=====================================================" << std::endl;
    std::cout << __func__ << std::endl;
    std::cout << "x: " << res_point.x << ", z: " << res_point.z << std::endl;
    std::cout << "======================================================\n" << std::endl;

    return res_point;
}

std::vector<std::vector<double>> SobitProJointController::inverseKinematics( const double arm_elbow_upper_tilt_joint_to_target_x, const double arm_elbow_upper_tilt_joint_to_target_z,
                                                                             const double arm_shoulder_tilt_joint_angle ){
    double diagonal_length     = sqrtf(powf(arm_elbow_upper_tilt_joint_to_target_x, 2.) + powf(arm_elbow_upper_tilt_joint_to_target_z, 2.));
    double diagonal_angle      = atanf(arm_elbow_upper_tilt_joint_to_target_z / arm_elbow_upper_tilt_joint_to_target_x);
    double cos_arm_elbow_upper_tilt_joint_joint = (powf(diagonal_length, 2) + powf(ARM_INNER, 2.) - powf(ARM_LOWER, 2.)) / (2. * diagonal_length * ARM_INNER);

    if     ( cos_arm_elbow_upper_tilt_joint_joint >  1. ) cos_arm_elbow_upper_tilt_joint_joint =  1.;
    else if( cos_arm_elbow_upper_tilt_joint_joint < -1. ) cos_arm_elbow_upper_tilt_joint_joint = -1.;
    
    double arm_elbow_upper_tilt_joint_angle1 = -((arm_shoulder_tilt_joint_angle - diagonal_angle) + acosf(cos_arm_elbow_upper_tilt_joint_joint));
    double arm_elbow_upper_tilt_joint_angle2 = -((arm_shoulder_tilt_joint_angle - diagonal_angle) - acosf(cos_arm_elbow_upper_tilt_joint_joint));

    double external_angle = acosf(cos_arm_elbow_upper_tilt_joint_joint) + acosf(cos_arm_elbow_upper_tilt_joint_joint); // NOTE: 辺の長さが同じため、同じ角度にした
    double arm_elbow_lower_tilt_joint_angle1 =  external_angle;
    double arm_elbow_lower_tilt_joint_angle2 = -external_angle;

    double arm_wrist_tilt_joint_angle1 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle1 + arm_elbow_lower_tilt_joint_angle1);
    double arm_wrist_tilt_joint_angle2 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle2 + arm_elbow_lower_tilt_joint_angle2);

    std::cout << "pair1: (" << arm_shoulder_tilt_joint_angle << ", " << arm_elbow_upper_tilt_joint_angle1 << ", " << arm_elbow_lower_tilt_joint_angle1 << ", " << arm_wrist_tilt_joint_angle1 << ")" << std::endl;
    std::cout << "pair2: (" << arm_shoulder_tilt_joint_angle << ", " << arm_elbow_upper_tilt_joint_angle2 << ", " << arm_elbow_lower_tilt_joint_angle2 << ", " << arm_wrist_tilt_joint_angle2 << ")" << std::endl;

    std::vector<double> result_angles1{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle1, arm_elbow_lower_tilt_joint_angle1, arm_wrist_tilt_joint_angle1};
    std::vector<double> result_angles2{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle2, arm_elbow_lower_tilt_joint_angle2, arm_wrist_tilt_joint_angle2};

    std::vector<std::vector<double>> result_angles_pairs{result_angles1, result_angles2};

    return result_angles_pairs;
}

bool SobitProJointController::moveGripperToTargetCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                                        const double shift_x, const double shift_y, const double shift_z ){
    double arm_to_target_x = target_pos_x + shift_x;
    double arm_to_target_y = target_pos_y + shift_y;
    double arm_to_target_z = target_pos_z + shift_z;

    if ( (arm_to_target_z < -(ARM_LENTGH+ARM_GRIPPER)) || (ARM_LENTGH < arm_to_target_z) ) {
        std::cout << "Arm cannot reach target" << std::endl;
        return false;
    }

    // 先に、arm_shoulder_tilt_joint_jointで目標値の3分の1の高さに調整
    double arm_to_arm_elbow_upper_tilt_joint_x = sqrtf(powf(ARM_UPPER, 2.) - powf(arm_to_target_z / 3., 2.));
    double arm_shoulder_tilt_joint_angle;

    if ( arm_to_target_z < 0 ) arm_shoulder_tilt_joint_angle = -acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);
    else                       arm_shoulder_tilt_joint_angle =  acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);

    double arm_elbow_upper_tilt_joint_to_target_x = arm_to_target_x - arm_to_arm_elbow_upper_tilt_joint_x;
    double arm_elbow_upper_tilt_joint_to_target_y = arm_to_target_y;
    double arm_elbow_upper_tilt_joint_to_target_z = arm_to_target_z - (arm_to_target_z / 3.0);
    //std::cout << "arm_elbow_upper_tilt_joint_to_target_x: " << arm_elbow_upper_tilt_joint_to_target_x << ", arm_elbow_upper_tilt_joint_to_target_z: " << arm_elbow_upper_tilt_joint_to_target_z << std::endl;


    // 車輪の移動量の計算
    double move_wheel_x = 0.0;
    double move_wheel_y = arm_elbow_upper_tilt_joint_to_target_y;

    double diagonal_length = sqrtf(powf(arm_elbow_upper_tilt_joint_to_target_x, 2.) + powf(arm_elbow_upper_tilt_joint_to_target_z, 2));
    //std::cout << "diagonal_length: " << diagonal_length << std::endl;

    // arm_elbow_upper_tilt_joint_to_target_z を基準に移動量を算出
    // diagonal_lengthを30cmに固定して計算
    if ( (ARM_INNER + ARM_LOWER) < diagonal_length || diagonal_length < ARM_INNER * sqrtf(2.) || arm_elbow_upper_tilt_joint_to_target_x <= 0 ) {
        double x     = sqrtf(powf(0.30, 2) - powf(arm_elbow_upper_tilt_joint_to_target_z, 2.));
        move_wheel_x = arm_elbow_upper_tilt_joint_to_target_x - x;
        arm_elbow_upper_tilt_joint_to_target_x = x;
    }

    std::vector<std::vector<double>> result         = inverseKinematics(arm_elbow_upper_tilt_joint_to_target_x, arm_elbow_upper_tilt_joint_to_target_z, arm_shoulder_tilt_joint_angle);
    std::vector<double>              result_angles1 = result.at(0);
    std::vector<double>              result_angles2 = result.at(1);

    /** 順運動学で確認 **/
    geometry_msgs::Point result_pos1 = forwardKinematics(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2));
    geometry_msgs::Point result_pos2 = forwardKinematics(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2));

    /** 車輪で最適な把持位置まで移動 **/
    std::cout << "(move_x, move_y): (" << move_wheel_x << ", " << move_wheel_y << ")" << std::endl;
    sobit_pro::SobitProWheelController wheel_ctr;
    wheel_ctr.controlWheelLinear(move_wheel_x, move_wheel_y);

    /** アームを物体のところまで移動 **/
    std::cout << "Arm position in result_angles1: " << std::endl;
    std::cout << "arm_shoulder_tilt_joint: "    << result_angles1.at(0) << std::endl;
    std::cout << "arm_elbow_upper_tilt_joint: " << result_angles1.at(1) << std::endl;
    std::cout << "arm_elbow_lower_tilt_joint: " << result_angles1.at(2) << std::endl;
    std::cout << "arm_wrist_joint: "            << result_angles1.at(3) << std::endl;

    // std::cout << "Arm position in result_angles2: " << std::endl;
    // std::cout << "arm_shoulder_tilt_joint: "    << result_angles2.at(0) << std::endl;
    // std::cout << "arm_elbow_upper_tilt_joint: " << result_angles2.at(1) << std::endl;
    // std::cout << "arm_elbow_lower_tilt_joint: " << result_angles2.at(2) << std::endl;
    // std::cout << "arm_wrist_joint: "            << result_angles2.at(3) << std::endl;
    
    // [ADD] choose the best angles (result_angles1 or result_angles2)

    /** 床の物体を把持するための判定 **/
    bool is_reached;
    if ( arm_to_target_z < -ARM_LENTGH ) is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3)-1.57); 
    else                                 is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3));

    std::cout << "Target Position: (x, y, z) : (" << target_pos_x << ", " << target_pos_y << ", "<< target_pos_z << ")" << std::endl;
    std::cout << "Result Position: (x, y, z) : (" << result_pos1.x + move_wheel_x << ", " << move_wheel_y << ", " << result_pos1.z << ")" << std::endl;

    return is_reached;
}

bool SobitProJointController::moveGripperToTargetTF( const std::string& target_name,
                                                     const double shift_x, const double shift_y, const double shift_z ){
    geometry_msgs::TransformStamped transformStamped;
    bool is_reached = false;

    try{
        tfBuffer_.canTransform("arm_base_link", target_name, ros::Time(0), ros::Duration(2.0));
        transformStamped = tfBuffer_.lookupTransform ("arm_base_link", target_name, ros::Time(0));
    } catch( tf2::TransformException &ex ){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    auto& tf_target_to_arm = transformStamped.transform.translation;
    is_reached = moveGripperToTargetCoord( tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z,
                                           shift_x, shift_y, shift_z );
    
    return is_reached;
}

bool SobitProJointController::moveGripperToPlaceCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                                       const double shift_x, const double shift_y, const double shift_z ){
    // 作成中
    double target_z         = 0.;

    /** 目標値から0.1[m]程下げた位置までアームを移動 **/
    /**  ハンドに負荷がかかった場合はそこで停止する  **/
    while( -target_z < shift_z ) {
        moveGripperToTargetCoord( target_pos_x, target_pos_y, target_pos_z, 
                                  shift_x, shift_y, shift_z );
        
        // ハンドのジョイントに負荷がかかった場合、そこで停止する
        // [UPD] new graspDecision() with speficic range
        if ( 500 < arm_wrist_tilt_joint_curr_ && arm_wrist_tilt_joint_curr_ < 1000 ) {
            break;
        }

        // 目標値からの差分を追加
        target_z -= 0.05;
    }

    return true;
}

bool SobitProJointController::moveGripperToPlaceTF( const std::string& target_name,
                                                    const double shift_x, const double shift_y, const double shift_z ){
    geometry_msgs::TransformStamped transformStamped;
    bool is_reached = false;

    try{
        tfBuffer_.canTransform("arm_base_link", target_name, ros::Time(0), ros::Duration(2.0));
        transformStamped = tfBuffer_.lookupTransform ("arm_base_link", target_name, ros::Time(0));
    } catch( tf2::TransformException &ex ){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    auto& tf_target_to_arm = transformStamped.transform.translation;
    is_reached = moveGripperToPlaceCoord( tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z,
                                          shift_x, shift_y, shift_z );
    
    return is_reached;
}

// [UPD] new graspDecision() with speficic range
bool SobitProJointController::graspDecision(){
    bool is_grasped = false;

    while ( gripper_joint_curr_ == 0. ) ros::spinOnce();
    
    // ros::spinOnce();
    std::cout << "gripper_joint_curr_ :" << gripper_joint_curr_ << std::endl;

    is_grasped = (300 <= gripper_joint_curr_ && gripper_joint_curr_ <= 1000) ? true : false;

    return is_grasped;
}

void SobitProJointController::callbackCurrArm( const sobits_msgs::current_state_array msg ){
    ros::spinOnce();

    for( const auto actuator : msg.current_state_array ){
        if( actuator.joint_name == joint_names_[ARM_WRIST_TILT_JOINT] ) arm_wrist_tilt_joint_curr_ = actuator.current_ma;
        if( actuator.joint_name == joint_names_[GRIPPER_JOINT] )        gripper_joint_curr_        = actuator.current_ma;
    }
}

}  // namespace sobit_pro