#include "sobit_pro_library/sobit_pro_joint_controller.h"
#include "sobit_pro_library/sobit_pro_wheel_controller.hpp"

using namespace sobit_pro;

SobitProJointController::SobitProJointController( const std::string& name ) : ROSCommonNode(name), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_){
    pub_arm_joint_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    
    sub_curr_arm    = nh_.subscribe( "/current_state_array", 1, &SobitProJointController::callbackCurrArm, this );

    loadPose();
}

SobitProJointController::SobitProJointController() : ROSCommonNode(), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_){
    pub_arm_joint_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_head_joint_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);

    sub_curr_arm    = nh_.subscribe( "/current_state_array", 1, &SobitProJointController::callbackCurrArm, this );

    loadPose();
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

    std::cout << std::endl;
    std::cout << __func__ << "results:" << std::endl;
    std::cout << "(x,z) = (" << res_point.x << ", " << res_point.z << ")" << std::endl;
    std::cout << std::endl;

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

    // As the lenght of other links are similar, we set the angle the same
    double external_angle = acosf(cos_arm_elbow_upper_tilt_joint_joint) + acosf(cos_arm_elbow_upper_tilt_joint_joint);
    double arm_elbow_lower_tilt_joint_angle1 =  external_angle;
    double arm_elbow_lower_tilt_joint_angle2 = -external_angle;

    double arm_wrist_tilt_joint_angle1 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle1 + arm_elbow_lower_tilt_joint_angle1);
    double arm_wrist_tilt_joint_angle2 = -(arm_shoulder_tilt_joint_angle + arm_elbow_upper_tilt_joint_angle2 + arm_elbow_lower_tilt_joint_angle2);

    std::cout << "inverseKinematics() Result 1: (" << arm_shoulder_tilt_joint_angle << ", " << arm_elbow_upper_tilt_joint_angle1 << ", " << arm_elbow_lower_tilt_joint_angle1 << ", " << arm_wrist_tilt_joint_angle1 << ")" << std::endl;
    std::cout << "inverseKinematics() Result 2: (" << arm_shoulder_tilt_joint_angle << ", " << arm_elbow_upper_tilt_joint_angle2 << ", " << arm_elbow_lower_tilt_joint_angle2 << ", " << arm_wrist_tilt_joint_angle2 << ")" << std::endl;

    std::vector<double> result_angles1{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle1, arm_elbow_lower_tilt_joint_angle1, arm_wrist_tilt_joint_angle1};
    std::vector<double> result_angles2{arm_shoulder_tilt_joint_angle, arm_elbow_upper_tilt_joint_angle2, arm_elbow_lower_tilt_joint_angle2, arm_wrist_tilt_joint_angle2};

    std::vector<std::vector<double>> result_angles{result_angles1, result_angles2};

    return result_angles;
}


void SobitProJointController::loadPose(){
    XmlRpc::XmlRpcValue pose_param;

    // Check if pose parameter YAML file was loaded
    if( !nh_.hasParam("/sobit_pro_pose") ) return;
    nh_.getParam( "/sobit_pro_pose", pose_param );

    int pose_num = pose_param.size();
    pose_list_.clear();

    // Save the poses
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
        joint_val[Joint::HEAD_PAN_JOINT]               =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_PAN_JOINT]]);
        joint_val[Joint::HEAD_TILT_JOINT]              =  static_cast<double>(pose_param[i][joint_names_[Joint::HEAD_TILT_JOINT]]);

        pose.joint_val                                 =  joint_val;

        pose_list_.push_back( pose );
    }

    return;
}

bool SobitProJointController::moveToPose( const std::string& pose_name,
                                          const double sec, bool is_sleep ){
    std::vector<double> joint_val;
    bool is_found = false;

    // Check if selected pose exists
    for( auto& pose : pose_list_ ){
        if ( pose.pose_name != pose_name ) continue;

        joint_val = pose.joint_val;
        is_found  = true;

        break;
    }

    // Set the pose
    if( is_found ){
        ROS_INFO("Pose '%s' was found successfully", pose_name.c_str());
        return moveAllJoint( joint_val[Joint::ARM_SHOULDER_1_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_LOWER_TILT_JOINT],
                             joint_val[Joint::ARM_ELBOW_LOWER_PAN_JOINT],
                             joint_val[Joint::ARM_WRIST_TILT_JOINT],
                             joint_val[Joint::HAND_JOINT],
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
                                            const double hand_joint,
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
        addJointTrajectory( joint_names_[Joint::HAND_JOINT]                  ,  hand_joint                 , sec, &arm_joint_trajectory );

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
        if( joint_num == ARM_SHOULDER_1_TILT_JOINT || joint_num == ARM_ELBOW_UPPER_1_TILT_JOINT ){
            setJointTrajectory( joint_names_[joint_num]    ,  rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num + 1], -rad, sec, &joint_trajectory );

        } else if( joint_num == ARM_SHOULDER_2_TILT_JOINT || joint_num == ARM_ELBOW_UPPER_2_TILT_JOINT ){
            setJointTrajectory( joint_names_[joint_num -1 ],  rad, sec, &joint_trajectory );
            addJointTrajectory( joint_names_[joint_num]    , -rad, sec, &joint_trajectory );

        } else setJointTrajectory( joint_names_[joint_num] ,  rad, sec, &joint_trajectory );


        // Select the proper publisher
        if( joint_num < Joint::HEAD_PAN_JOINT){
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
                                       const double arm_wrist_tilt_joint,
                                       const double sec, bool is_sleep ){
    try{
        trajectory_msgs::JointTrajectory arm_joint_trajectory;

        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT]   ,  arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT]   , -arm_shoulder_tilt_joint    , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_1_TILT_JOINT],  arm_elbow_upper_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_UPPER_2_TILT_JOINT], -arm_elbow_upper_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_TILT_JOINT]  ,  arm_elbow_lower_tilt_joint , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_LOWER_PAN_JOINT]   ,  arm_elbow_lower_pan_joint  , sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT]        ,  arm_wrist_tilt_joint       , sec, &arm_joint_trajectory );

        checkPublishersConnection( pub_arm_joint_ );
        pub_arm_joint_.publish( arm_joint_trajectory );

        if( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProJointController::moveHandToTargetCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                                     const double shift_x     , const double shift_y     , const double shift_z,
                                                     const double sec, bool is_sleep ){
    double arm_to_target_x = target_pos_x + shift_x;
    double arm_to_target_y = target_pos_y + shift_y;
    double arm_to_target_z = target_pos_z + shift_z;
    bool   is_reached      = false;

    // Check if the arm can reach the target
    if( (arm_to_target_z < -(ARM_LENTGH+ARM_GRIPPER)) || (ARM_LENTGH < arm_to_target_z) ){
        std::cout << "Arm cannot reach the selected target" << std::endl;
        std::cout << "Target (x,y,z) = (" << arm_to_target_x << ", " << arm_to_target_y << ", " << arm_to_target_z << ")" << std::endl;
        std::cout << "ARM_LENTGH: " << ARM_LENTGH << std::endl;
        std::cout << "ARM_LENTGH+ARM_GRIPPER: " << ARM_LENTGH+ARM_GRIPPER << std::endl;
        
        return is_reached;
    }

    // Arm_shoulder_tilt_joint is set based on arm_to_target_z 
    double arm_to_arm_elbow_upper_tilt_joint_x = sqrtf(powf(ARM_UPPER, 2.) - powf(arm_to_target_z / 3., 2.));
    double arm_shoulder_tilt_joint_angle;

    if ( arm_to_target_z < 0 ) arm_shoulder_tilt_joint_angle = -acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);
    else                       arm_shoulder_tilt_joint_angle =  acosf(arm_to_arm_elbow_upper_tilt_joint_x / ARM_UPPER);

    double arm_elbow_upper_tilt_joint_to_target_x = arm_to_target_x - arm_to_arm_elbow_upper_tilt_joint_x;
    double arm_elbow_upper_tilt_joint_to_target_y = arm_to_target_y;
    double arm_elbow_upper_tilt_joint_to_target_z = arm_to_target_z - (arm_to_target_z / 3.0);
    //std::cout << "arm_elbow_upper_tilt_joint_to_target_x: " << arm_elbow_upper_tilt_joint_to_target_x << ", arm_elbow_upper_tilt_joint_to_target_z: " << arm_elbow_upper_tilt_joint_to_target_z << std::endl;


    // Calculate the distance to move the wheels
    double move_wheel_x = 0.0;
    double move_wheel_y = arm_elbow_upper_tilt_joint_to_target_y;

    double diagonal_length = sqrtf(powf(arm_elbow_upper_tilt_joint_to_target_x, 2.) + powf(arm_elbow_upper_tilt_joint_to_target_z, 2));
    //std::cout << "diagonal_length: " << diagonal_length << std::endl;

    // The point reference is arm_elbow_upper_tilt_joint_to_target_z
    // diagonal_length is fixed to 30cm to calculate the distance to move the wheels
    if ( (ARM_INNER + ARM_LOWER) < diagonal_length || diagonal_length < ARM_INNER * sqrtf(2.) || arm_elbow_upper_tilt_joint_to_target_x <= 0 ) {
        double x     = sqrtf(powf(0.30, 2) - powf(arm_elbow_upper_tilt_joint_to_target_z, 2.));
        move_wheel_x = arm_elbow_upper_tilt_joint_to_target_x - x;
        arm_elbow_upper_tilt_joint_to_target_x = x;
    }

    std::vector<std::vector<double>> results        = inverseKinematics(arm_elbow_upper_tilt_joint_to_target_x, arm_elbow_upper_tilt_joint_to_target_z, arm_shoulder_tilt_joint_angle);
    std::vector<double>              result_angles1 = results.at(0);
    std::vector<double>              result_angles2 = results.at(1);

    // Check the results with forwardKinematics()
    geometry_msgs::Point result_pos1 = forwardKinematics(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2));
    geometry_msgs::Point result_pos2 = forwardKinematics(result_angles2.at(0), result_angles2.at(1), result_angles2.at(2));

    // Move the wheels to the calculated distance
    sobit_pro::SobitProWheelController wheel_ctr;

    std::cout << "(move_x, move_y): (" << move_wheel_x << ", " << move_wheel_y << ")" << std::endl;
    wheel_ctr.controlWheelLinear(move_wheel_x, move_wheel_y);

    // Move the arm to the calculated angles
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
    
    // TODO: choose the best angles (result_angles1 or result_angles2)

    // If the target is below the arm, the wrist is rotated 180 degrees
    if ( arm_to_target_z < -ARM_LENTGH ) is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3)-1.57); 
    else                                 is_reached = moveArm(result_angles1.at(0), result_angles1.at(1), result_angles1.at(2), 0.0, result_angles1.at(3));

    std::cout << "Target Position: (x, y, z) : (" << target_pos_x << ", " << target_pos_y << ", "<< target_pos_z << ")" << std::endl;
    std::cout << "Result Position: (x, y, z) : (" << result_pos1.x + move_wheel_x << ", " << move_wheel_y << ", " << result_pos1.z << ")" << std::endl;

    return is_reached;
}

bool SobitProJointController::moveHandToTargetTF( const std::string& target_name,
                                                  const double shift_x, const double shift_y, const double shift_z,
                                                  const double sec, bool is_sleep ){
    geometry_msgs::TransformStamped transformStamped;
    bool is_reached = false;

    try{
        tfBuffer_.canTransform("arm_base_link", target_name, ros::Time(0), ros::Duration(0.5));
        transformStamped = tfBuffer_.lookupTransform ("arm_base_link", target_name, ros::Time(0));
    } catch( tf2::TransformException &ex ){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    auto& tf_target_to_arm = transformStamped.transform.translation;
    is_reached = moveHandToTargetCoord( tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z,
                                           shift_x, shift_y, shift_z );
    
    return is_reached;
}

// Check!!
bool SobitProJointController::moveHandToPlaceCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                                    const double shift_x     , const double shift_y     , const double shift_z,
                                                    const double sec, bool is_sleep ){
    double target_z    = 0.;
    bool   is_reached = false;

    // Reduce the target_pos_z by 0.05[m], until expected collision is detected
    while( !(is_reached && placeDecision(500, 1000)) ) {
        is_reached = moveHandToTargetCoord( target_pos_x, target_pos_y, target_pos_z, 
                                               shift_x, shift_y, shift_z+target_z );

        if( !is_reached ) return is_reached;

        // Accumulate the target_z
        target_z -= 0.05;
    }

    return is_reached;
}

bool SobitProJointController::moveHandToPlaceTF( const std::string& target_name,
                                                 const double shift_x, const double shift_y, const double shift_z,
                                                 const double sec, bool is_sleep ){
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
    is_reached = moveHandToPlaceCoord( tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z,
                                          shift_x, shift_y, shift_z );
    
    return is_reached;
}

bool SobitProJointController::graspDecision( const int min_curr, const int max_curr ){
    bool is_grasped = false;

    // Spin until the current value is obtained
    while( hand_joint_curr_ == 0. ) ros::spinOnce();

    // ros::spinOnce();
    std::cout << "hand_joint_curr_ = " << hand_joint_curr_ << std::endl;

    is_grasped = (min_curr <= hand_joint_curr_ && hand_joint_curr_ <= max_curr) ? true : false;

    return is_grasped;
}

bool SobitProJointController::placeDecision( const int min_curr, const int max_curr ){
    bool is_placed = false;

    // Spin until the current value is obtained
    while( arm_wrist_tilt_joint_curr_ == 0. ) ros::spinOnce();

    // ros::spinOnce();
    std::cout << "arm_wrist_tilt_joint_curr_ = " << arm_wrist_tilt_joint_curr_ << std::endl;

    is_placed = (min_curr <= arm_wrist_tilt_joint_curr_ && arm_wrist_tilt_joint_curr_ <= max_curr) ? true : false;

    return is_placed;
}