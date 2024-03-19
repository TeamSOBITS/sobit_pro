#ifndef _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_
#define _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_

#include <cmath>
#include <cstring>

#include <ros/ros.h>
#include "sobit_pro_library/sobit_pro_library.h"
#include <tf2_ros/transform_listener.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
// #include <sobits_msgs/current_state.h>
#include <sobits_msgs/current_state_array.h>

// #define ARM_UPPER   0.15
// #define ARM_INNER   0.15
// #define ARM_LOWER   0.15
// #define ARM_GRIPPER 0.25 // Prev 25.0?
// #define ARM_LENTGH  ARM_UPPER+ARM_INNER+ARM_LOWER

namespace sobit_pro{

enum Joint{ ARM_SHOULDER_1_TILT_JOINT = 0,
            ARM_SHOULDER_2_TILT_JOINT,
            ARM_ELBOW_UPPER_1_TILT_JOINT,
            ARM_ELBOW_UPPER_2_TILT_JOINT,
            ARM_ELBOW_LOWER_TILT_JOINT,
            ARM_ELBOW_LOWER_PAN_JOINT,
            ARM_WRIST_TILT_JOINT,
            HAND_JOINT,
            HEAD_PAN_JOINT,
            HEAD_TILT_JOINT,
            JOINT_NUM
           };

typedef struct{
    std::string         pose_name;
    std::vector<double> joint_val;
} Pose;

class SobitProJointController : private ROSCommonNode{
    private:
        ros::NodeHandle   nh_;
        ros::NodeHandle   pnh_;

        ros::Publisher    pub_arm_joint_;
        ros::Publisher    pub_head_joint_;
        ros::Subscriber   sub_curr_arm;
        
        tf2_ros::Buffer            tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::vector<Pose> pose_list_;

        const double ARM_UPPER   = 0.15;
        const double ARM_INNER   = 0.15;
        const double ARM_LOWER   = 0.15;
        const double ARM_GRIPPER = 0.25; // Prev 25.0?
        const double ARM_LENTGH  = ARM_UPPER+ARM_INNER+ARM_LOWER;

        double arm_wrist_tilt_joint_curr_ = 0.;
        double hand_joint_curr_           = 0.;

        const std::vector<std::string> joint_names_ = { "arm_shoulder_1_tilt_joint", 
                                                        "arm_shoulder_2_tilt_joint",
                                                        "arm_elbow_upper_1_tilt_joint",
                                                        "arm_elbow_upper_2_tilt_joint",
                                                        "arm_elbow_lower_tilt_joint",
                                                        "arm_elbow_lower_pan_joint",
                                                        "arm_wrist_tilt_joint",
                                                        "hand_joint",
                                                        "head_pan_joint",
                                                        "head_tilt_joint" };

        void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
        void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
        void checkPublishersConnection( const ros::Publisher& pub );
        void callbackCurrArm( const sobits_msgs::current_state_array& msg );

        geometry_msgs::Point forwardKinematics( const double arm_shoulder_tilt_joint_angle,
                                                const double arm_elbow_upper_tilt_joint_angle,
                                                const double arm_elbow_lower_tilt_joint_angle );
        std::vector<std::vector<double>> inverseKinematics( const double arm_elbow_upper_tilt_joint_to_target_x, const double arm_elbow_upper_tilt_joint_to_target_z,
                                                            const double arm_shoulder_tilt_joint_angle );

        void loadPose();

    public:
        SobitProJointController( const std::string& name );
        SobitProJointController();

        bool moveToPose( const std::string& pose_name,
                         const double sec = 5.0, bool is_sleep = true );
        bool moveAllJoint( const double arm_shoulder_tilt_joint,
                           const double arm_elbow_upper_tilt_joint,
                           const double arm_elbow_lower_tilt_joint,
                           const double arm_elbow_lower_pan_joint,
                           const double arm_wrist_tilt_joint,
                           const double hand_joint,
                           const double head_pan_joint,
                           const double head_tilt_joint,
                           const double sec = 5.0, bool is_sleep = true );
        bool moveJoint( const Joint joint_num,
                        const double rad,
                        const double sec = 5.0, bool is_sleep = true );
        bool moveArm( const double arm_shoulder_tilt_joint,
                      const double arm_elbow_upper_tilt_joint,
                      const double arm_elbow_lower_tilt_joint,
                      const double arm_elbow_lower_pan_joint,
                      const double arm_wrist_tilt_joint,
                      const double sec = 5.0, bool is_sleep = true );
        bool moveHeadPanTilt( const double head_pan_joint,
                              const double head_tilt_joint,
                              const double sec = 5.0, bool is_sleep = true );
        bool moveHandToTargetCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                    const double shift_x     , const double shift_y     , const double shift_z,
                                    const double sec = 5.0, bool is_sleep = true );
        bool moveHandToTargetTF( const std::string& target_name,
                                 const double shift_x, const double shift_y, const double shift_z,
                                 const double sec = 5.0, bool is_sleep = true );
        bool moveHandToPlaceCoord( const double target_pos_x, const double target_pos_y, const double target_pos_z,
                                   const double shift_x     , const double shift_y     , const double shift_z,
                                   const double sec = 5.0, bool is_sleep = true );
        bool moveHandToPlaceTF( const std::string& target_name,
                                const double shift_x, const double shift_y, const double shift_z,
                                const double sec = 5.0, bool is_sleep = true );
        bool graspDecision( const int min_curr = 300, const int max_curr = 1000 );
        bool placeDecision( const int min_curr = 500, const int max_curr = 1000 );

};

} // namespace sobit_pro

inline void sobit_pro::SobitProJointController::setJointTrajectory( const std::string& joint_name,
                                                                    const double       rad,
                                                                    const double       sec,
                                                                    trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory      joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;

    joint_trajectory.joint_names.push_back( joint_name );
    joint_trajectory_point.positions.push_back( rad );
    // joint_trajectory_point.velocities.push_back( 0.0 );
    // joint_trajectory_point.accelerations.push_back( 0.0 );
    // joint_trajectory_point.effort.push_back( 0.0 );
    joint_trajectory_point.time_from_start = ros::Duration( sec );
    joint_trajectory.points.push_back( joint_trajectory_point );

    *jt = joint_trajectory;

    return;
}

inline void sobit_pro::SobitProJointController::addJointTrajectory( const std::string& joint_name,
                                                                    const double        rad,
                                                                    const double        sec,
                                                                    trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory = *jt;

    joint_trajectory.joint_names.push_back( joint_name );
    joint_trajectory.points[0].positions.push_back( rad );
    // joint_trajectory.points[0].velocities.push_back( 0.0 );
    // joint_trajectory.points[0].accelerations.push_back( 0.0 );
    // joint_trajectory.points[0].effort.push_back( 0.0 );
    joint_trajectory.points[0].time_from_start = ros::Duration( sec );

    *jt = joint_trajectory;

    return;
}

inline void sobit_pro::SobitProJointController::checkPublishersConnection( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );

    while( pub.getNumSubscribers( ) == 0 && ros::ok( ) ){
        try{ loop_rate.sleep(); }
        catch( const std::exception& ex ){ break; }
    }

    return;
}

// Check!
inline void sobit_pro::SobitProJointController::callbackCurrArm( const sobits_msgs::current_state_array& msg ){
    // ros::spinOnce();

    for( const auto actuator : msg.current_state_array ){
        if( actuator.joint_name == joint_names_[ARM_WRIST_TILT_JOINT] ) arm_wrist_tilt_joint_curr_ = actuator.current_ma;
        if( actuator.joint_name == joint_names_[HAND_JOINT] )           hand_joint_curr_           = actuator.current_ma;
    }
}

#endif /* _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_ */
