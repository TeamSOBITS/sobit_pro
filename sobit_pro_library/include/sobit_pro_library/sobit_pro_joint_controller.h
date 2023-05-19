#ifndef _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_
#define _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sobit_pro_library/sobit_pro_library.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sobit_common_msg/current_state.h>
#include <sobit_common_msg/current_state_array.h>

#include <cmath>
#include <cstring>

namespace sobit_pro {
    enum Joint { ARM_SHOULDER_1_TILT_JOINT = 0,
                 ARM_SHOULDER_2_TILT_JOINT,
                 ARM_ELBOW_UPPER_1_TILT_JOINT,
                 ARM_ELBOW_UPPER_2_TILT_JOINT,
                 ARM_ELBOW_LOWER_TILT_JOINT,
                 ARM_ELBOW_LOWER_PAN_JOINT,
                 ARM_WRIST_TILT_JOINT,
                 HAND_JOINT,
                 HEAD_CAMERA_PAN_JOINT,
                 HEAD_CAMERA_TILT_JOINT,
                 JOINT_NUM };

    typedef struct {
    std::string         pose_name;
    std::vector<double> joint_val;
    } Pose;

    class SobitProJointController : private ROSCommonNode {
        private:
            ros::NodeHandle       nh_;
            ros::NodeHandle       pnh_;

            ros::Publisher        pub_arm_joint_;
            ros::Publisher        pub_head_camera_joint_;
            tf::TransformListener listener_;

            const std::vector<std::string> joint_names_ = { "arm_shoulder_1_tilt_joint",
                                                            "arm_shoulder_2_tilt_joint",
                                                            "arm_elbow_upper_1_tilt_joint",
                                                            "arm_elbow_upper_2_tilt_joint",
                                                            "arm_elbow_lower_tilt_joint",
                                                            "arm_elbow_lower_pan_joint",
                                                            "arm_wrist_tilt_joint",
                                                            "hand_joint",
                                                            "head_camera_pan_joint",
                                                            "head_camera_tilt_joint" };
            std::vector<Pose> pose_list_;

            static const double arm1_link_length;
            static const double arm2_link_length;
            static const double arm_elbow_lower_tilt_joint_length;
            static const double hand_length;
            static const double sum_arm123_link_length;

            void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void checkPublishersConnection( const ros::Publisher& pub );
            double distanceToSec( const std::string& joint_name, const double rad, const double sec );

            void loadPose( );
            bool moveAllJoint( const double arm1,
                               const double arm2,
                               const double arm3,
                               const double arm3_pan,
                               const double arm4,
                               const double gripper,
                               const double head_camera_pan,
                               const double head_camera_tilt,
                               const double sec,
                               bool         is_sleep = true );
            geometry_msgs::Point forwardKinematics( double arm1_joint_angle, double arm2_joint_angle, double arm_elbow_lower_tilt_joint_angle );
            std::vector<std::vector<double>> inverseKinematics( double arm2_joint_to_object_x, double arm2_joint_to_object_z, double arm1_joint_angle );

            double arm_wrist_tilt_joint_current_ = 0.;
            double hand_joint_current_ = 0.;
            void callbackCurrentStateArray( const sobit_common_msg::current_state_array );
            ros::Subscriber sub_current_state_array = nh_.subscribe( "/current_state_array", 1, &SobitProJointController::callbackCurrentStateArray, this );

        public:
            SobitProJointController( const std::string& name );
            SobitProJointController( );

            bool moveToPose( const std::string& pose_name, const double sec = 5.0 );
            bool moveJoint( const Joint joint_num, const double rad, const double sec = 5.0, bool is_sleep = true );
            bool moveArm( const double arm1, const double arm2, const double arm3, const double arm3_pan, const double arm4, const double sec = 5.0, bool is_sleep = true );
            bool moveHeadPanTilt( const double head_camera_pan, const double head_camera_tilt, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToTargetCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToTargetTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToPlaceCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToPlaceTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z, const double sec = 5.0, bool is_sleep = true );
            bool graspDecision( );
    };
} // namespace sobit_pro

inline void sobit_pro::SobitProJointController::setJointTrajectory( const std::string&                joint_name,
                                                                    const double                      rad,
                                                                    const double                      sec,
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

inline void sobit_pro::SobitProJointController::addJointTrajectory( const std::string&                joint_name,
                                                                    const double                      rad,
                                                                    const double                      sec,
                                                                    trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory = *jt;
    joint_trajectory.joint_names.push_back( joint_name );
    joint_trajectory.points[0].positions.push_back( rad );
    // joint_trajectory.points[0].velocities.push_back( 0.0 );
    // joint_trajectory.points[0].accelerations.push_back( 0.0 );
    // joint_trajectory.points[0].effort.push_back( 0.0 );
    joint_trajectory.points[0].time_from_start = ros::Duration( sec );
    *jt                                        = joint_trajectory;
    return;
}

inline void sobit_pro::SobitProJointController::checkPublishersConnection( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );
    while ( pub.getNumSubscribers( ) == 0 && ros::ok( ) ) {
        try {
            loop_rate.sleep( );
        } catch ( const std::exception& ex ) {
            break;
        }
    }
    return;
}

inline double sobit_pro::SobitProJointController::distanceToSec( const std::string& joint_name, const double rad, const double sec ) {
    // get the current position of the joint
    // calculate the distance to the target (current_pos - goal_pos) in rad
    // calculate the seconds to be executed if to complete 90 degrees 1 sec is needed
    // multiply the result by a constant so it can be faster or slower
    return sec;
}

#endif /* _SOBIT_PRO_LIBRARY_JOINT_CONTROLLER_H_ */
