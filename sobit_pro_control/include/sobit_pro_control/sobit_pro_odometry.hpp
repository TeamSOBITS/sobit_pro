#ifndef SOBIT_PRO_ODOMETRY_H_
#define SOBIT_PRO_ODOMETRY_H_

#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

// Define motion key value
// #define STOP_MOTION          0
// #define TRANSLATIONAL_MOTION 1
// #define ROTATIONAL_MOTION    2
// #define SWIVEL_MOTION        3 // Motion can be added

// #define WHEEL_DIAMETER       0.144                  // Wheel Circumference
// #define WHEEL_LENGTH         M_PI*WHEEL_DIAMETER    // Wheel Length
// #define BODY_DIAMETER        0.44775010             // Robot Diameter
// #define TRACK                0.31660713             // Distance between left and right wheels

class SobitProOdometry{
    private:
        enum MODE{
        NONE = -1,
        STOP_MOTION_MODE, TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE, SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

    public:
        bool odom(int32_t steer_fl_curt_pos, int32_t steer_fr_curt_pos,
                  int32_t steer_bl_curt_pos, int32_t steer_br_curt_pos,
                  // int32_t wheel_fl_curt_vel, int32_t wheel_fr_curt_vel,
                  // int32_t wheel_bl_curt_vel, int32_t wheel_br_curt_vel,
                  int32_t wheel_fl_curt_pos, int32_t wheel_fr_curt_pos,
                  int32_t wheel_bl_curt_pos, int32_t wheel_br_curt_pos,
                  int32_t wheel_fl_init_pos, int32_t wheel_fr_init_pos,
                  int32_t wheel_bl_init_pos, int32_t wheel_br_init_pos,
                  int32_t prev_motion,
                  nav_msgs::Odometry prev_odom, nav_msgs::Odometry* result_odom,
                  ros::Time prev_time);
        double distance_calculation(double wheel_curt_pos);
        double position_calculation(double steer_curt_pos);
        void   pose_broadcaster(const nav_msgs::Odometry tf_odom);

        MODE getMotion(int motion){
            switch (motion){
                case (STOP_MOTION_MODE)          : motion_mode = STOP_MOTION_MODE;          break;
                case (TRANSLATIONAL_MOTION_MODE) : motion_mode = TRANSLATIONAL_MOTION_MODE; break;
                case (ROTATIONAL_MOTION_MODE)    : motion_mode = ROTATIONAL_MOTION_MODE;    break;
                case (SWIVEL_MOTION_MODE)        : motion_mode = SWIVEL_MOTION_MODE;        break; // Motion can be added

                default: motion_mode = NONE; break;
            }
        
            return motion_mode;
        }

};

#endif // SOBIT_PRO_ODOMETRY_H_
