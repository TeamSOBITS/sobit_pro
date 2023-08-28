#ifndef SOBIT_PRO_ODOMETRY_H_
#define SOBIT_PRO_ODOMETRY_H_

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

// Define motion key value
#define STOP_MOTION          0
#define TRANSLATIONAL_MOTION 1
#define ROTATIONAL_MOTION    2
#define SWIVEL_MOTION        3 // Motion can be added

#define WHEEL_LENGTH         0.452389 // Wheel Circumference
#define BODY_DIAMETER        0.448051 // Robot Diameter

class SobitProOdometry{
    private:
        enum MODE{
        NONE = 0,
        STOP_MOTION_MODE, TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE, SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

  public:
        bool odom(int32_t steer_fr_curt_position, int32_t steer_fl_curt_position,
                  int32_t steer_br_curt_position, int32_t steer_bl_curt_position,
                  int32_t wheel_fr_curt_velocity, int32_t wheel_fl_curt_velocity,
                  int32_t wheel_br_curt_velocity, int32_t wheel_bl_curt_velocity,
                  int32_t wheel_fr_curt_position, int32_t wheel_fl_curt_position,
                  int32_t wheel_br_curt_position, int32_t wheel_bl_curt_position,
                  int32_t wheel_fr_init_position, int32_t wheel_fl_init_position,
                  int32_t wheel_br_init_position, int32_t wheel_bl_init_position,
                  int32_t prev_motion,
                  nav_msgs::Odometry prev_odom, nav_msgs::Odometry* result_odom,
                  ros::Time prev_time);
        float distance_calculation(float wheel_curt_position);
        float position_calculation(float steer_curt_position);
        void  pose_broadcaster(const nav_msgs::Odometry tf_odom);
        double integrateXY(double linear_x, double linear_y, double angular, double pre_ori_z);

        MODE getMotion(int motion){
            switch (motion){
                case (STOP_MOTION)         : motion_mode = STOP_MOTION_MODE;          break;
                case (TRANSLATIONAL_MOTION): motion_mode = TRANSLATIONAL_MOTION_MODE; break;
                case (ROTATIONAL_MOTION)   : motion_mode = ROTATIONAL_MOTION_MODE;    break;
                case (SWIVEL_MOTION)       : motion_mode = SWIVEL_MOTION_MODE;        break; // Motion can be added

                default: motion_mode = NONE; break;
            }
        
        return motion_mode;
        }

};

#endif // SOBIT_PRO_ODOMETRY_H_
