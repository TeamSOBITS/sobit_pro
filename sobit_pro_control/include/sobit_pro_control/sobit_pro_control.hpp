#ifndef SOBIT_PRO_CONTROL_H_
#define SOBIT_PRO_CONTROL_H_

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

// Define motion key value
#define TRANSLATIONAL_MOTION 1
#define ROTATIONAL_MOTION    2
#define SWIVEL_MOTION        3 // Motion can be added

#define LIMIT_VEL_VALUE      200.
#define WHEEL_LENGTH         0.452389 // Wheel Circumference
#define BODY_DIAMETER        0.448051 // Robot Diameter

class SobitProControl{
    private:
        int32_t steer_angle[4] = {0, };
        int32_t wheel_vel[4]   = {0, };

        enum MODE{
            NONE = 0,
            TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE, SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

  public:
        float steer_fr_goal_angle, steer_fl_goal_angle, steer_br_goal_angle, steer_bl_goal_angle;
        float wheel_fr_goal_vel,   wheel_fl_goal_vel,   wheel_br_goal_vel,   wheel_bl_goal_vel;

        // Constructor
        SobitProControl():
        steer_fr_goal_angle(0), steer_fl_goal_angle(0), steer_br_goal_angle(0), steer_bl_goal_angle(0),
        wheel_fr_goal_vel(0),   wheel_fl_goal_vel(0),   wheel_br_goal_vel(0),   wheel_bl_goal_vel(0){
        }

        MODE getMotion(int motion){
            switch (motion){
                case (TRANSLATIONAL_MOTION): motion_mode = TRANSLATIONAL_MOTION_MODE; break;
                case (ROTATIONAL_MOTION)   : motion_mode = ROTATIONAL_MOTION_MODE;    break;
                case (SWIVEL_MOTION)       : motion_mode = SWIVEL_MOTION_MODE;        break; // Motion can be added

                default: motion_mode = NONE; break;
            }
            return motion_mode;
        }

        void setParams(geometry_msgs::Twist vel_twist);
        int showMode();
        int32_t *setSteerAngle();
        int32_t *setWheelVel();
};

#endif // SOBIT_PRO_CONTROL_H_
