#ifndef SOBIT_PRO_CONTROL_H_
#define SOBIT_PRO_CONTROL_H_

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Define motion key value
#define STOP_MOTION          0
#define TRANSLATIONAL_MOTION 1
#define ROTATIONAL_MOTION    2
#define SWIVEL_MOTION        3 // Motion can be added

// #define LIMIT_VEL_VALUE      200.
#define LIMIT_VEL_VALUE      1023.    // DXL Velocity Limit Value
#define VEL_UNIT             0.229    // DXL Velocity Unit [rmp]
#define WHEEL_DIAMETER       0.144    // Wheel Circumference
#define WHEEL_RADIUS         WHEEL_DIAMETER/2.0
#define WHEEL_LENGTH         M_PI*WHEEL_DIAMETER // Wheel Length

// #define WHEEL_BASE           0.31660713 // Distance between front and rear wheel
#define BODY_DIAMETER        0.448051   // Robot Diameter
#define TRACK                0.31660713 // Distance between left and right wheels
// #define WHEEL_STEER_Y_OFFSET 0.0 // Distance between a wheel joint and the associated steering joint



class SobitProControl{
    private:
        int32_t steer_angle[4] = {0, };
        int32_t wheel_vel[4]   = {0, };

        enum MODE{
            NONE = -1,
            STOP_MOTION_MODE, TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE, SWIVEL_MOTION_MODE // Motion can be added
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
                case (STOP_MOTION)         : motion_mode = STOP_MOTION_MODE;          break;
                case (TRANSLATIONAL_MOTION): motion_mode = TRANSLATIONAL_MOTION_MODE; break;
                case (ROTATIONAL_MOTION)   : motion_mode = ROTATIONAL_MOTION_MODE;    break;
                case (SWIVEL_MOTION)       : motion_mode = SWIVEL_MOTION_MODE;        break; // Motion can be added

                default: motion_mode = NONE; break;
            }
            return motion_mode;
        }

        void setParams(geometry_msgs::Twist vel_twist);
        int showMode();
        int32_t *setSteerPos();
        int32_t *setWheelVel();
};

#endif // SOBIT_PRO_CONTROL_H_
