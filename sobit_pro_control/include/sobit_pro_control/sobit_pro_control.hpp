#ifndef SOBIT_PRO_CONTROL_H_
#define SOBIT_PRO_CONTROL_H_

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>


class SobitProControl{
    private:
        int32_t steer_pos[4] = {0, };
        int32_t wheel_vel[4] = {0, };

        enum MODE{
            NONE = -1,
            STOP_MOTION_MODE, TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE, SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

    public:
        static const int STOP_MOTION          = 0;
        static const int TRANSLATIONAL_MOTION = 1;
        static const int ROTATIONAL_MOTION    = 2;
        static const int SWIVEL_MOTION        = 3; // Motion can be added


        static constexpr const double LIMIT_VEL_VALUE = 330.;       // DXL Velocity Limit Value - XM430-W210: 330
        static constexpr const double VEL_UNIT        = 0.229;      // DXL Velocity Unit [rmp]
        static constexpr const double WHEEL_DIAMETER  = 0.144;      // Wheel Circumference [m]
        static constexpr const double WHEEL_LENGTH    = M_PI*WHEEL_DIAMETER; // Wheel Length [m]
        static constexpr const double BODY_DIAMETER   = 0.44775010; // Robot Diameter [m] (respect to the center of wheels)
        static constexpr const double TRACK           = 0.31660713; // Distance between left and right wheels [m]

        double steer_fl_goal_pos, steer_fr_goal_pos, steer_bl_goal_pos, steer_br_goal_pos;
        double wheel_fl_goal_vel, wheel_fr_goal_vel, wheel_bl_goal_vel, wheel_br_goal_vel;

        // Constructor
        SobitProControl():
        steer_fl_goal_pos(0), steer_fr_goal_pos(0), steer_bl_goal_pos(0), steer_br_goal_pos(0),
        wheel_fl_goal_vel(0), wheel_fr_goal_vel(0), wheel_bl_goal_vel(0), wheel_br_goal_vel(0){
        }

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

        void setParams(geometry_msgs::Twist vel_twist);
        inline int getMotionMode(){ return int(motion_mode); };
        int32_t *setSteerPos();
        int32_t *setWheelVel();
};

#endif // SOBIT_PRO_CONTROL_H_
