#ifndef SOBIT_PRO_CONTROL_HPP_
#define SOBIT_PRO_CONTROL_HPP_

#include <cmath>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

class SobitProControl {
    private:
        std::array<int32_t, 4> steer_pos = {0, 0, 0, 0};
        std::array<int32_t, 4> wheel_vel = {0, 0, 0, 0};

        enum MODE {
            NONE = -1,
            STOP_MOTION_MODE,
            TRANSLATIONAL_MOTION_MODE,
            ROTATIONAL_MOTION_MODE,
            SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

    public:
        static constexpr int STOP_MOTION          = 0;
        static constexpr int TRANSLATIONAL_MOTION = 1;
        static constexpr int ROTATIONAL_MOTION    = 2;
        static constexpr int SWIVEL_MOTION        = 3; // Motion can be added

        static constexpr double LIMIT_VEL_VALUE = 330.0;       // DXL Velocity Limit Value - XM430-W210: 330
        static constexpr double VEL_UNIT        = 0.229;       // DXL Velocity Unit [rpm]
        static constexpr double WHEEL_DIAMETER  = 0.144;       // Wheel Diameter [m]
        static constexpr double WHEEL_LENGTH    = M_PI * WHEEL_DIAMETER; // Wheel Length [m]
        static constexpr double BODY_DIAMETER   = 0.44775010;  // Robot Diameter [m] (respect to the center of wheels)
        static constexpr double TRACK           = 0.31660713;  // Distance between left and right wheels [m]

        double steer_fl_goal_pos, steer_fr_goal_pos, steer_bl_goal_pos, steer_br_goal_pos;
        double wheel_fl_goal_vel, wheel_fr_goal_vel, wheel_bl_goal_vel, wheel_br_goal_vel;

        // Constructor
        SobitProControl()
            : steer_fl_goal_pos(0), steer_fr_goal_pos(0),
              steer_bl_goal_pos(0), steer_br_goal_pos(0),
              wheel_fl_goal_vel(0), wheel_fr_goal_vel(0),
              wheel_bl_goal_vel(0), wheel_br_goal_vel(0) {}

        MODE getMotion(int motion) {
            switch (motion) {
                case STOP_MOTION_MODE:          motion_mode = STOP_MOTION_MODE; break;
                case TRANSLATIONAL_MOTION_MODE: motion_mode = TRANSLATIONAL_MOTION_MODE; break;
                case ROTATIONAL_MOTION_MODE:    motion_mode = ROTATIONAL_MOTION_MODE; break;
                case SWIVEL_MOTION_MODE:        motion_mode = SWIVEL_MOTION_MODE; break;
                default: motion_mode = NONE; break;
            }
            return motion_mode;
        }

        void setParams(const geometry_msgs::msg::Twist& vel_twist);
        inline int getMotionMode() const { return static_cast<int>(motion_mode); }
        std::array<int32_t, 4> setSteerPos();
        std::array<int32_t, 4> setWheelVel();
};

#endif // SOBIT_PRO_CONTROL_HPP_
