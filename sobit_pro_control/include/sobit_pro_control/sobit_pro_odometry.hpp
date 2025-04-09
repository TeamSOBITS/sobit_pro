#ifndef SOBIT_PRO_ODOMETRY_HPP_
#define SOBIT_PRO_ODOMETRY_HPP_

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/point.hpp>

class SobitProOdometry : public rclcpp::Node{
    private:
        enum MODE {
            NONE = -1,
            STOP_MOTION_MODE,
            TRANSLATIONAL_MOTION_MODE,
            ROTATIONAL_MOTION_MODE,
            SWIVEL_MOTION_MODE // Motion can be added
        } motion_mode;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    public:

        SobitProOdometry() 
            : rclcpp::Node("sobit_pro_odometry_node")
        {
            // 必要な初期化処理
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        
        bool odom(int32_t steer_fl_curt_pos, int32_t steer_fr_curt_pos,
                  int32_t steer_bl_curt_pos, int32_t steer_br_curt_pos,
                  int32_t wheel_fl_curt_pos, int32_t wheel_fr_curt_pos,
                  int32_t wheel_bl_curt_pos, int32_t wheel_br_curt_pos,
                  int32_t wheel_fl_init_pos, int32_t wheel_fr_init_pos,
                  int32_t wheel_bl_init_pos, int32_t wheel_br_init_pos,
                  int32_t prev_motion,
                  nav_msgs::msg::Odometry& prev_odom, nav_msgs::msg::Odometry& result_odom,//);
                  rclcpp::Time& prev_time);

        double distance_calculation(double wheel_curt_pos);
        double position_calculation(double steer_curt_pos);
        void pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom);

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
};

#endif // SOBIT_PRO_ODOMETRY_HPP_
