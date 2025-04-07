#ifndef SOBIT_PRO_MAIN_HPP_
#define SOBIT_PRO_MAIN_HPP_

#include <iostream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"



#include <ament_index_cpp/get_package_share_directory.hpp>

class SobitProMain : public rclcpp::Node {
    public:
        SobitProMain();
    
        bool start_up_sound();
        bool shut_down_sound();
        void control_wheel();
    
    private:
        int motion;
        // ROS2 I/F
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_info_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_steer_joint_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_joint_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_wheels_error_;
    
        // コールバック
        void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
        // ユーティリティ関数
        double getJointPos(const std::string& joint_name);
        double getJointVel(const std::string& joint_name);
        void setPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt);
        void addPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt);
        void checkPublishersConnection(rclcpp::PublisherBase::SharedPtr pub);
    
        // ロボット状態管理
        int32_t wheel_fl_init_pos, wheel_fr_init_pos, wheel_bl_init_pos, wheel_br_init_pos;
        int32_t wheel_fl_curt_pos, wheel_fr_curt_pos, wheel_bl_curt_pos, wheel_br_curt_pos;
        int32_t steer_fl_curt_pos, steer_fr_curt_pos, steer_bl_curt_pos, steer_br_curt_pos;
    
        int32_t* set_steer_pos;
        int32_t* set_wheel_vel;
        int32_t prev_motion = -1;
    
        std::map<std::string, double> joints_pos;
        std::map<std::string, double> joints_vel;
    
        nav_msgs::msg::Odometry result_odom;
        nav_msgs::msg::Odometry prev_odom;
        std_msgs::msg::Bool wheels_error;
    };
    
    // 実装部
    inline SobitProMain::SobitProMain() : Node("sobit_pro_main")
    {
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "mobile_base/commands/velocity", 10, std::bind(&SobitProMain::callback, this, std::placeholders::_1));
    
        sub_joint_info_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&SobitProMain::joint_callback, this, std::placeholders::_1));
    
        pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_steer_joint_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("steer_trajectory_controller/command", 10);
        pub_wheel_joint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_trajectory_controller/command", 10);
        pub_wheels_error_ = this->create_publisher<std_msgs::msg::Bool>("wheels_error", 10);
    }
    
    inline double SobitProMain::getJointPos(const std::string& joint_name) {
        return joints_pos[joint_name];
    }
    
    inline double SobitProMain::getJointVel(const std::string& joint_name) {
        return joints_vel[joint_name];
    }
    
    inline void SobitProMain::setPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt) {
        jt->joint_names = {joint_name};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {rad};
        point.time_from_start = rclcpp::Duration::from_seconds(sec);
        jt->points = {point};
    }
    
    inline void SobitProMain::addPosJointTrajectory(const std::string& joint_name, double rad, double sec, trajectory_msgs::msg::JointTrajectory* jt) {
        jt->joint_names.push_back(joint_name);
        if (jt->points.empty()) {
            jt->points.emplace_back();
            jt->points[0].time_from_start = rclcpp::Duration::from_seconds(sec);
        }
        jt->points[0].positions.push_back(rad);
        jt->points[0].time_from_start = rclcpp::Duration::from_seconds(sec);
    }
    
    inline void SobitProMain::checkPublishersConnection(rclcpp::PublisherBase::SharedPtr pub) {
        rclcpp::Rate rate(10);
        while (pub->get_subscription_count() == 0 && rclcpp::ok()) {
            rate.sleep();
        }
    }
    
    // コールバック関数は別途実装
    
    #endif // SOBIT_PRO_MAIN_HPP_
