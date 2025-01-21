#ifndef SOBIT_PRO_MAIN_H_
#define SOBIT_PRO_MAIN_H_

#include <iostream>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>


class SobitProMain{
    private:
        int motion;
        
        void callback(const geometry_msgs::Twist vel_twist);
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Subscriber sub_vel          = nh.subscribe("mobile_base/commands/velocity", 1, &SobitProMain::callback, this);
        ros::Publisher  pub_odometry     = nh.advertise<nav_msgs::Odometry>("odom", 1);
        ros::Publisher  pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        ros::Publisher  pub_wheels_error = nh.advertise<std_msgs::Bool>("wheels_error", 1);

        int32_t wheel_fl_init_pos;
        int32_t wheel_fr_init_pos;
        int32_t wheel_bl_init_pos;
        int32_t wheel_br_init_pos;

        int32_t wheel_fl_curt_pos;
        int32_t wheel_fr_curt_pos;
        int32_t wheel_bl_curt_pos;
        int32_t wheel_br_curt_pos;

        int32_t steer_fl_curt_pos;
        int32_t steer_fr_curt_pos;
        int32_t steer_bl_curt_pos;
        int32_t steer_br_curt_pos;

        int32_t *set_steer_pos;
        int32_t *set_wheel_vel;

        int32_t prev_motion = -1; // When init avoid 0~3 motions

        sensor_msgs::JointState joint_state;

        nav_msgs::Odometry result_odom;
        nav_msgs::Odometry prev_odom;

        std_msgs::Bool wheels_error;

    public:
        bool start_up_sound();
        bool shut_down_sound();
        void control_wheel();

};

#endif // SOBIT_PRO_MAIN_H_
