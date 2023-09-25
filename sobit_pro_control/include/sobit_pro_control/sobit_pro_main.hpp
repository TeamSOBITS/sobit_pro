#ifndef SOBIT_PRO_MAIN_H_
#define SOBIT_PRO_MAIN_H_

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#define VEL_UNIT             0.229

class SobitProMain{
    private:
        int motion;
        
        void callback(const geometry_msgs::Twist);
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Subscriber sub_velocity     = nh.subscribe("/mobile_base/commands/velocity", 1, &SobitProMain::callback, this);
        ros::Publisher  pub_odometry     = nh.advertise<nav_msgs::Odometry>("/odom", 1);
        ros::Publisher  pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
        ros::Publisher  pub_wheels_error = nh.advertise<std_msgs::Bool>("/wheels_error", 1);
        // ros::Publisher pub_hz = nh.advertise<std_msgs::Empty>("/hz", 1);

        int32_t wheel_fr_init_position;
        int32_t wheel_fl_init_position;
        int32_t wheel_br_init_position;
        int32_t wheel_bl_init_position;

        int32_t wheel_fr_curt_position;
        int32_t wheel_fl_curt_position;
        int32_t wheel_br_curt_position;
        int32_t wheel_bl_curt_position;

        int32_t steer_fr_curt_position;
        int32_t steer_fl_curt_position;
        int32_t steer_br_curt_position;
        int32_t steer_bl_curt_position;

        int32_t *set_steer_angle;
        int32_t *set_wheel_vel;

        int32_t prev_motion = -1; // Non 0, 1, 2 3 motion

        sensor_msgs::JointState joint_state;

        nav_msgs::Odometry result_odom;
        nav_msgs::Odometry prev_odom;
        
        ros::Time prev_time;

        std_msgs::Bool wheels_error;

    public:
        void start_up_sound();
        void shut_down_sound();
        void control_wheel();

};

#endif // SOBIT_PRO_MAIN_H_
