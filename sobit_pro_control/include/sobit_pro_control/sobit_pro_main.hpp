#ifndef SOBIT_PRO_MAIN_H_
#define SOBIT_PRO_MAIN_H_

#include <ros/ros.h>
#include <stdlib.h>
#include<bits/stdc++.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

class SobitProMain{
  private:
    int motion;
    
    void callback(const geometry_msgs::Twist);
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber sub_velocity = nh.subscribe("/mobile_base/commands/velocity", 1, &SobitProMain::callback, this);
    ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher pub_wheels_error = nh.advertise<std_msgs::Bool>("/wheels_error", 1);
    // ros::Publisher pub_hz = nh.advertise<std_msgs::Empty>("/hz", 1);

    int32_t wheel_fr_initial_position;
    int32_t wheel_fl_initial_position;
    int32_t wheel_fr_present_position;
    int32_t wheel_fl_present_position;
    int32_t steer_fr_present_position;
    int32_t *set_steer_angle;
    int32_t *set_wheel_vel;
    int32_t old_motion = 3; // Non 0, 1, 2 motion
    sensor_msgs::JointState joint_state;
    nav_msgs::Odometry result_odom;
    nav_msgs::Odometry old_odom;
    std_msgs::Bool wheels_error;

  public:
    void start_up_sound();
    void shut_down_sound();
    void control_wheel();

};

#endif // SOBIT_PRO_MAIN_H_
