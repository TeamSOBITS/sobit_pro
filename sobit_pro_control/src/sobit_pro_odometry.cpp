#include "sobit_pro_odometry.hpp"

// Odometry calculation
nav_msgs::Odometry SobitProOdometry::odom(float steer_fr_present_angle, float steer_fl_present_angle, float wheel_fr_present_vel, float wheel_fl_present_vel){
  float time_begin = ros::Time::now().toSec(); // Record time before calculation

  float fr_vel_ms = velocity_calculation(steer_fr_present_angle); // Record present position
  float fl_vel_ms = velocity_calculation(steer_fl_present_angle); // Record present position
  float fr_direction_deg = position_calculation(wheel_fr_present_vel); // Record present velocity
  float fl_direction_deg = position_calculation(wheel_fl_present_vel); // Record present velocity

  nav_msgs::Odometry result_odom;

  // Translational motion
  if((0 < fr_vel_ms) && (fr_direction_deg != 0 && fl_direction_deg != 0)){
    if(fr_direction_deg < 0){
      fr_direction_deg = 90 -fr_direction_deg;
    }
    fr_direction_deg = fr_direction_deg -45;

    if(fr_direction_deg < 0){
      float time_duration = ros::Time::now().toSec() - time_begin;
      float distance_m = fr_vel_ms * time_duration;
      result_odom.pose.pose.position.x += -distance_m * cos(fr_direction_deg * (PAI / 180.));
      result_odom.pose.pose.position.y += -distance_m * sin(fr_direction_deg * (PAI / 180.));
    }
    else if(0 < fr_vel_ms){
      float time_duration = ros::Time::now().toSec() - time_begin;
      float distance_m = fr_vel_ms * time_duration;
      result_odom.pose.pose.position.x += distance_m * cos(fr_direction_deg * (PAI / 180.));
      result_odom.pose.pose.position.y += distance_m * sin(fr_direction_deg * (PAI / 180.));
    }
  }
  else if((fr_vel_ms < 0) && (fr_direction_deg != 0 && fl_direction_deg != 0)){
    if(fr_direction_deg < 0){
      fr_direction_deg = 90 -fr_direction_deg;
    }
    fr_direction_deg = fr_direction_deg -45;
    if(fr_direction_deg < 0){
      float time_duration = ros::Time::now().toSec() - time_begin;
      float distance_m = fr_vel_ms * time_duration;
      result_odom.pose.pose.position.x += distance_m * cos(fr_direction_deg * (PAI / 180.));
      result_odom.pose.pose.position.y += distance_m * sin(fr_direction_deg * (PAI / 180.));
    }
    else if(0 < fr_vel_ms){
      float time_duration = ros::Time::now().toSec() - time_begin;
      float distance_m = fr_vel_ms * time_duration;
      result_odom.pose.pose.position.x += -distance_m * cos(fr_direction_deg * (PAI / 180.));
      result_odom.pose.pose.position.y += -distance_m * sin(fr_direction_deg * (PAI / 180.));
    }
  }

  // Rotational motion
  else if(fr_direction_deg == 0 && fl_direction_deg == 0){
    //velocity_degs = twist.angular.z / (PAI / 180.); // rad = deg * (PAI / 180.)
    //velocity_rpm = (velocity_degs / 360.) * 60.;
    //velocity_value = velocity_rpm / 0.229;
    //result_odom.pose.pose.orientation = 1;
  }
  return result_odom;
}

float SobitProOdometry::velocity_calculation(float wheel_present_vel){
  float vel_rpm;
  float vel_ms;

  // Velocity calculation
  vel_rpm = wheel_present_vel * 0.299;
  vel_ms = vel_rpm * WHEEL_LENGTH * 60;

  return vel_ms;
}

float SobitProOdometry::position_calculation(float steer_present_angle){
  float direction_deg;

  // Position calculation
  direction_deg = (steer_present_angle - 2048) * 360. / 4096.;

  return direction_deg;
}
