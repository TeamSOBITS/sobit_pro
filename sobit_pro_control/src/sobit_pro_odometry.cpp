#include "sobit_pro_odometry.hpp"

// Odometry calculation
bool SobitProOdometry::odom(float steer_fr_present_angle, float steer_fl_present_angle, float wheel_fr_present_vel, float wheel_fl_present_vel, nav_msgs::Odometry* output_odom){
  float time_begin = ros::Time::now().toSec(); // Record time before calculation
  nav_msgs::Odometry result_odom;
  result_odom = *output_odom;

  switch (motion_mode){
    case TRANSLATIONAL_MOTION_MODE:{
      float fr_vel_ms = translational_velocity_calculation(wheel_fr_present_vel); // Record present velocity
      float fl_vel_ms = translational_velocity_calculation(wheel_fl_present_vel); // Record present velocity
      float fr_direction_deg = translational_position_calculation(steer_fr_present_angle); // Record present position
      float fl_direction_deg = translational_position_calculation(steer_fl_present_angle); // Record present position

      /* ############################################################################################ */

      std::cout << "\n[ Odometry_R ]" << fr_direction_deg << std::endl;
      std::cout << "\n[ Odometry_L ]" << fl_direction_deg << std::endl;
  
      if(fr_direction_deg < 0){
          fr_direction_deg = 90 - fr_direction_deg;
      }
      fr_direction_deg = fr_direction_deg - 45;
      
      float time_duration = ros::Time::now().toSec() - time_begin;
      float distance_m = fr_vel_ms * time_duration;
      result_odom.pose.pose.position.x += distance_m * cos(fr_direction_deg * (PAI / 180.));
      result_odom.pose.pose.position.y += distance_m * sin(fr_direction_deg * (PAI / 180.));

      /*
      if((0 < fr_vel_ms) && (fr_direction_deg != 0 && fl_direction_deg != 0)){
        if(fr_direction_deg < 0){
          fr_direction_deg = 90 - fr_direction_deg;
        }
        fr_direction_deg = fr_direction_deg - 45;

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
        else{
          printf("aaaaaaaaaaaaaa\n");
        }
      }
      else if((fr_vel_ms < 0) && (fr_direction_deg != 0 && fl_direction_deg != 0)){
        if(fr_direction_deg < 0){
          fr_direction_deg = 90 - fr_direction_deg;
        }
        fr_direction_deg = fr_direction_deg - 45;

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
        else{
          printf("bbbbbbbbbbbbbbb\n");
        }
      }

      else{
          printf("cccccccccccccccc\n");
        }
      */

      /* ############################################################################################ */

      *output_odom = result_odom;
      return true;
    }
    case ROTATIONAL_MOTION_MODE:{
      float fr_vel_ms = rotational_velocity_calculation(wheel_fr_present_vel); // Record present velocity
      float fl_vel_ms = rotational_velocity_calculation(wheel_fl_present_vel); // Record present velocity
      float fr_direction_deg = rotational_position_calculation(steer_fr_present_angle); // Record present position
      float fl_direction_deg = rotational_position_calculation(steer_fl_present_angle); // Record present position

      // result_odom.pose.pose.orientation.z += 1;
      // result_odom.pose.pose.orientation.w += 1;

      /*
      // Rotational motion
      else if(fr_direction_deg == 0 && fl_direction_deg == 0){
        //velocity_degs = twist.angular.z / (PAI / 180.); // rad = deg * (PAI / 180.)
        //velocity_rpm = (velocity_degs / 360.) * 60.;
        //velocity_value = velocity_rpm / 0.229;
        //result_odom.pose.pose.orientation = 1;
      }
      */

      *output_odom = result_odom;
      return true;
    }
    default:
      return true;
  }
}


/* ############################################################################################ */


float SobitProOdometry::translational_velocity_calculation(float wheel_present_vel){
  float vel_rpm;
  float vel_ms;

  // Velocity calculation
  vel_rpm = wheel_present_vel * 0.299;
  vel_ms = vel_rpm * WHEEL_LENGTH / 60; //60:minute

  return vel_ms;
}

float SobitProOdometry::translational_position_calculation(float steer_present_angle){
  float direction_deg;

  // Position calculation
  direction_deg = (steer_present_angle - 2048) * 360. / 4096.;

  return direction_deg;
}


/* ############################################################################################ */


float SobitProOdometry::rotational_velocity_calculation(float wheel_present_vel){

  // Velocity calculation
  float vel_rpm = wheel_present_vel * 0.299;
  float vel_ms = vel_rpm * WHEEL_LENGTH / 60;  //60:minute
  float vel_deg = vel_ms * 360 / SOBIT_CAR_DIAMETER / PAI;
  float vel_rad = vel_deg / 180 * PAI; // deg to rad

  return vel_rad;
}

float SobitProOdometry::rotational_position_calculation(float steer_present_angle){

  // Position calculation
  float direction_deg = (steer_present_angle - 2048) * 360. / 4096.;

  return direction_deg;
}



