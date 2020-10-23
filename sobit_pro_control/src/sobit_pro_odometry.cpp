
#include "sobit_pro_odometry.hpp"

// Odometry calculation
bool SobitProOdometry::odom(float steer_fr_present_position, float steer_fl_present_position, float wheel_fr_present_position, float wheel_fl_present_position, int32_t wheel_fr_initial_position, int32_t wheel_fl_initial_position, nav_msgs::Odometry* output_odom){
  nav_msgs::Odometry result_odom;
  result_odom = *output_odom;

  switch (motion_mode){

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{
      float fr_distance_m = translational_distance_calculation(wheel_fr_present_position - wheel_fr_initial_position); // Calculation distance[m]
      float fl_distance_m = translational_distance_calculation(wheel_fl_present_position - wheel_fl_initial_position); // Calculation distance[m]
      float fr_direction_deg = translational_position_calculation(steer_fr_present_position); // Record present position
      float fl_direction_deg = translational_position_calculation(steer_fl_present_position); // Record present position

      // Convert Wheel Coordinates to Robot Coordinates
      fr_direction_deg = fr_direction_deg + 45;
      fl_direction_deg = fl_direction_deg - 45;

      if(0.5 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 0.5){
        float distance_m = std::abs(fr_distance_m) + std::abs(fl_distance_m) / 2;
        result_odom.pose.pose.position.x = distance_m * cos(fr_direction_deg * (PAI / 180.));
        result_odom.pose.pose.position.y = distance_m * sin(fr_direction_deg * (PAI / 180.));
      }
      else{
        printf("Odometry ERROR\n");
      }

      *output_odom = result_odom;
      return true;
    }

/* ############################################################################################ */

    // Rotational motion
    case ROTATIONAL_MOTION_MODE:{
      float fr_distance_m = rotational_distance_calculation(wheel_fr_present_position - wheel_fr_initial_position); // Record present velocity
      float fl_distance_m = rotational_distance_calculation(wheel_fl_present_position - wheel_fr_initial_position); // Record present velocity
      float fr_direction_deg = rotational_position_calculation(steer_fr_present_position); // Record present position
      float fl_direction_deg = rotational_position_calculation(steer_fl_present_position); // Record present position
      float rotational_position_deg;
      float rotational_position_rad;

      if(0.5 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 0.5){
        float distance_m = std::abs(fr_distance_m) + std::abs(fl_distance_m) / 2.;

        rotational_position_deg = distance_m / SOBIT_CAR_DIAMETER * PAI * 360.;
        rotational_position_deg = rotational_position_deg * (PAI / 180.); // rad = deg * (PAI / 180.)

        result_odom.pose.pose.orientation.x = 1;
        result_odom.pose.pose.orientation.y = 1;
        result_odom.pose.pose.orientation.z = 1;
        result_odom.pose.pose.orientation.w = 1;
      }
      else{
        printf("Odometry ERROR\n");
      }

      std::cout << "\n[ rotational_position_deg z : ]" << rotational_position_deg << std::endl;

      *output_odom = result_odom;
      return true;
    }

/* ############################################################################################ */
    
    // Other motion
    default:
      return true;
  }
}


/* ############################################################################################ */


float SobitProOdometry::translational_distance_calculation(float wheel_present_position){
  float distance_m;

  // Distance calculation
  distance_m = WHEEL_LENGTH * wheel_present_position / 4096.;

  return distance_m;
}

float SobitProOdometry::translational_position_calculation(float steer_present_position){
  float direction_deg;

  // Position calculation
  direction_deg = (steer_present_position - 2048) * 360. / 4096.;

  return direction_deg;
}


/* ############################################################################################ */


float SobitProOdometry::rotational_distance_calculation(float wheel_present_vel){
  float distance_m;

  // Distance calculation
  distance_m = WHEEL_LENGTH * wheel_present_vel / 4096.;

  return distance_m;
}

float SobitProOdometry::rotational_position_calculation(float steer_present_position){

  // Position calculation
  float direction_deg = (steer_present_position - 2048) * 360. / 4096.;

  return direction_deg;
}



