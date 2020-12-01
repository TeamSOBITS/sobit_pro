#include "sobit_pro_odometry.hpp"

// Odometry calculation
bool SobitProOdometry::odom(float steer_fr_present_position, float steer_fl_present_position, float wheel_fr_present_position, float wheel_fl_present_position, 
                            int32_t wheel_fr_initial_position, int32_t wheel_fl_initial_position, int32_t old_motion, nav_msgs::Odometry old_odom, nav_msgs::Odometry* result_odom){

  float fr_distance_m = distance_calculation(wheel_fr_present_position - wheel_fr_initial_position); // Calculation distance[m]
  float fl_distance_m = distance_calculation(wheel_fl_present_position - wheel_fl_initial_position); // Calculation distance[m]
  float fr_direction_deg = position_calculation(steer_fr_present_position); // Record present position
  float fl_direction_deg = position_calculation(steer_fl_present_position); // Record present position
  double old_odom_orientation_x, old_odom_orientation_y, old_odom_orientation_z;
  float distance_m;
  nav_msgs::Odometry calculation_odom;
  tf::Quaternion quat_tf;
  calculation_odom = *result_odom;

  if(old_motion == 1){
    motion_mode = TRANSLATIONAL_MOTION_MODE;
  }
  else if (old_motion ==2){
    motion_mode = ROTATIONAL_MOTION_MODE;
  }

  switch (motion_mode){

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{

      // Convert Wheel Coordinates to Robot Coordinates
      fr_direction_deg = fr_direction_deg + 45.;
      fl_direction_deg = fl_direction_deg - 45.;

      // Check the calculation
      if(0.0 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1.0){

        // Positive distance or Negative distance
        if(-45 <= fr_distance_m && fr_distance_m <= 90){
          if(0 <= fr_distance_m){
            distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
          else{
            distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
        }
        else if(90 < fr_distance_m && fr_distance_m <= 135){
          if(0 <= fr_distance_m){
            distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
          else{
            distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
        }
        else printf("Calculation ERROR\n");
      }
      else printf("Odometry ERROR\n");

      // Change euler (old_odom)
      quaternionMsgToTF(old_odom.pose.pose.orientation, quat_tf);
      tf::Matrix3x3(quat_tf).getRPY(old_odom_orientation_x, old_odom_orientation_y, old_odom_orientation_z);

      // Add the amount of movement to the odometry
      calculation_odom.pose.pose.position.x = old_odom.pose.pose.position.x + distance_m * cos(fr_direction_deg * (PAI / 180.)) * cos(old_odom_orientation_z)
                                                                            + distance_m * sin(fr_direction_deg * (PAI / 180.)) * cos(old_odom_orientation_z + 1.5708);

      calculation_odom.pose.pose.position.y = old_odom.pose.pose.position.y + distance_m * cos(fr_direction_deg * (PAI / 180.)) * sin(old_odom_orientation_z)
                                                                            + distance_m * sin(fr_direction_deg * (PAI / 180.)) * sin(old_odom_orientation_z + 1.5708);

      calculation_odom.pose.pose.orientation = old_odom.pose.pose.orientation;

      *result_odom = calculation_odom;

      return true;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE:{
      float rotational_position_deg;
      float rotational_position_rad;

      // Check the calculation
      if(0.0 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1.0){

        // Positive distance or Negative distance
        if(0 <= fr_distance_m){
          distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
        }
        else{
          distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
        }
      }
      else printf("Odometry ERROR\n");

      // Change euler (old_odom)
      quaternionMsgToTF(old_odom.pose.pose.orientation, quat_tf);
      tf::Matrix3x3(quat_tf).getRPY(old_odom_orientation_x, old_odom_orientation_y, old_odom_orientation_z);

      // Add the amount of movement to the odometry
      rotational_position_deg = (distance_m * 360.) / (SOBIT_CAR_DIAMETER * PAI);
      rotational_position_rad =  old_odom_orientation_z + (rotational_position_deg * (PAI / 180.)); // rad = deg * (PAI / 180.)

      // Change quaternion (calculation_odom)
      tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0.0, 0.0, rotational_position_rad);
      quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

      *result_odom = calculation_odom;

      return true;
    }

    // Other motion
    default:{
      *result_odom = old_odom;
      return true;
    }
  }
}

// Distance calculation
float SobitProOdometry::distance_calculation(float wheel_present_position){
  float distance_m;

  distance_m = WHEEL_LENGTH * wheel_present_position / 4096.;

  return distance_m;
}

// Position calculation
float SobitProOdometry::position_calculation(float steer_present_position){
  float direction_deg;

  direction_deg = (steer_present_position - 2048) * 360. / 4096.;

  return direction_deg;
}

void SobitProOdometry::pose_broadcaster(nav_msgs::Odometry tf_odom){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion tf_quat;

  transform.setOrigin(tf::Vector3(tf_odom.pose.pose.position.x, tf_odom.pose.pose.position.y, tf_odom.pose.pose.position.z));
  quaternionMsgToTF(tf_odom.pose.pose.orientation, tf_quat);
  transform.setRotation(tf_quat);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}