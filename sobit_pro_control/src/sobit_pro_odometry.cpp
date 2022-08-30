#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Odometry calculation
bool SobitProOdometry::odom(float steer_fr_present_position, float steer_fl_present_position, float wheel_fr_present_position, float wheel_fl_present_position, 
                            int32_t wheel_fr_initial_position, int32_t wheel_fl_initial_position, int32_t old_motion, nav_msgs::Odometry old_odom, nav_msgs::Odometry* result_odom){

  float fr_distance_m = distance_calculation(wheel_fr_present_position - wheel_fr_initial_position); // Calculation distance[m]
  float fl_distance_m = distance_calculation(wheel_fl_present_position - wheel_fl_initial_position); // Calculation distance[m]
  float fr_direction_deg = position_calculation(steer_fr_present_position); // Record present position
  float fl_direction_deg = position_calculation(steer_fl_present_position); // Record present position
  double old_odom_orientation_x = 0., old_odom_orientation_y = 0., old_odom_orientation_z = 0.;
  float distance_m;
  nav_msgs::Odometry calculation_odom;
  tf::Quaternion quat_tf;
  calculation_odom = *result_odom;

  if(old_motion == 1){
    motion_mode = TRANSLATIONAL_MOTION_MODE;
  }
  else if(old_motion ==2){
    motion_mode = ROTATIONAL_MOTION_MODE;
  }
  else if(old_motion ==3){
    motion_mode = SWIVEL_MOTION_MODE;
  }

  switch(motion_mode){

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{

      // Convert Wheel Coordinates to Robot Coordinates
      fr_direction_deg = fr_direction_deg + 45.;
      fl_direction_deg = fl_direction_deg - 45.;

      // Check the calculation
      if(0.0 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1.0){

        // Positive distance or Negative distance
        if(-45 <= fr_direction_deg && fr_direction_deg <= 90){
          if(0 <= fr_distance_m){
            distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
          else{
            distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
        }
        else if(90 < fr_direction_deg && fr_direction_deg <= 135){
          if(0 <= fr_distance_m){
            distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
          else{
            distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
          }
        }
        else ROS_ERROR("Calculation ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
      }
      else ROS_ERROR("Odometry ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);

      // Change euler (old_odom)
      quaternionMsgToTF(old_odom.pose.pose.orientation, quat_tf);
      tf::Matrix3x3(quat_tf).getRPY(old_odom_orientation_x, old_odom_orientation_y, old_odom_orientation_z);

      // Add the amount of movement to the odometry
      calculation_odom.pose.pose.position.x = old_odom.pose.pose.position.x + distance_m * cos(fr_direction_deg * (PAI / 180.)) * cos(old_odom_orientation_z)
                                                                            + distance_m * sin(fr_direction_deg * (PAI / 180.)) * cos(old_odom_orientation_z + 1.5708);

      calculation_odom.pose.pose.position.y = old_odom.pose.pose.position.y + distance_m * cos(fr_direction_deg * (PAI / 180.)) * sin(old_odom_orientation_z)
                                                                            + distance_m * sin(fr_direction_deg * (PAI / 180.)) * sin(old_odom_orientation_z + 1.5708);

      calculation_odom.pose.pose.orientation = old_odom.pose.pose.orientation;


      // debug
      if (std::isnan(calculation_odom.pose.pose.position.x) || std::isnan(calculation_odom.pose.pose.position.y)) ROS_ERROR("------ Odom calcuration : Nan error !! : Transition motion pose ------");
      if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) ROS_ERROR("------ Odom calcuration : Nan error !! : Transition motion orientation------");

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
      else ROS_ERROR("Odometry ERROR : Rotational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);

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

    // Swivel motion
    case SWIVEL_MOTION_MODE:{
      float svl_orientation_rad;
      float svl_r, svl_r_in, svl_r_out, svl_deg, svl_dis;
      float pose_x = 0., pose_y = 0., ori_z =0.0;
      float d = SOBIT_CAR_DIAMETER / 2. * cos(PAI / 4.);  // Manhattan distance (cos value shud be change)

      // Change euler (old_odom)
      quaternionMsgToTF(old_odom.pose.pose.orientation, quat_tf);
      tf::Matrix3x3(quat_tf).getRPY(old_odom_orientation_x, old_odom_orientation_y, old_odom_orientation_z);

      if( ( fl_direction_deg >= -45.0 && fl_direction_deg < 0 ) && ( fr_direction_deg >= -22.5 && fr_direction_deg < 0 )){
        svl_r_in = d / cos((45. + fl_direction_deg) * PAI / 180.);
        svl_r_out = d / cos((45. - fr_direction_deg) * PAI / 180.);
        svl_r = (svl_r_out * sin((45. - fr_direction_deg) * PAI / 180.) - svl_r_in * sin((45. + fl_direction_deg) * PAI / 180.)) / 2.;
        // ROS_ERROR("fl_deg    : %f\n", fl_direction_deg);
        // ROS_ERROR("fr_deg    : %f\n", fr_direction_deg);
        // ROS_ERROR("svl_r_in  : %f\n", svl_r_in);
        // ROS_ERROR("svl_r_out : %f\n", svl_r_out);
        // ROS_ERROR("svl_r     : %f\n", svl_r);
        svl_deg = (std::abs(fl_distance_m) / (2. * svl_r_in * PAI) * 360. + std::abs(fr_distance_m) / (2. * svl_r_out * PAI) * 360.) / 2.;
        svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * PAI / 180.)));
        if(fr_distance_m >= 0){
          pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          ori_z = svl_deg;
        }
        else{
          pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          ori_z = -svl_deg;
        }
      }
      else if(fl_direction_deg >= 0 && fl_direction_deg < 22.5 && fr_direction_deg >= 0 && fr_direction_deg < 45.0){
        svl_r_in = d / cos((45. - fr_direction_deg) * PAI / 180.);
        svl_r_out = d / cos((45. + fl_direction_deg) * PAI / 180.);
        svl_r = (svl_r_out * sin((45. + fl_direction_deg) * PAI / 180.) - svl_r_in * sin((45. - fr_direction_deg) * PAI / 180.)) / 2.;
        // ROS_ERROR("fl_deg    : %f\n", fl_direction_deg);
        // ROS_ERROR("fr_deg    : %f\n", fr_direction_deg);
        // ROS_ERROR("svl_r_in  : %f\n", svl_r_in);
        // ROS_ERROR("svl_r_out : %f\n", svl_r_out);
        // ROS_ERROR("svl_r : %f\n", svl_r);
        svl_deg = (std::abs(fr_distance_m) / (2. * svl_r_in * PAI) * 360. + std::abs(fl_distance_m) / (2. * svl_r_out * PAI) * 360.) / 2.;
        svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * PAI / 180.)));
        // ROS_ERROR("svl_deg : %f\n", svl_deg);
        // ROS_ERROR("svl_dis : %f\n", svl_dis);
        if(fr_distance_m >= 0){
          pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          ori_z = svl_deg;
        }
        else{
          pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
          ori_z = -svl_deg;
        }
      }

      else if(abs(fr_direction_deg) <= 45 && abs(fl_direction_deg) >= 45){
        if(fl_direction_deg > -45){
          fl_direction_deg += 45;
          fr_direction_deg += 135.;
          }
        else{
          fl_direction_deg += 225;
          fr_direction_deg += 135.;
        }

        svl_r_in = d / sin((fl_direction_deg - 90.) * PAI / 180.);
        svl_r_out = d / sin((fr_direction_deg - 90.) * PAI / 180.);
        svl_r = (svl_r_in + svl_r_out) / 2.;
        svl_deg = (std::abs(fl_distance_m) / (2. * svl_r_in * PAI) * 360. + std::abs(fr_distance_m) / (2. * svl_r_out * PAI) * 360.) / 2.;
        svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * PAI / 180.)));

        if(90 <= fl_direction_deg && fl_direction_deg < 135){
          if(fl_distance_m <= 0){
            pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = svl_deg;
          }
          else{
            pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = -svl_deg;
          }
        }
        else if(135 <= fl_direction_deg && fl_direction_deg <= 180){
          if(fl_distance_m >= 0){
            pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = svl_deg;
          }
          else{
            pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = -svl_deg;
          }
        }
        else ROS_ERROR("Calculation ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
      }
      else if(abs(fr_direction_deg) >= 45 && abs(fl_direction_deg) <= 45){
        if(fr_direction_deg < 45){
          fr_direction_deg = 45 - fr_direction_deg;
          fl_direction_deg = 135 - fl_direction_deg;
        }
        else{
          fr_direction_deg = 225 - fr_direction_deg;
          fl_direction_deg = 135 - fl_direction_deg;
        }

        svl_r_in = d / sin((fr_direction_deg - 90.) * PAI / 180.);
        svl_r_out = d / sin((fl_direction_deg - 90.) * PAI / 180.);
        svl_r = (svl_r_in + svl_r_out) / 2.;
        svl_deg = (std::abs(fr_distance_m) / (2. * svl_r_in * PAI) * 360. + std::abs(fl_distance_m) / (2. * svl_r_out * PAI) * 360.) / 2.;
        svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * PAI / 180.)));
        if(90 <= fr_direction_deg && fr_direction_deg < 135){
          if(fr_distance_m >= 0){
            pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = -svl_deg;
          }
          else{
            pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = svl_deg;
          }
        }
        else if(135 <= fr_direction_deg && fr_direction_deg <= 180){
          if(fr_distance_m >= 0){
            pose_x = -svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = -svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = svl_deg;
          }
          else{
            pose_x = svl_dis * sin((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            pose_y = svl_dis * cos((180 - svl_deg) / 2 * PAI / 180. - old_odom_orientation_z);
            ori_z = -svl_deg;
          }
        }
        else ROS_ERROR("Calculation ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
      }
      else{
          ROS_ERROR("Odometry ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
          pose_x = 0.; pose_y = 0.; ori_z = 0.;
          *result_odom = old_odom;
          return true;
      }
      // debug
      if (std::isnan(pose_x) || std::isnan(pose_y) || std::isnan(ori_z)) {
        ROS_ERROR("------ Odom calcuration : Nan error !! : Swivel motion ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", pose_x, pose_y, ori_z );
        *result_odom = old_odom;
        return true;
      }
      if (std::isinf(pose_x) || std::isinf(pose_y) || std::isinf(ori_z)) {
        ROS_ERROR("------ Odom calcuration : Inf error !! : Swivel motion ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", pose_x, pose_y, ori_z );
        *result_odom = old_odom;
        return true;
      }

      // Add the amount of movement to the odometry
      calculation_odom.pose.pose.position.x = old_odom.pose.pose.position.x + pose_x;
      calculation_odom.pose.pose.position.y = old_odom.pose.pose.position.y + pose_y;
      calculation_odom.pose.pose.position.z = 0.0;
      svl_orientation_rad =  old_odom_orientation_z + (ori_z * (PAI / 180.)); // rad = deg * (PAI / 180.)

      // debug
      if (std::isnan(calculation_odom.pose.pose.position.x) || std::isnan(calculation_odom.pose.pose.position.y) || std::isnan(svl_orientation_rad)) {
        ROS_ERROR("------ Odom calcuration : Nan error !! : Swivel motion(calculation_odom) ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, svl_orientation_rad );
        *result_odom = old_odom;
        return true;
      }
      if (std::isinf(calculation_odom.pose.pose.position.x) || std::isinf(calculation_odom.pose.pose.position.y) || std::isinf(svl_orientation_rad)) {
        ROS_ERROR("------ Odom calcuration : Inf error !! : Swivel motion(calculation_odom) ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, svl_orientation_rad );
        *result_odom = old_odom;
        return true;
      }
      // Change quaternion (calculation_odom)
      tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0.0, 0.0, svl_orientation_rad);
      quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

      // debug
      if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) {
        ROS_ERROR("------ Odom calcuration : Nan error !! : Swivel motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
        *result_odom = old_odom;
        return true;
      }
      if (std::isinf(calculation_odom.pose.pose.orientation.x) || std::isinf(calculation_odom.pose.pose.orientation.y) || std::isinf(calculation_odom.pose.pose.orientation.z) || std::isinf(calculation_odom.pose.pose.orientation.w)) {
        ROS_ERROR("------ Odom calcuration : Inf error !! : Swivel motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
        *result_odom = old_odom;
        return true;
      }
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

// Pose broadcaster(Generate a pose from Odometry)
void SobitProOdometry::pose_broadcaster(nav_msgs::Odometry tf_odom){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion tf_quat;

  transform.setOrigin(tf::Vector3(tf_odom.pose.pose.position.x, tf_odom.pose.pose.position.y, tf_odom.pose.pose.position.z));
  quaternionMsgToTF(tf_odom.pose.pose.orientation, tf_quat);
  transform.setRotation(tf_quat);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}