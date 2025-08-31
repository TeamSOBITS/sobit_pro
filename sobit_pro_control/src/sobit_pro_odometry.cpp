#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

#include <iostream>

// namespace {
//   int g_trans_total = 0;
//   int g_trans_ok = 0;
//   int g_trans_skip = 0;
// }

// Calculate Odometry
bool SobitProOdometry::odom(
  
  double steer_fl_curt_pos, double steer_fr_curt_pos,
  double steer_bl_curt_pos, double steer_br_curt_pos,
  double wheel_fl_curt_pos, double wheel_fr_curt_pos,
  double wheel_bl_curt_pos, double wheel_br_curt_pos,
  double wheel_fl_init_pos, double wheel_fr_init_pos,
  double wheel_bl_init_pos, double wheel_br_init_pos,
  double prev_motion,
  nav_msgs::msg::Odometry prev_odom, nav_msgs::msg::Odometry* result_odom)
{
  double fl_distance_m    = distance_calculation(wheel_fl_curt_pos - wheel_fl_init_pos); // Calculation distance[m]
  double fr_distance_m    = distance_calculation(wheel_fr_curt_pos - wheel_fr_init_pos); // Calculation distance[m]
  double bl_distance_m    = distance_calculation(wheel_bl_curt_pos - wheel_bl_init_pos); // Calculation distance[m]
  double br_distance_m    = distance_calculation(wheel_br_curt_pos - wheel_br_init_pos); // Calculation distance[m]

  // double fl_direction_deg = steer_fl_curt_pos / (M_PI/180.); // Record present position
  // double fr_direction_deg = steer_fr_curt_pos / (M_PI/180.); // Record present position
  // double bl_direction_deg = steer_bl_curt_pos / (M_PI/180.); // Record present position
  // double br_direction_deg = steer_br_curt_pos / (M_PI/180.); // Record present position



  double prev_roll = 0., prev_pitch = 0., prev_yaw = 0.;
  double distance_m = 0.;
  nav_msgs::msg::Odometry calculation_odom = *result_odom;
  tf2::Quaternion quat_tf;

  if      (prev_motion == STOP_MOTION_MODE )          motion_mode = STOP_MOTION_MODE;
  else if (prev_motion == TRANSLATIONAL_MOTION_MODE ) motion_mode = TRANSLATIONAL_MOTION_MODE;
  else if (prev_motion == ROTATIONAL_MOTION_MODE )    motion_mode = ROTATIONAL_MOTION_MODE;
  else if (prev_motion == SWIVEL_MOTION_MODE )        motion_mode = SWIVEL_MOTION_MODE;


  switch (motion_mode) {
    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{

      // Made into delta in order to track whow much of odometry is accepted, right now it is 14 degrees but needs adjustment
      double delta = (steer_fr_curt_pos + M_PI/4.) - (steer_bl_curt_pos - M_PI*3./4.);
      if (fr_distance_m * bl_distance_m < 0.) delta -= M_PI*delta/fabsf(delta);
      // const bool accepted_for_odom = (delta <= 14.0);

      double direction = 0.;
      if (fabsf(delta) <= 14.*(M_PI/180.)) {
        //
        direction = (steer_bl_curt_pos - M_PI*3./4.) + delta/2.;
        if (bl_distance_m < 0.) {
          direction += M_PI;
        }

        // 
        distance_m = (fabsf(fr_distance_m) + fabsf(bl_distance_m)) / 2.;
      }

      // Transform euler to RPY (prev_odom)
      tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
      tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

      // Add the amount of movement to the odometry
      calculation_odom.pose.pose.position.x =
          prev_odom.pose.pose.position.x
          + distance_m * cosf(direction + prev_yaw);
          // + distance_m * cosf(direction) * cosf(prev_yaw)
          // + distance_m * sinf(direction) * cosf(prev_yaw + M_PI_2);
      calculation_odom.pose.pose.position.y =
          prev_odom.pose.pose.position.y
          + distance_m * sinf(direction + prev_yaw);
          // + distance_m * cosf(direction) * sinf(prev_yaw)
          // + distance_m * sinf(direction) * sinf(prev_yaw + M_PI_2);
      calculation_odom.pose.pose.position.z = prev_odom.pose.pose.position.z; 

      calculation_odom.pose.pose.orientation = prev_odom.pose.pose.orientation;
      *result_odom = calculation_odom;


      return true;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE:{
      // Transform euler->RPY (prev_odom)
      tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
      tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

      double yaw = 0.;
      yaw = 
          ((fl_distance_m + fr_distance_m + bl_distance_m + br_distance_m) / 4.)
          / (SobitProControl::TRACK / sqrtf(2.));

      // Transform quaternion->msg (calculation_odom)
      quat_tf.setRPY(0., 0., (prev_yaw + yaw));
      tf2::convert(quat_tf, calculation_odom.pose.pose.orientation);

      *result_odom = calculation_odom;


      return true;
    }

    // Swivel motion
    case SWIVEL_MOTION_MODE:{
      // Transform euler->RPY (prev_odom)
      tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
      tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
      
      double pose_x = 0., pose_y = 0., yaw = 0.;
      geometry_msgs::msg::Point wheel_point_fl, wheel_point_fr, wheel_point_bl, wheel_point_br;

      wheel_point_fl.x = SobitProControl::TRACK / 2.;
      wheel_point_fl.y = SobitProControl::TRACK / 2.;
      wheel_point_fr.x = SobitProControl::TRACK / 2.;
      wheel_point_fr.y = SobitProControl::TRACK / 2. * (-1);
      wheel_point_bl.x = SobitProControl::TRACK / 2. * (-1);
      wheel_point_bl.y = SobitProControl::TRACK / 2.;
      wheel_point_br.x = SobitProControl::TRACK / 2. * (-1);
      wheel_point_br.y = SobitProControl::TRACK / 2. * (-1);

      double a_fl = tanf(atan2f(wheel_point_fl.y, wheel_point_fl.x) + steer_fl_curt_pos);
      double a_fr = tanf(atan2f(wheel_point_fr.y, wheel_point_fr.x) + steer_fr_curt_pos);
      double a_bl = tanf(atan2f(wheel_point_bl.y, wheel_point_bl.x) + steer_bl_curt_pos);
      double a_br = tanf(atan2f(wheel_point_br.y, wheel_point_br.x) + steer_br_curt_pos);

      geometry_msgs::msg::Point base_center;

      if (fabsf(a_fr - a_bl) > fabsf(a_fl - a_br)) {
        // fr and bl
        base_center.x = 
            (a_fr * wheel_point_fr.x - a_bl * wheel_point_bl.x
            + wheel_point_bl.y - wheel_point_fr.y)
            / (a_fr - a_bl);
        base_center.y =
            a_fr
            * (base_center.x - wheel_point_fr.x)
            + wheel_point_fr.y;
        
        if (sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.))) {
          yaw = fr_distance_m / sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.));
        } else {
          yaw = bl_distance_m / sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.));
        }
      } else {
        // fl and br
        base_center.x = (a_fl * wheel_point_fl.x - a_br * wheel_point_br.x + wheel_point_br.y - wheel_point_fl.y) / (a_fl - a_br);
        base_center.y = a_fl * (base_center.x - wheel_point_fl.x) + wheel_point_fl.y;

        if (sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.))) {
          yaw = fl_distance_m / sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.));
        } else {
          yaw = br_distance_m / sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.));
        }
      }

      pose_x =
          (0. - base_center.x) * cosf(yaw)
          - (0. - base_center.y) * sinf(yaw)
          + base_center.x;
      pose_y =
          (0. - base_center.x) * sinf(yaw)
          + (0. - base_center.y) * cosf(yaw)
          + base_center.y;
      
      if ( std::isnan(pose_x) ) pose_x = 0.;
      if ( std::isnan(pose_y) ) pose_y = 0.;
      if ( std::isnan(yaw) )    yaw = 0.;

      calculation_odom.pose.pose.position.x =
          pose_x * cosf(prev_yaw) - pose_y * sinf(prev_yaw)
          + prev_odom.pose.pose.position.x;
      calculation_odom.pose.pose.position.y =
          pose_x * sinf(prev_yaw) + pose_y * cosf(prev_yaw)
          + prev_odom.pose.pose.position.y;
      calculation_odom.pose.pose.position.z = prev_odom.pose.pose.position.z;

      // Change quaternion (calculation_odom)
      quat_tf.setRPY(0., 0., (prev_yaw + yaw));
      tf2::convert(quat_tf, calculation_odom.pose.pose.orientation);

      *result_odom = calculation_odom;


      return true;
    }

    // Other motion
    default:{
      *result_odom = prev_odom;
      return true;
    }
  }
}

// Distance calculation
double SobitProOdometry::distance_calculation(double wheel_curt_pos){
  return SobitProControl::WHEEL_DIAMETER/2 * wheel_curt_pos;
}

// Position calculation
double SobitProOdometry::position_calculation(double steer_curt_pos){
  return steer_curt_pos / (M_PI / 180.);
}

// Pose broadcaster (Generate a pose from Odometry)
void SobitProOdometry::pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom) {
  geometry_msgs::msg::TransformStamped transformStamped;

  std::string robot_name =
      (std::strcmp(node_->get_namespace(), "/") != 0)
      ? std::string(node_->get_namespace()).substr(1) + "/"
      : "";

  transformStamped.header.stamp         = node_->get_clock()->now();
  transformStamped.header.frame_id      = robot_name + "odom";
  transformStamped.child_frame_id       = robot_name + "base_footprint";

  transformStamped.transform.translation.x = tf_odom.pose.pose.position.x;
  transformStamped.transform.translation.y = tf_odom.pose.pose.position.y;
  transformStamped.transform.translation.z = tf_odom.pose.pose.position.z;

  transformStamped.transform.rotation.x    = tf_odom.pose.pose.orientation.x;
  transformStamped.transform.rotation.y    = tf_odom.pose.pose.orientation.y;
  transformStamped.transform.rotation.z    = tf_odom.pose.pose.orientation.z;
  transformStamped.transform.rotation.w    = tf_odom.pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(transformStamped);
}