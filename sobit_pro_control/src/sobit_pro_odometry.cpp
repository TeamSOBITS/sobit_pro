#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

// #include <iostream>

// Calculate Odometry
nav_msgs::msg::Odometry SobitProOdometry::odom(
  
  double steer_fl_curt_pos, double steer_fr_curt_pos,
  double steer_bl_curt_pos, double steer_br_curt_pos,
  double wheel_fl_curt_pos, double wheel_fr_curt_pos,
  double wheel_bl_curt_pos, double wheel_br_curt_pos,
  double wheel_fl_prev_pos, double wheel_fr_prev_pos,
  double wheel_bl_prev_pos, double wheel_br_prev_pos,
  nav_msgs::msg::Odometry prev_odom)
{
  nav_msgs::msg::Odometry result_odom;

  // get the movement of each wheel[m]
  std::vector<double> distance_m(4);
  distance_m[0] = distance_calculation(wheel_fl_curt_pos - wheel_fl_prev_pos);
  distance_m[1] = distance_calculation(wheel_fr_curt_pos - wheel_fr_prev_pos);
  distance_m[2] = distance_calculation(wheel_bl_curt_pos - wheel_bl_prev_pos);
  distance_m[3] = distance_calculation(wheel_br_curt_pos - wheel_br_prev_pos);

  // Transform to Roll, Pitch and Yaw from prev_odom
  tf2::Quaternion quat_tf;
  double prev_roll, prev_pitch, prev_yaw;
  tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

  // 
  std::vector<double> direction_rad(4);
  direction_rad[0] = steer_fl_curt_pos + ( 3./4.)*M_PI;
  direction_rad[1] = steer_fr_curt_pos + ( 1./4.)*M_PI;
  direction_rad[2] = steer_bl_curt_pos + (-3./4.)*M_PI;
  direction_rad[3] = steer_br_curt_pos + (-1./4.)*M_PI;

  // each wheel point from robot base
  std::vector<geometry_msgs::msg::Point> wheels_point(4);
  wheels_point[0].x = wheels_point[1].x = SobitProControl::TRACK / 2.;        // X position of front wheel is (+)
  wheels_point[2].x = wheels_point[3].x = SobitProControl::TRACK / 2. * (-1); // X position of  back wheel is (-)
  wheels_point[0].y = wheels_point[2].y = SobitProControl::TRACK / 2.;        // Y position of  left wheel is (+)
  wheels_point[1].y = wheels_point[3].y = SobitProControl::TRACK / 2. * (-1); // Y position of right wheel is (i)

  // calculate the base_center. base_center is center in SWEVEL Motion's robot movement circle
  // 4C2 = 6 -> i:j=[0~5]
  double diff_x = 0.,diff_y = 0., diff_yaw = 0.;
  int normalize = ((wheels_point.size()*(wheels_point.size()-1)) / (2.*1.));
  for (size_t i=0; i<wheels_point.size()-1; i++) {
    for (size_t j=i+1; j<wheels_point.size(); j++) {

      double x=0., y=0., yaw=0.;
      geometry_msgs::msg::Point base_center;
      double wheel_rad_1 = direction_rad[i] + ((0. < distance_m[i]) ? 0. : M_PI);
      double wheel_rad_2 = direction_rad[j] + ((0. < distance_m[j]) ? 0. : M_PI);
      double wheel_m_1 = fabsf(distance_m[i]);
      double wheel_m_2 = fabsf(distance_m[j]);
      if (1. - fabsf(cos(wheel_rad_2 - wheel_rad_1)) < 0.001) { // near the parallel of direction_rad[j] and direction_rad[i]
        if ((fabsf(wheel_m_2 - wheel_m_1) < 0.001) && (0. < cos(wheel_rad_2 - wheel_rad_1))) { // Translational motion
          double delta = wheel_rad_2 - wheel_rad_1 - M_PI*((int)((wheel_rad_2 - wheel_rad_1)/M_PI));
          delta -= M_PI * ((int)(delta/(M_PI/2.)));
          x = (wheel_m_1 + wheel_m_2)/2. * cos(wheel_rad_1 + delta/2.);
          y = (wheel_m_1 + wheel_m_2)/2. * sin(wheel_rad_1 + delta/2.);
          yaw = 0.;
          base_center.x = base_center.y = INFINITY;
        } else {
          int ie = (0. < cos(wheel_rad_2 - wheel_rad_1)) ? -1.: 1.; //internally or externally divide
          base_center.x = (wheel_m_2*ie * wheels_point[i].x + wheel_m_1 * wheels_point[j].x) / (wheel_m_1 + wheel_m_2*ie);
          base_center.y = (wheel_m_2*ie * wheels_point[i].y + wheel_m_1 * wheels_point[j].y) / (wheel_m_1 + wheel_m_2*ie);
        }
      } else {
        double a1 = tan(wheel_rad_1 + M_PI/2.);
        double a2 = tan(wheel_rad_2 + M_PI/2.);
        base_center.x = ((a1*wheels_point[i].x - a2*wheels_point[j].x) - (wheels_point[i].y - wheels_point[j].y)) / (a1 - a2);
        base_center.y = a1 * (base_center.x - wheels_point[i].x) + wheels_point[i].y;
      }

      // 
      if ((std::isfinite(base_center.x)) || (std::isfinite(base_center.y))) {
        // 
        double dist_base_wheel_1 = sqrtf(powf((wheels_point[i].x - base_center.x), 2.) + powf((wheels_point[i].y - base_center.y), 2.));
        double dist_base_wheel_2 = sqrtf(powf((wheels_point[j].x - base_center.x), 2.) + powf((wheels_point[j].y - base_center.y), 2.));
        double pn1, pn2;
        pn1 = pn2 = 0.5;

        if (dist_base_wheel_1 < 0.001) {
          pn1 = 0;
          pn2 = 1.;
          dist_base_wheel_1 = 1.; // dummy of zero devided...
        }
        if (dist_base_wheel_2 < 0.001) {
          pn2 = 0;
          pn1 = 1.;
          dist_base_wheel_2 = 1.; // dummy of zero devided...
        }
        
        if (cos(atan2(base_center.y-wheels_point[i].y, base_center.x-wheels_point[i].x) - direction_rad[i] - M_PI/2.) < 0.) 
          pn1 = -1 * pn1;
        if (cos(atan2(base_center.y-wheels_point[j].y, base_center.x-wheels_point[j].x) - direction_rad[j] - M_PI/2.) < 0.) 
          pn2 = -1 * pn2;

        // 
        yaw = distance_m[i] * pn1 / dist_base_wheel_1 + distance_m[j] * pn2 / dist_base_wheel_2;

        // 
        x = base_center.x + (-base_center.x) * cos(yaw) - (-base_center.y) * sin(yaw);
        y = base_center.y + (-base_center.x) * sin(yaw) + (-base_center.y) * cos(yaw);

      }

      // 
      if (!std::isfinite(x)  ) x   = 0.;
      if (!std::isfinite(y)  ) y   = 0.;
      if (!std::isfinite(yaw)) yaw = 0.;

      // 
      diff_x   += x   / normalize;
      diff_y   += y   / normalize;
      diff_yaw += yaw / normalize;
    }
  }

  // Update the Odometry
  result_odom.pose.pose.position.x = prev_odom.pose.pose.position.x + 
      diff_x * cos(prev_yaw) - diff_y * sin(prev_yaw);
  result_odom.pose.pose.position.y = prev_odom.pose.pose.position.y + 
      diff_x * sin(prev_yaw) + diff_y * cos(prev_yaw);
  result_odom.pose.pose.position.z = prev_odom.pose.pose.position.z;

  // Change quaternion
  quat_tf.setRPY(0., 0., (prev_yaw + diff_yaw));
  tf2::convert(quat_tf, result_odom.pose.pose.orientation);

  return result_odom;
}

// Distance calculation
double SobitProOdometry::distance_calculation(double wheel_delta_pos) {
  return SobitProControl::WHEEL_DIAMETER/2. * wheel_delta_pos;
}

// Pose broadcaster (Generate a pose from Odometry)
void SobitProOdometry::pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom) {
  geometry_msgs::msg::TransformStamped transformStamped;

  std::string robot_name =
      (std::strcmp(node_->get_namespace(), "/") != 0)
      ? std::string(node_->get_namespace()).substr(1) + "/"
      : "";

  // transformStamped.header.stamp         = node_->get_clock()->now();
  // transformStamped.header.frame_id      = robot_name + "odom";
  // transformStamped.child_frame_id       = robot_name + "base_footprint";
  transformStamped.header          = tf_odom.header;
  transformStamped.child_frame_id  = tf_odom.child_frame_id;

  transformStamped.transform.translation.x = tf_odom.pose.pose.position.x;
  transformStamped.transform.translation.y = tf_odom.pose.pose.position.y;
  transformStamped.transform.translation.z = tf_odom.pose.pose.position.z;

  transformStamped.transform.rotation      = tf_odom.pose.pose.orientation;
  // transformStamped.transform.rotation.x    = tf_odom.pose.pose.orientation.x;
  // transformStamped.transform.rotation.y    = tf_odom.pose.pose.orientation.y;
  // transformStamped.transform.rotation.z    = tf_odom.pose.pose.orientation.z;
  // transformStamped.transform.rotation.w    = tf_odom.pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(transformStamped);
}