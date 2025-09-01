#include "sobit_pro_control/sobit_pro_control.hpp"

// void SobitProControl::setParams( geometry_msgs::msg::Twist vel_twist )
void SobitProControl::setParams(const geometry_msgs::msg::Twist vel_twist)
{
  switch (motion_mode) {
    // Stop motion
    case STOP_MOTION_MODE: {
      wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = 0.;
      break;
    }

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{
      // Goal velocity calculation
      double vel_ms    = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // vel_twist [m/s] to vel_ms [m/s]
      double vel_rads  = vel_ms / (WHEEL_DIAMETER/2.); // vel_ms   [m/s]   to vel_rads  [rad/s]

      // Goal position calculation
      double goal_rad = atan2(vel_twist.linear.y, vel_twist.linear.x);

      steer_fl_goal_pos = goal_rad - ((3./4.)*M_PI);
      steer_fr_goal_pos = goal_rad - ((1./4.)*M_PI);
      steer_bl_goal_pos = goal_rad - ((3./4.)*M_PI*(-1));
      steer_br_goal_pos = goal_rad - ((1./4.)*M_PI*(-1));

      // Direction of wheel rotation
      if (vel_rads > LIMIT_VEL_RADS) wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = LIMIT_VEL_RADS;
      else                           wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = vel_rads;

      // 
      steer_fl_goal_pos = steer_fl_goal_pos - (2*M_PI) * ((int)(steer_fl_goal_pos / (2*M_PI)));
      steer_fr_goal_pos = steer_fr_goal_pos - (2*M_PI) * ((int)(steer_fr_goal_pos / (2*M_PI)));
      steer_bl_goal_pos = steer_bl_goal_pos - (2*M_PI) * ((int)(steer_bl_goal_pos / (2*M_PI)));
      steer_br_goal_pos = steer_br_goal_pos - (2*M_PI) * ((int)(steer_br_goal_pos / (2*M_PI)));

      // 
      if (M_PI < fabsf(steer_fl_goal_pos)) steer_fl_goal_pos -= 2*M_PI*steer_fl_goal_pos/fabsf(steer_fl_goal_pos);
      if (M_PI < fabsf(steer_fr_goal_pos)) steer_fr_goal_pos -= 2*M_PI*steer_fr_goal_pos/fabsf(steer_fr_goal_pos);
      if (M_PI < fabsf(steer_bl_goal_pos)) steer_bl_goal_pos -= 2*M_PI*steer_bl_goal_pos/fabsf(steer_bl_goal_pos);
      if (M_PI < fabsf(steer_br_goal_pos)) steer_br_goal_pos -= 2*M_PI*steer_br_goal_pos/fabsf(steer_br_goal_pos);

      // 
      if (M_PI/2. < fabsf(steer_fl_goal_pos)) {
        steer_fl_goal_pos -= M_PI*steer_fl_goal_pos/fabsf(steer_fl_goal_pos);
        wheel_fl_goal_vel *= -1;
      }
      if (M_PI/2. < fabsf(steer_fr_goal_pos)) {
        steer_fr_goal_pos -= M_PI*steer_fr_goal_pos/fabsf(steer_fr_goal_pos);
        wheel_fr_goal_vel *= -1;
      }
      if (M_PI/2. < fabsf(steer_bl_goal_pos)) {
        steer_bl_goal_pos -= M_PI*steer_bl_goal_pos/fabsf(steer_bl_goal_pos);
        wheel_bl_goal_vel *= -1;
      }
      if (M_PI/2. < fabsf(steer_br_goal_pos)) {
        steer_br_goal_pos -= M_PI*steer_br_goal_pos/fabsf(steer_br_goal_pos);
        wheel_br_goal_vel *= -1;
      }

      break;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE:{
      // Goal velocity calculation
      double vel_ms    = vel_twist.angular.z * (BODY_DIAMETER/2.); // vel_deg  [deg/s] to vel_ms    [m/s]
      double vel_rads  = vel_ms / (WHEEL_DIAMETER/2.);             // vel_ms   [m/s]   to vel_rads  [rad/s]

      // Goal angle calculation
      steer_fl_goal_pos = steer_fr_goal_pos = steer_bl_goal_pos = steer_br_goal_pos = 0.;

      // Velocity of wheel
      if (vel_twist.angular.z < 0.) {
        if (vel_rads < -LIMIT_VEL_RADS) wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = -LIMIT_VEL_RADS;
        else                            wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = vel_rads;
      } else {
        if (vel_rads > LIMIT_VEL_RADS)  wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = LIMIT_VEL_RADS;
        else                            wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = vel_rads;
      }

      break;
    }

    //Swivel motion
    case SWIVEL_MOTION_MODE:{
      double base_vel = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.));
      double r = base_vel / fabsf(vel_twist.angular.z);
      double base_angle = atan2(vel_twist.linear.y , vel_twist.linear.x);

      geometry_msgs::msg::Point base_center;
      base_center.x = r * cosf(base_angle + (M_PI/2.) * (vel_twist.angular.z/fabsf(vel_twist.angular.z)));
      base_center.y = r * sinf(base_angle + (M_PI/2.) * (vel_twist.angular.z/fabsf(vel_twist.angular.z)));

      geometry_msgs::msg::Point wheel_point_fl, wheel_point_fr, wheel_point_bl, wheel_point_br;
      wheel_point_fl.x = TRACK / 2.;
      wheel_point_fl.y = TRACK / 2.;
      wheel_point_fr.x = TRACK / 2.;
      wheel_point_fr.y = TRACK / 2. * (-1);
      wheel_point_bl.x = TRACK / 2. * (-1);
      wheel_point_bl.y = TRACK / 2.;
      wheel_point_br.x = TRACK / 2. * (-1);
      wheel_point_br.y = TRACK / 2. * (-1);

      // TODO: improve code readability
      double r_wheel_fl, r_wheel_fr, r_wheel_bl, r_wheel_br;
      r_wheel_fl = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2.*(TRACK / sqrtf(2.))*r*(((wheel_point_fl.x * base_center.x) + (wheel_point_fl.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
      r_wheel_fr = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2.*(TRACK / sqrtf(2.))*r*(((wheel_point_fr.x * base_center.x) + (wheel_point_fr.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
      r_wheel_bl = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2.*(TRACK / sqrtf(2.))*r*(((wheel_point_bl.x * base_center.x) + (wheel_point_bl.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
      r_wheel_br = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2.*(TRACK / sqrtf(2.))*r*(((wheel_point_br.x * base_center.x) + (wheel_point_br.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));

      geometry_msgs::msg::Point wheel_base_fl, wheel_base_fr, wheel_base_bl, wheel_base_br;
      double wheel_to_base_dist = 1.3;
      wheel_base_fl.x = wheel_point_fl.x * wheel_to_base_dist;
      wheel_base_fl.y = wheel_point_fl.y * wheel_to_base_dist;
      wheel_base_fr.x = wheel_point_fr.x * wheel_to_base_dist;
      wheel_base_fr.y = wheel_point_fr.y * wheel_to_base_dist;
      wheel_base_bl.x = wheel_point_bl.x * wheel_to_base_dist;
      wheel_base_bl.y = wheel_point_bl.y * wheel_to_base_dist;
      wheel_base_br.x = wheel_point_br.x * wheel_to_base_dist;
      wheel_base_br.y = wheel_point_br.y * wheel_to_base_dist;

      // TODO: improve code readability
      double steer_fl_rad, steer_fr_rad, steer_bl_rad, steer_br_rad;
      steer_fl_rad = acosf(((-1)*(wheel_point_fl.x) * (base_center.x-wheel_point_fl.x) + (-1)*(wheel_point_fl.y) * (base_center.y-wheel_point_fl.y))/((TRACK / sqrtf(2.)) * r_wheel_fl)) * (((-1)*(wheel_point_fl.x)*(base_center.y-wheel_point_fl.y) - (-1)*(wheel_point_fl.y)*(base_center.x-wheel_point_fl.x))/fabsf((-1)*(wheel_point_fl.x)*(base_center.y-wheel_point_fl.y) - (-1)*(wheel_point_fl.y)*(base_center.x-wheel_point_fl.x)));
      steer_fr_rad = acosf(((-1)*(wheel_point_fr.x) * (base_center.x-wheel_point_fr.x) + (-1)*(wheel_point_fr.y) * (base_center.y-wheel_point_fr.y))/((TRACK / sqrtf(2.)) * r_wheel_fr)) * (((-1)*(wheel_point_fr.x)*(base_center.y-wheel_point_fr.y) - (-1)*(wheel_point_fr.y)*(base_center.x-wheel_point_fr.x))/fabsf((-1)*(wheel_point_fr.x)*(base_center.y-wheel_point_fr.y) - (-1)*(wheel_point_fr.y)*(base_center.x-wheel_point_fr.x)));
      steer_bl_rad = acosf(((-1)*(wheel_point_bl.x) * (base_center.x-wheel_point_bl.x) + (-1)*(wheel_point_bl.y) * (base_center.y-wheel_point_bl.y))/((TRACK / sqrtf(2.)) * r_wheel_bl)) * (((-1)*(wheel_point_bl.x)*(base_center.y-wheel_point_bl.y) - (-1)*(wheel_point_bl.y)*(base_center.x-wheel_point_bl.x))/fabsf((-1)*(wheel_point_bl.x)*(base_center.y-wheel_point_bl.y) - (-1)*(wheel_point_bl.y)*(base_center.x-wheel_point_bl.x)));
      steer_br_rad = acosf(((-1)*(wheel_point_br.x) * (base_center.x-wheel_point_br.x) + (-1)*(wheel_point_br.y) * (base_center.y-wheel_point_br.y))/((TRACK / sqrtf(2.)) * r_wheel_br)) * (((-1)*(wheel_point_br.x)*(base_center.y-wheel_point_br.y) - (-1)*(wheel_point_br.y)*(base_center.x-wheel_point_br.x))/fabsf((-1)*(wheel_point_br.x)*(base_center.y-wheel_point_br.y) - (-1)*(wheel_point_br.y)*(base_center.x-wheel_point_br.x)));

      if (std::isnan(steer_fl_rad)) steer_fl_rad = 0.;
      if (std::isnan(steer_fr_rad)) steer_fr_rad = 0.;
      if (std::isnan(steer_bl_rad)) steer_bl_rad = 0.;
      if (std::isnan(steer_br_rad)) steer_br_rad = 0.;

      while ((steer_fl_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_fl_rad)) {
        if      (steer_fl_rad > (M_PI / 2.))        steer_fl_rad -= M_PI;
        else if (steer_fl_rad < ((-1) * M_PI / 2.)) steer_fl_rad += M_PI;
      }
      while ((steer_fr_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_fr_rad)) {
        if      (steer_fr_rad > (M_PI / 2.))        steer_fr_rad -= M_PI;
        else if (steer_fr_rad < ((-1) * M_PI / 2.)) steer_fr_rad += M_PI;
      }
      while ((steer_bl_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_bl_rad)) {
        if      (steer_bl_rad > (M_PI / 2.))        steer_bl_rad -= M_PI;
        else if (steer_bl_rad < ((-1) * M_PI / 2.)) steer_bl_rad += M_PI;
      }
      while ((steer_br_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_br_rad)) {
        if      (steer_br_rad > (M_PI / 2.))        steer_br_rad -= M_PI;
        else if (steer_br_rad < ((-1) * M_PI / 2.)) steer_br_rad += M_PI;
      }
      
      double temp_x, temp_y;
      temp_x = wheel_base_fl.x;
      temp_y = wheel_base_fl.y;
      wheel_base_fl.x =
          (temp_x - wheel_point_fl.x) * cosf(steer_fl_rad)
          - (temp_y - wheel_point_fl.y) * sinf(steer_fl_rad)
          + wheel_point_fl.x;
      wheel_base_fl.y =
          (temp_x - wheel_point_fl.x) * sinf(steer_fl_rad)
          + (temp_y - wheel_point_fl.y) * cosf(steer_fl_rad)
          + wheel_point_fl.y;

      temp_x = wheel_base_fr.x;
      temp_y = wheel_base_fr.y;
      wheel_base_fr.x =
          (temp_x - wheel_point_fr.x) * cosf(steer_fr_rad)
          - (temp_y - wheel_point_fr.y) * sinf(steer_fr_rad)
          + wheel_point_fr.x;
      wheel_base_fr.y =
          (temp_x - wheel_point_fr.x) * sinf(steer_fr_rad)
          + (temp_y - wheel_point_fr.y) * cosf(steer_fr_rad)
          + wheel_point_fr.y;

      temp_x = wheel_base_bl.x;
      temp_y = wheel_base_bl.y;
      wheel_base_bl.x =
          (temp_x - wheel_point_bl.x) * cosf(steer_bl_rad)
          - (temp_y - wheel_point_bl.y) * sinf(steer_bl_rad)
          + wheel_point_bl.x;
      wheel_base_bl.y =
          (temp_x - wheel_point_bl.x) * sinf(steer_bl_rad)
          + (temp_y - wheel_point_bl.y) * cosf(steer_bl_rad)
          + wheel_point_bl.y;

      temp_x = wheel_base_br.x;
      temp_y = wheel_base_br.y;
      wheel_base_br.x =
          (temp_x - wheel_point_br.x) * cosf(steer_br_rad)
          - (temp_y - wheel_point_br.y) * sinf(steer_br_rad)
          + wheel_point_br.x;
      wheel_base_br.y =
          (temp_x - wheel_point_br.x) * sinf(steer_br_rad)
          + (temp_y - wheel_point_br.y) * cosf(steer_br_rad)
          + wheel_point_br.y;

      double vel_rads = base_vel / (WHEEL_DIAMETER/2.); 
      if (vel_rads > LIMIT_VEL_RADS) {
        wheel_fl_goal_vel = LIMIT_VEL_RADS * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_fr_goal_vel = LIMIT_VEL_RADS * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_bl_goal_vel = LIMIT_VEL_RADS * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_br_goal_vel = LIMIT_VEL_RADS * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
      }
      else {
        wheel_fl_goal_vel = vel_rads * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_fr_goal_vel = vel_rads * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_bl_goal_vel = vel_rads * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
        wheel_br_goal_vel = vel_rads * (vel_twist.angular.z / fabsf(vel_twist.angular.z));
      }

      // TODO: improve code readability
      if ((acosf(((wheel_base_fl.x - wheel_point_fl.x) * (base_center.x - wheel_point_fl.x)) + ((wheel_base_fl.y - wheel_point_fl.y) * (base_center.y - wheel_point_fl.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_fl.x - wheel_point_fl.x) * (base_center.x - wheel_point_fl.x)) + ((wheel_base_fl.y - wheel_point_fl.y) * (base_center.y - wheel_point_fl.y)))))) {
        wheel_fl_goal_vel *= -1;
      }
      if ((acosf(((wheel_base_fr.x - wheel_point_fr.x) * (base_center.x - wheel_point_fr.x)) + ((wheel_base_fr.y - wheel_point_fr.y) * (base_center.y - wheel_point_fr.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_fr.x - wheel_point_fr.x) * (base_center.x - wheel_point_fr.x)) + ((wheel_base_fr.y - wheel_point_fr.y) * (base_center.y - wheel_point_fr.y)))))) {
        wheel_fr_goal_vel *= -1;
      }
      if ((acosf(((wheel_base_bl.x - wheel_point_bl.x) * (base_center.x - wheel_point_bl.x)) + ((wheel_base_bl.y - wheel_point_bl.y) * (base_center.y - wheel_point_bl.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_bl.x - wheel_point_bl.x) * (base_center.x - wheel_point_bl.x)) + ((wheel_base_bl.y - wheel_point_bl.y) * (base_center.y - wheel_point_bl.y))))) ) {
        wheel_bl_goal_vel *= -1;
      }
      if ((acosf(((wheel_base_br.x - wheel_point_br.x) * (base_center.x - wheel_point_br.x)) + ((wheel_base_br.y - wheel_point_br.y) * (base_center.y - wheel_point_br.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_br.x - wheel_point_br.x) * (base_center.x - wheel_point_br.x)) + ((wheel_base_br.y - wheel_point_br.y) * (base_center.y - wheel_point_br.y))))) ) {
        wheel_br_goal_vel *= -1;
      }

      if (0.01 < fabsf(vel_twist.linear.y)) {
        wheel_fl_goal_vel *= (r_wheel_fl / r);
        wheel_fr_goal_vel *= (r_wheel_fr / r);
        wheel_bl_goal_vel *= (r_wheel_bl / r);
        wheel_br_goal_vel *= (r_wheel_br / r);
      } else {
        if (0. < steer_fl_rad) wheel_fl_goal_vel = -1 * fabsf(wheel_fl_goal_vel);
        else                   wheel_fl_goal_vel = fabsf(wheel_fl_goal_vel);
        if (0. < steer_fr_rad) wheel_fr_goal_vel = -1 * fabsf(wheel_fr_goal_vel);
        else                   wheel_fr_goal_vel = fabsf(wheel_fr_goal_vel);
        if (0. < steer_bl_rad) wheel_bl_goal_vel = fabsf(wheel_bl_goal_vel);
        else                   wheel_bl_goal_vel = -1 * fabsf(wheel_bl_goal_vel);
        if (0. < steer_br_rad) wheel_br_goal_vel = fabsf(wheel_br_goal_vel);
        else                   wheel_br_goal_vel = -1 * fabsf(wheel_br_goal_vel);
      }

      steer_fl_goal_pos = steer_fl_rad;
      steer_fr_goal_pos = steer_fr_rad;
      steer_bl_goal_pos = steer_bl_rad;
      steer_br_goal_pos = steer_br_rad;

      break;
    }

    // Other motion
    default:{
      wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = 0.;
      break;
    }
  }
}

// Return wheel joint goal velocities as a fixed-size array.
// std::array used to avoid raw pointer.
std::array<double, 4> SobitProControl::setSteerPos() {
    std::array<double, 4> out = {
        steer_fl_goal_pos,
        steer_fr_goal_pos,
        steer_bl_goal_pos,
        steer_br_goal_pos
    };
    return out;
}

std::array<double, 4> SobitProControl::setWheelVel() {
    std::array<double, 4> out = {
        wheel_fl_goal_vel,
        wheel_fr_goal_vel,
        wheel_bl_goal_vel,
        wheel_br_goal_vel
    };
    return out;
}

