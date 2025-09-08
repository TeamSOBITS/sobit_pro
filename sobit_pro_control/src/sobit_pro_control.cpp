#include "sobit_pro_control/sobit_pro_control.hpp"

void SobitProControl::setParams(const geometry_msgs::msg::Twist vel_twist)
{
  switch (motion_mode) {
    // Stop motion
    case STOP_MOTION_MODE: {
      wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = 0.;
      break;
    }

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE: {
      // Goal velocity calculation
      double vel_ms    = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // vel_twist [m/s] to vel_ms [m/s]
      double vel_rads  = vel_ms / (WHEEL_DIAMETER/2.); // vel_ms   [m/s]   to vel_rads  [rad/s]

      // Goal position calculation
      double goal_rad = atan2(vel_twist.linear.y, vel_twist.linear.x);

      steer_fl_goal_pos = goal_rad - ( 3./4.)*M_PI;
      steer_fr_goal_pos = goal_rad - ( 1./4.)*M_PI;
      steer_bl_goal_pos = goal_rad - (-3./4.)*M_PI;
      steer_br_goal_pos = goal_rad - (-1./4.)*M_PI;

      // Direction of wheel rotation
      if (vel_rads > LIMIT_VEL_RADS) wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = LIMIT_VEL_RADS;
      else                           wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = vel_rads;
      break;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE: {
      // Goal velocity calculation
      double vel_ms    = vel_twist.angular.z * (BODY_DIAMETER/2.); // vel_deg  [deg/s] to vel_ms    [m/s]
      double vel_rads  = vel_ms / (WHEEL_DIAMETER/2.);             // vel_ms   [m/s]   to vel_rads  [rad/s]

      // Goal angle calculation
      steer_fl_goal_pos = steer_fr_goal_pos = steer_bl_goal_pos = steer_br_goal_pos = 0.;

      // Velocity of wheel
      if (fabsf(vel_rads) < LIMIT_VEL_RADS)
            wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = vel_rads;
      else  wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = LIMIT_VEL_RADS * (vel_rads/fabsf(vel_rads));
      break;
    }

    // Swivel motion
    case SWIVEL_MOTION_MODE: {
      double base_vel = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.));
      double r = base_vel / fabsf(vel_twist.angular.z);
      double base_angle = atan2(vel_twist.linear.y , vel_twist.linear.x);
      int angle_pn = (0. < vel_twist.angular.z) ? 1 : -1;

      geometry_msgs::msg::Point base_center;
      base_center.x = r * cosf(base_angle + (M_PI/2.) * angle_pn);
      base_center.y = r * sinf(base_angle + (M_PI/2.) * angle_pn);

      // each wheel point from robot base
      geometry_msgs::msg::Point wheel_point_fl, wheel_point_fr, wheel_point_bl, wheel_point_br;
      wheel_point_fl.x = wheel_point_fr.x = TRACK / 2.;        // X position of front wheel is (+)
      wheel_point_bl.x = wheel_point_br.x = TRACK / 2. * (-1); // X position of  back wheel is (-)
      wheel_point_fl.y = wheel_point_bl.y = TRACK / 2.;        // Y position of  left wheel is (+)
      wheel_point_fr.y = wheel_point_br.y = TRACK / 2. * (-1); // Y position of right wheel is (i)

      // calculate the direction of each wheel (|direction| > 2PI is okay. )
      steer_fl_goal_pos = atan2(wheel_point_fl.y - base_center.y, wheel_point_fl.x - base_center.x) + M_PI/2.*angle_pn;
      steer_fr_goal_pos = atan2(wheel_point_fr.y - base_center.y, wheel_point_fr.x - base_center.x) + M_PI/2.*angle_pn;
      steer_bl_goal_pos = atan2(wheel_point_bl.y - base_center.y, wheel_point_bl.x - base_center.x) + M_PI/2.*angle_pn;
      steer_br_goal_pos = atan2(wheel_point_br.y - base_center.y, wheel_point_br.x - base_center.x) + M_PI/2.*angle_pn;

      // Align the reference angle in the direction of the base
      steer_fl_goal_pos -= ( 3./ 4.) * M_PI;
      steer_fr_goal_pos -= ( 1./ 4.) * M_PI;
      steer_bl_goal_pos -= (-3./ 4.) * M_PI;
      steer_br_goal_pos -= (-1./ 4.) * M_PI;

      // calculate the velocity of each wheel ([rad/s])
      double base_vel_rads = base_vel / (WHEEL_DIAMETER/2.);
      wheel_fl_goal_vel = base_vel_rads * sqrtf(powf(wheel_point_fl.x - base_center.x, 2.) + powf(wheel_point_fl.y - base_center.y, 2.)) / r;
      wheel_fr_goal_vel = base_vel_rads * sqrtf(powf(wheel_point_fr.x - base_center.x, 2.) + powf(wheel_point_fr.y - base_center.y, 2.)) / r;
      wheel_bl_goal_vel = base_vel_rads * sqrtf(powf(wheel_point_bl.x - base_center.x, 2.) + powf(wheel_point_bl.y - base_center.y, 2.)) / r;
      wheel_br_goal_vel = base_vel_rads * sqrtf(powf(wheel_point_br.x - base_center.x, 2.) + powf(wheel_point_br.y - base_center.y, 2.)) / r;
      break;
    }

    // Other motion
    default: {
      wheel_fl_goal_vel = wheel_fr_goal_vel = wheel_bl_goal_vel = wheel_br_goal_vel = 0.;
      break;
    }
  }
  
  // Normalize steering angles into [0, 2π)
  steer_fl_goal_pos = steer_fl_goal_pos - (2*M_PI) * ((int)(steer_fl_goal_pos / (2*M_PI)));
  steer_fr_goal_pos = steer_fr_goal_pos - (2*M_PI) * ((int)(steer_fr_goal_pos / (2*M_PI)));
  steer_bl_goal_pos = steer_bl_goal_pos - (2*M_PI) * ((int)(steer_bl_goal_pos / (2*M_PI)));
  steer_br_goal_pos = steer_br_goal_pos - (2*M_PI) * ((int)(steer_br_goal_pos / (2*M_PI)));

  // Wrap angles to [-π, π]
  if (M_PI < fabsf(steer_fl_goal_pos)) steer_fl_goal_pos -= 2*M_PI*steer_fl_goal_pos/fabsf(steer_fl_goal_pos);
  if (M_PI < fabsf(steer_fr_goal_pos)) steer_fr_goal_pos -= 2*M_PI*steer_fr_goal_pos/fabsf(steer_fr_goal_pos);
  if (M_PI < fabsf(steer_bl_goal_pos)) steer_bl_goal_pos -= 2*M_PI*steer_bl_goal_pos/fabsf(steer_bl_goal_pos);
  if (M_PI < fabsf(steer_br_goal_pos)) steer_br_goal_pos -= 2*M_PI*steer_br_goal_pos/fabsf(steer_br_goal_pos);

  // Flip steering by 180° if angle > 90°, invert wheel velocity
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
}

// Return wheel joint goal velocities. 
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

