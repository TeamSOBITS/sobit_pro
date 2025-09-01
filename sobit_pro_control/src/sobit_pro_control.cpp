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

    // Swivel motion
    // TODO: Many bugs still remain...
    case SWIVEL_MOTION_MODE:{
      double base_vel = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.));
      double r = base_vel / fabsf(vel_twist.angular.z);
      double base_angle = atan2(vel_twist.linear.y , vel_twist.linear.x);

      geometry_msgs::msg::Point base_center;
      base_center.x = r * cosf(base_angle + (M_PI/2.) * (vel_twist.angular.z/fabsf(vel_twist.angular.z)));
      base_center.y = r * sinf(base_angle + (M_PI/2.) * (vel_twist.angular.z/fabsf(vel_twist.angular.z)));

      /*
      wheel_fl_goal_vel
      wheel_fr_goal_vel
      wheel_bl_goal_vel
      wheel_br_goal_vel

      steer_fl_goal_pos
      steer_fr_goal_pos
      steer_bl_goal_pos
      steer_br_goal_pos
      */

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

