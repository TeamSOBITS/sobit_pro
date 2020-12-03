#include "sobit_pro_control.hpp"

void SobitProControl::setParams(geometry_msgs::Twist vel_twist){

  switch (motion_mode){

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE:{

      // Goal velocity calculation
      float vel_ms = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // Euclidean distance
      float vel_rpm = vel_ms / WHEEL_LENGTH * 60.; // 60:minute
      float vel_value = vel_rpm / 0.229;

      // Goal angle calculation
      float goal_deg = atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.);
      float steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg;

      if(-45 <= goal_deg && goal_deg <= 45){
        steer_fr_deg = 45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_fl_deg = -45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_br_deg = -45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_bl_deg = 45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)

        // Direction of wheel rotation
        if(LIMIT_VEL_VALUE < vel_value){
          wheel_fr_goal_vel = LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = LIMIT_VEL_VALUE;
          wheel_br_goal_vel = -LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = vel_value;
          wheel_fl_goal_vel = vel_value;
          wheel_br_goal_vel = -vel_value;
          wheel_bl_goal_vel = -vel_value;
        }
      }

      else if(45 < goal_deg && goal_deg < 135){
        steer_fr_deg = 45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_fl_deg = 135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_br_deg = 135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_bl_deg = 45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)

        // Direction of wheel rotation
        if(LIMIT_VEL_VALUE < vel_value){
          wheel_fr_goal_vel = LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
          wheel_br_goal_vel = LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = vel_value;
          wheel_fl_goal_vel = -vel_value;
          wheel_br_goal_vel = vel_value;
          wheel_bl_goal_vel = -vel_value;
        }
      }

      else if((135 <= goal_deg && goal_deg <= 180) || (-180 <= goal_deg && goal_deg <= -135)){
        if(135 <= goal_deg && goal_deg <= 180){
          steer_fr_deg = 225. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_fl_deg = 135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_br_deg = 135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_bl_deg = 225. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        }
        if(-180 <= goal_deg && goal_deg <= -135){
          steer_fr_deg = -135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_fl_deg = -225. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_br_deg = -225. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
          steer_bl_deg = -135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        }

        // Direction of wheel rotation
        if(LIMIT_VEL_VALUE < vel_value){
          wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
          wheel_br_goal_vel = LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = -vel_value;
          wheel_fl_goal_vel = -vel_value;
          wheel_br_goal_vel = vel_value;
          wheel_bl_goal_vel = vel_value;
        }
      }

      else if(-135 < goal_deg && goal_deg < -45){
        steer_fr_deg = -135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_fl_deg = -45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_br_deg = -45. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)
        steer_bl_deg = -135. - (atan2(vel_twist.linear.x, vel_twist.linear.y) / (PAI / 180.)); // rad = deg * (PAI / 180.)

        // Direction of wheel rotation
        if(LIMIT_VEL_VALUE < vel_value){
          wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = LIMIT_VEL_VALUE;
          wheel_br_goal_vel = -LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = -vel_value;
          wheel_fl_goal_vel = vel_value;
          wheel_br_goal_vel = -vel_value;
          wheel_bl_goal_vel = vel_value;
        }
      }

      steer_fr_goal_angle = steer_fr_deg * 4096. / 360.;
      steer_fl_goal_angle = steer_fl_deg * 4096. / 360.;
      steer_br_goal_angle = steer_br_deg * 4096. / 360.;
      steer_bl_goal_angle = steer_bl_deg * 4096. / 360.;

      break;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE:{
      // Goal velocity calculation
      float vel_deg = vel_twist.angular.z * 180.0 / PAI; // rad to deg
      float vel_ms = vel_deg / 360 * SOBIT_CAR_DIAMETER * PAI;
      float vel_rpm = vel_ms / WHEEL_LENGTH * 60.; // 60:minute
      float vel_value = vel_rpm / 0.229;

      // Goal angle calculation
      steer_fr_goal_angle = 0; steer_fl_goal_angle = 0; steer_br_goal_angle = 0; steer_bl_goal_angle = 0;

      // Direction of wheel rotation
      if(vel_twist.angular.z < 0){
        if(vel_value < -LIMIT_VEL_VALUE){
          wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
          wheel_br_goal_vel = -LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = vel_value;
          wheel_fl_goal_vel = vel_value;
          wheel_br_goal_vel = vel_value;
          wheel_bl_goal_vel = vel_value;
        }
      }
      else{
        if(LIMIT_VEL_VALUE < vel_value){
          wheel_fr_goal_vel = LIMIT_VEL_VALUE;
          wheel_fl_goal_vel = LIMIT_VEL_VALUE;
          wheel_br_goal_vel = LIMIT_VEL_VALUE;
          wheel_bl_goal_vel = LIMIT_VEL_VALUE;
        }
        else{
          wheel_fr_goal_vel = vel_value;
          wheel_fl_goal_vel = vel_value;
          wheel_br_goal_vel = vel_value;
          wheel_bl_goal_vel = vel_value;
        }
      }
      break;
    }

    // Other motion
    default:
      wheel_fr_goal_vel = 0; wheel_fl_goal_vel = 0; wheel_br_goal_vel = 0; wheel_bl_goal_vel = 0;
      break;
  }
}

int SobitProControl::showMode(){
  return (int)motion_mode;
}

int32_t *SobitProControl::setSteerAngle(){
  steer_angle[0] = steer_fr_goal_angle + 2048;
  steer_angle[1] = steer_fl_goal_angle + 2048;
  steer_angle[2] = steer_br_goal_angle + 2048;
  steer_angle[3] = steer_bl_goal_angle + 2048;

  return steer_angle;
}

int32_t *SobitProControl::setWheelVel(){
  wheel_vel[0] = wheel_fr_goal_vel;
  wheel_vel[1] = wheel_fl_goal_vel;
  wheel_vel[2] = wheel_br_goal_vel;
  wheel_vel[3] = wheel_bl_goal_vel;

  return wheel_vel;
}
