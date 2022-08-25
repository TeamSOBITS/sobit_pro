#include "sobit_pro_control/sobit_pro_control.hpp"

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
        steer_fr_deg = 45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_fl_deg = -45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_br_deg = -45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_bl_deg = 45. - goal_deg; // rad = deg * (PAI / 180.)

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
        steer_fr_deg = 45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_fl_deg = 135. - goal_deg; // rad = deg * (PAI / 180.)
        steer_br_deg = 135. - goal_deg; // rad = deg * (PAI / 180.)
        steer_bl_deg = 45. - goal_deg; // rad = deg * (PAI / 180.)

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
          steer_fr_deg = 225. - goal_deg; // rad = deg * (PAI / 180.)
          steer_fl_deg = 135. - goal_deg; // rad = deg * (PAI / 180.)
          steer_br_deg = 135. - goal_deg; // rad = deg * (PAI / 180.)
          steer_bl_deg = 225. - goal_deg; // rad = deg * (PAI / 180.)
        }
        if(-180 <= goal_deg && goal_deg <= -135){
          steer_fr_deg = -135. - goal_deg; // rad = deg * (PAI / 180.)
          steer_fl_deg = -225. - goal_deg; // rad = deg * (PAI / 180.)
          steer_br_deg = -225. - goal_deg; // rad = deg * (PAI / 180.)
          steer_bl_deg = -135. - goal_deg; // rad = deg * (PAI / 180.)
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
        steer_fr_deg = -135. - goal_deg; // rad = deg * (PAI / 180.)
        steer_fl_deg = -45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_br_deg = -45. - goal_deg; // rad = deg * (PAI / 180.)
        steer_bl_deg = -135. - goal_deg; // rad = deg * (PAI / 180.)

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

    //Swivel motion
    case SWIVEL_MOTION_MODE:{
      // Radius calculation
      float vel_ms = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // Euclidean distance
      float vel_deg = abs(vel_twist.angular.z) * 180.0 / PAI; // rad to deg
      float cycle_sec = 360.0 / vel_deg;
      float svl_r = vel_ms * cycle_sec / 2. / PAI;    // meter

      if(svl_r < SOBIT_CAR_DIAMETER / 2){
        printf("Swivel radius is too small!!\n");
        printf("Swivel radius : %f\n", svl_r);
        printf("SOBIT_CAR_DIAMETER : %f\n", SOBIT_CAR_DIAMETER);
        break;
      }
      float d = SOBIT_CAR_DIAMETER / 2 * cos(PAI / 4);  // Manhattan distance (cos value shud be change)
      float svl_r_in = sqrtf(powf((svl_r - d), 2.) + powf(d, 2.));
      float svl_r_out = sqrtf(powf((svl_r + d), 2.) + powf(d, 2.));


      // Goal velocity calculation
      float vel_ms_in = vel_deg / 360. * 2. * svl_r_in * PAI;
      float vel_ms_out = vel_deg / 360. * 2. * svl_r_out * PAI;
      float vel_rpm_in = vel_ms_in / WHEEL_LENGTH * 60.; // 60:minute
      float vel_rpm_out = vel_ms_out / WHEEL_LENGTH * 60.; // 60:minute
      float vel_value_in = vel_rpm_in / 0.229;
      float vel_value_out = vel_rpm_out / 0.229;

      // Goal angle calculation
      float goal_deg_in = (90.0 + (atan2(d, (svl_r - d)) / (PAI / 180.)));
      float goal_deg_out = (90.0 + (atan2(d, (svl_r + d)) / (PAI / 180.)));
      
      // Direction of wheel rotation
      float steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg;

      if(vel_twist.angular.z > 0){
        if(90 <= goal_deg_in && goal_deg_in < 135){
          steer_fl_deg = goal_deg_in - 45.; // rad = deg * (PAI / 180.)
          steer_bl_deg = -(goal_deg_in - 45.); // rad = deg * (PAI / 180.)

          if(LIMIT_VEL_VALUE < vel_value_in){
            if(vel_twist.linear.x > 0){
              wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
              wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
            }
            else{
              wheel_fl_goal_vel = LIMIT_VEL_VALUE;
              wheel_bl_goal_vel = LIMIT_VEL_VALUE;
            }
          }
          else{
            if(vel_twist.linear.x > 0){
              wheel_fl_goal_vel = -vel_value_in;
              wheel_bl_goal_vel = -vel_value_in;
            }
            else{
              wheel_fl_goal_vel = vel_value_in;
              wheel_bl_goal_vel = vel_value_in;
            }
          }
        }
        else if(135 <= goal_deg_in && goal_deg_in <= 180){
          steer_fl_deg = goal_deg_in - 225.; // rad = deg * (PAI / 180.)
          steer_bl_deg = -(goal_deg_in - 225.); // rad = deg * (PAI / 180.)

          if(LIMIT_VEL_VALUE < vel_value_in){
            if(vel_twist.linear.x > 0){
              wheel_fl_goal_vel = LIMIT_VEL_VALUE;
              wheel_bl_goal_vel = LIMIT_VEL_VALUE;
            }
            else{
              wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
              wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
            }
          }
          else{
            if(vel_twist.linear.x > 0){
              wheel_fl_goal_vel = vel_value_in;
              wheel_bl_goal_vel = vel_value_in;
            }
            else{
              wheel_fl_goal_vel = -vel_value_in;
              wheel_bl_goal_vel = -vel_value_in;
            }
          }

        }

        steer_fr_deg = goal_deg_out - 135.; // rad = deg * (PAI / 180.)
        steer_br_deg = -(goal_deg_out - 135.); // rad = deg * (PAI / 180.)

        if(LIMIT_VEL_VALUE < vel_value_out){
          if(vel_twist.linear.x > 0){
            wheel_fr_goal_vel = LIMIT_VEL_VALUE;
            wheel_br_goal_vel = LIMIT_VEL_VALUE;
          }
          else{
            wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
            wheel_br_goal_vel = -LIMIT_VEL_VALUE;
          }
        }
        else{
          if(vel_twist.linear.x > 0){
            wheel_fr_goal_vel = vel_value_out;
            wheel_br_goal_vel = vel_value_out;
          }
          else{
            wheel_fr_goal_vel = -vel_value_out;
            wheel_br_goal_vel = -vel_value_out;
          }
        }

      }

      else{
        if(90 <= goal_deg_in && goal_deg_in < 135){
          steer_fr_deg = -(goal_deg_in - 45.); // rad = deg * (PAI / 180.)
          steer_br_deg = goal_deg_in - 45.; // rad = deg * (PAI / 180.)

          if(LIMIT_VEL_VALUE < vel_value_in){
            if(vel_twist.linear.x > 0){
              wheel_fr_goal_vel = LIMIT_VEL_VALUE;
              wheel_br_goal_vel = LIMIT_VEL_VALUE;
            }
            else{
              wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
              wheel_br_goal_vel = -LIMIT_VEL_VALUE;
            }
          }
          else{
            if(vel_twist.linear.x > 0){
              wheel_fr_goal_vel = vel_value_in;
              wheel_br_goal_vel = vel_value_in;
            }
            else{
              wheel_fr_goal_vel = -vel_value_in;
              wheel_br_goal_vel = -vel_value_in;
            }
          }
        }
        else if(135 <= goal_deg_in && goal_deg_in <= 180){
          steer_fr_deg = -(goal_deg_in - 225.); // rad = deg * (PAI / 180.)
          steer_br_deg = goal_deg_in - 225.; // rad = deg * (PAI / 180.)

          if(LIMIT_VEL_VALUE < vel_value_in){
            if(vel_twist.linear.x > 0){
              wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
              wheel_br_goal_vel = -LIMIT_VEL_VALUE;
            }
            else{
              wheel_fr_goal_vel = LIMIT_VEL_VALUE;
              wheel_br_goal_vel = LIMIT_VEL_VALUE;
            }
          }
          else{
            if(vel_twist.linear.x > 0){
              wheel_fr_goal_vel = -vel_value_in;
              wheel_br_goal_vel = -vel_value_in;
            }
            else{
              wheel_fr_goal_vel = vel_value_in;
              wheel_br_goal_vel = vel_value_in;
            }
          }

        }

        steer_fl_deg = -(goal_deg_out - 135.); // rad = deg * (PAI / 180.)
        steer_bl_deg = goal_deg_out - 135.; // rad = deg * (PAI / 180.)

        if(LIMIT_VEL_VALUE < vel_value_out){
          if(vel_twist.linear.x > 0){
            wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
            wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
          }
          else{
            wheel_fl_goal_vel = LIMIT_VEL_VALUE;
            wheel_bl_goal_vel = LIMIT_VEL_VALUE;
          }
        }
        else{
          if(vel_twist.linear.x > 0){
            wheel_fl_goal_vel = -vel_value_out;
            wheel_bl_goal_vel = -vel_value_out;
          }
          else{
            wheel_fl_goal_vel = vel_value_out;
            wheel_bl_goal_vel = vel_value_out;
          }
        }
      }  
      
      steer_fr_goal_angle = steer_fr_deg * 4096. / 360.;
      steer_fl_goal_angle = steer_fl_deg * 4096. / 360.;
      steer_br_goal_angle = steer_br_deg * 4096. / 360.;
      steer_bl_goal_angle = steer_bl_deg * 4096. / 360.;
      
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
