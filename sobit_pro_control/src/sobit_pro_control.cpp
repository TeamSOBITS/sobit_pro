#include "sobit_pro_control/sobit_pro_control.hpp"

void SobitProControl::setParams(geometry_msgs::Twist vel_twist){

    switch (motion_mode){
        // Translational motion
        case TRANSLATIONAL_MOTION_MODE:{
            // Goal velocity calculation
            float vel_ms    = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // Euclidean distance
            // float vel_rpm   = vel_ms / WHEEL_LENGTH * 60.; // 60:minute
            // float vel_value = vel_rpm / 0.229;
            float vel_rpm   = vel_ms / (M_PI*WHEEL_DIAMETER) * 60.; // 60:minute
            float vel_value = vel_rpm / VEL_UNIT;

            // Goal angle calculation
            float goal_deg = atan2(vel_twist.linear.x, vel_twist.linear.y) / (M_PI / 180.);
            float steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg;

            if(-45 <= goal_deg && goal_deg <= 45){
                steer_fr_deg =  45. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_fl_deg = -45. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_br_deg = -45. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_bl_deg =  45. - goal_deg; // rad = deg * (M_PI / 180.)

                // Direction of wheel rotation
                if(LIMIT_VEL_VALUE < vel_value){
                    wheel_fr_goal_vel =  LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel =  LIMIT_VEL_VALUE;
                    wheel_br_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel =  vel_value;
                    wheel_fl_goal_vel =  vel_value;
                    wheel_br_goal_vel = -vel_value;
                    wheel_bl_goal_vel = -vel_value;
                }
            }

            else if(45 < goal_deg && goal_deg <= 135){
                steer_fr_deg = 45.  - goal_deg; // rad = deg * (M_PI / 180.)
                steer_fl_deg = 135. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_br_deg = 135. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_bl_deg = 45.  - goal_deg; // rad = deg * (M_PI / 180.)

                // Direction of wheel rotation
                if(LIMIT_VEL_VALUE < vel_value){
                    wheel_fr_goal_vel =  LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_br_goal_vel =  LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel =  vel_value;
                    wheel_fl_goal_vel = -vel_value;
                    wheel_br_goal_vel =  vel_value;
                    wheel_bl_goal_vel = -vel_value;
                }
            }

            else if(-45 > goal_deg  && goal_deg >= -135){
                steer_fr_deg = -135. - goal_deg; // rad = deg * (M_PI / 180.)
                steer_fl_deg = -45.  - goal_deg; // rad = deg * (M_PI / 180.)
                steer_br_deg = -45.  - goal_deg; // rad = deg * (M_PI / 180.)
                steer_bl_deg = -135. - goal_deg; // rad = deg * (M_PI / 180.)

                // Direction of wheel rotation
                if(LIMIT_VEL_VALUE < vel_value){
                    wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel = LIMIT_VEL_VALUE;
                    wheel_br_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel = LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel = -vel_value;
                    wheel_fl_goal_vel = vel_value;
                    wheel_br_goal_vel = -vel_value;
                    wheel_bl_goal_vel = vel_value;
                }
            }

            else if((135 < goal_deg && goal_deg <= 180) || (-135 > goal_deg && goal_deg >= -180)){
                if(135 < goal_deg && goal_deg <= 180){
                    steer_fr_deg = 225. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_fl_deg = 135. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_br_deg = 135. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_bl_deg = 225. - goal_deg; // rad = deg * (M_PI / 180.)
                }
                if(-135 > goal_deg && goal_deg >= -180){
                    steer_fr_deg = -135. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_fl_deg = -225. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_br_deg = -225. - goal_deg; // rad = deg * (M_PI / 180.)
                    steer_bl_deg = -135. - goal_deg; // rad = deg * (M_PI / 180.)
                }

                // Direction of wheel rotation
                if(LIMIT_VEL_VALUE < vel_value){
                    wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_br_goal_vel =  LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel =  LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel = -vel_value;
                    wheel_fl_goal_vel = -vel_value;
                    wheel_br_goal_vel =  vel_value;
                    wheel_bl_goal_vel =  vel_value;
                }
            }

            steer_fr_goal_angle = steer_fr_deg * 4096. / 360.;
            steer_fl_goal_angle = steer_fl_deg * 4096. / 360.;
            steer_br_goal_angle = steer_br_deg * 4096. / 360.;
            steer_bl_goal_angle = steer_bl_deg * 4096. / 360.;
            // ROS_INFO("Wheel INFO(Translational motion)\n\t steer_fr_deg = %.3f\n\t steer_fl_deg = %.3f\n\t steer_br_deg = %.3f\n\t steer_bl_deg = %.3f\n\t wheel_fr_goal_vel = %.3f\n\t wheel_fl_goal_vel = %.3f\n\t wheel_br_goal_vel = %.3f\n\t wheel_bl_goal_vel = %.3f", steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg, wheel_fr_goal_vel, wheel_fl_goal_vel, wheel_br_goal_vel, wheel_bl_goal_vel );

            break;
        }

        // Rotational motion
        case ROTATIONAL_MOTION_MODE:{
            // Goal velocity calculation
            float vel_deg   = vel_twist.angular.z * 180. / M_PI; // rad to deg
            float vel_ms    = vel_deg / 360. * BODY_DIAMETER * M_PI;
            float vel_rpm   = vel_ms / WHEEL_LENGTH * 60.; // 60:minute
            float vel_value = vel_rpm / VEL_UNIT;

            // Goal angle calculation
            steer_fr_goal_angle = 0.; steer_fl_goal_angle = 0.;
            steer_br_goal_angle = 0.; steer_bl_goal_angle = 0.;

            // Velocity of wheel
            if(vel_twist.angular.z < 0.){
                if(vel_value < -LIMIT_VEL_VALUE){
                    wheel_fr_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_br_goal_vel = -LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel = -LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel = vel_value;
                    wheel_fl_goal_vel = vel_value;
                    wheel_br_goal_vel = vel_value;
                    wheel_bl_goal_vel = vel_value;
                }
            } else{
                if(vel_value > LIMIT_VEL_VALUE ){
                    wheel_fr_goal_vel = LIMIT_VEL_VALUE;
                    wheel_fl_goal_vel = LIMIT_VEL_VALUE;
                    wheel_br_goal_vel = LIMIT_VEL_VALUE;
                    wheel_bl_goal_vel = LIMIT_VEL_VALUE;
                } else{
                    wheel_fr_goal_vel = vel_value;
                    wheel_fl_goal_vel = vel_value;
                    wheel_br_goal_vel = vel_value;
                    wheel_bl_goal_vel = vel_value;
                }
            }

            // ROS_INFO("Wheel INFO(Rotational motion)\n\t wheel_fr_goal_vel = %.3f\n\t wheel_fl_goal_vel = %.3f\n\t wheel_br_goal_vel = %.3f\n\t wheel_bl_goal_vel = %.3f", wheel_fr_goal_vel, wheel_fl_goal_vel, wheel_br_goal_vel, wheel_bl_goal_vel );
            break;
        }

        //Swivel motion
        case SWIVEL_MOTION_MODE:{
            // Radius calculation
            float vel_ms = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.)); // Euclidean distance
            float vel_deg = abs(vel_twist.angular.z) * 180. / M_PI; // rad to deg
            float cycle_sec = 360. / vel_deg;
            float svl_r = vel_ms * cycle_sec / 2. / M_PI;    // meter

            float d = BODY_DIAMETER / 2 * cos(M_PI / 4);  // Manhattan distance (cos value shud be change)
            float svl_r_in, svl_r_out;
            float goal_deg_in, goal_deg_out;

            if(svl_r < d){
                svl_r_in  = sqrtf(powf((d - svl_r), 2.) + powf(d, 2.));
                svl_r_out = sqrtf(powf((svl_r + d), 2.) + powf(d, 2.));

                // Goal angle calculation
                goal_deg_in  = 45. - atan2(d, (d - svl_r)) / (M_PI / 180.);
                goal_deg_out = atan2(d, (svl_r + d)) / (M_PI / 180.) - 45.;
            } else{
                svl_r_in  = sqrtf(powf((svl_r - d), 2.) + powf(d, 2.));
                svl_r_out = sqrtf(powf((svl_r + d), 2.) + powf(d, 2.));

                // Goal angle calculation
                goal_deg_in  = 45. + atan2(d, (svl_r - d)) / (M_PI / 180.);
                goal_deg_out = atan2(d, (svl_r + d)) / (M_PI / 180.) - 45.;
            }


            // Goal velocity calculation
            float vel_ms_in     = vel_deg / 360. * 2. * svl_r_in * M_PI;
            float vel_ms_out    = vel_deg / 360. * 2. * svl_r_out * M_PI;
            float vel_rpm_in    = vel_ms_in / WHEEL_LENGTH * 60.; // 60:minute
            float vel_rpm_out   = vel_ms_out / WHEEL_LENGTH * 60.; // 60:minute
            float vel_value_in  = vel_rpm_in / VEL_UNIT;
            float vel_value_out = vel_rpm_out / VEL_UNIT;
            
            // Direction of wheel rotation
            float steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg;
            float rotate_dir_in = 1., rotate_dir_out = 1.;

            if(vel_twist.angular.z > 0){
                if(svl_r < d){
                    steer_fl_deg = goal_deg_in;
                    steer_bl_deg = -goal_deg_in;
                    steer_fr_deg = goal_deg_out; // rad = deg * (M_PI / 180.)
                    steer_br_deg = -goal_deg_out; // rad = deg * (M_PI / 180.)

                    if(vel_twist.linear.x > 0) { rotate_dir_out = 1.; rotate_dir_in = 1.; }
                    else { rotate_dir_out = -1.; rotate_dir_in = -1.; }

                    if(LIMIT_VEL_VALUE < vel_value_out){
                        ROS_WARN_STREAM("Too Fast !! : vel_value_out = " << vel_value_out );
                        // out
                        wheel_fr_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        wheel_br_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;

                        // in
                        wheel_fl_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                        wheel_bl_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                    } else{
                        // out
                        wheel_fr_goal_vel = rotate_dir_out * vel_value_out;
                        wheel_br_goal_vel = rotate_dir_out * vel_value_out;
                        
                        // in
                        wheel_fl_goal_vel = rotate_dir_in * vel_value_in;
                        wheel_bl_goal_vel = rotate_dir_in * vel_value_in;
                    }
                }
            
                else{
                    if(45 <= goal_deg_in && goal_deg_in < 90.){
                        steer_fl_deg = goal_deg_in; // rad = deg * (M_PI / 180.)
                        steer_bl_deg = -goal_deg_in; // rad = deg * (M_PI / 180.)

                        if(vel_twist.linear.x > 0) rotate_dir_in = -1.;
                        else rotate_dir_in = 1.;
                    }
                    else if(90. <= goal_deg_in && goal_deg_in <= 135){
                        steer_fl_deg = goal_deg_in - 180.; // rad = deg * (M_PI / 180.)
                        steer_bl_deg = -(goal_deg_in - 180.); // rad = deg * (M_PI / 180.)

                        if(vel_twist.linear.x > 0) rotate_dir_in = 1.;
                        else rotate_dir_in = -1.;
                    }

                    steer_fr_deg = goal_deg_out; // rad = deg * (M_PI / 180.)
                    steer_br_deg = -goal_deg_out; // rad = deg * (M_PI / 180.)

                    if(vel_twist.linear.x > 0) rotate_dir_out = 1.;
                    else rotate_dir_out = -1.;

                    if(LIMIT_VEL_VALUE < vel_value_out){
                        ROS_WARN_STREAM("Too Fast !! : vel_value_out = " << vel_value_out );
                        // out
                        wheel_fr_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        wheel_br_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        // in
                        wheel_fl_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                        wheel_bl_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                    }
                    else{
                        // out
                        wheel_fr_goal_vel = rotate_dir_out * vel_value_out;
                        wheel_br_goal_vel = rotate_dir_out * vel_value_out;
                        // in
                        wheel_fl_goal_vel = rotate_dir_in * vel_value_in;
                        wheel_bl_goal_vel = rotate_dir_in * vel_value_in;
                    }
                }

            }

            else{
                if(svl_r < d){
                    steer_fl_deg = -goal_deg_out; // rad = deg * (M_PI / 180.)
                    steer_bl_deg = goal_deg_out; // rad = deg * (M_PI / 180.)
                    steer_fr_deg = -goal_deg_in;
                    steer_br_deg = goal_deg_in;

                    if(vel_twist.linear.x > 0) { rotate_dir_out = -1.; rotate_dir_in = -1.; }
                    else { rotate_dir_out = 1.; rotate_dir_in = 1.; }

                    if(LIMIT_VEL_VALUE < vel_value_out){
                        ROS_WARN_STREAM("Too Fast !! : vel_value_out = " << vel_value_out );
                        // out
                        wheel_fl_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        wheel_bl_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        // in
                        wheel_fr_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                        wheel_br_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                    }
                    else{
                        // out
                        wheel_fl_goal_vel = rotate_dir_out * vel_value_out;
                        wheel_bl_goal_vel = rotate_dir_out * vel_value_out;
                        // in
                        wheel_fr_goal_vel = rotate_dir_in * vel_value_in;
                        wheel_br_goal_vel = rotate_dir_in * vel_value_in;
                    }
                }

                else {
                    if(45 <= goal_deg_in && goal_deg_in < 90){
                        steer_fr_deg = -goal_deg_in; // rad = deg * (M_PI / 180.)
                        steer_br_deg = goal_deg_in; // rad = deg * (M_PI / 180.)

                        if(vel_twist.linear.x > 0) rotate_dir_in = 1.;
                        else rotate_dir_in = -1.;
                    }
                    else if(90 <= goal_deg_in && goal_deg_in <= 135){
                        steer_fr_deg = -(goal_deg_in - 180.); // rad = deg * (M_PI / 180.)
                        steer_br_deg = goal_deg_in - 180.; // rad = deg * (M_PI / 180.)

                        if(vel_twist.linear.x > 0) rotate_dir_in = -1.;
                        else rotate_dir_in = 1.;

                    }

                    steer_fl_deg = -goal_deg_out; // rad = deg * (M_PI / 180.)
                    steer_bl_deg = goal_deg_out; // rad = deg * (M_PI / 180.)

                    if(vel_twist.linear.x > 0) rotate_dir_out = -1.;
                    else rotate_dir_out = 1.;

                    if(vel_value_out > LIMIT_VEL_VALUE){
                        ROS_WARN_STREAM("Too Fast !! : vel_value_out = " << vel_value_out );
                        // out
                        wheel_fl_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        wheel_bl_goal_vel = rotate_dir_out * LIMIT_VEL_VALUE;
                        // in
                        wheel_fr_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                        wheel_br_goal_vel = rotate_dir_in * vel_value_in / vel_value_out * LIMIT_VEL_VALUE;
                    }
                    else{
                        // out
                        wheel_fl_goal_vel = rotate_dir_out * vel_value_out;
                        wheel_bl_goal_vel = rotate_dir_out * vel_value_out;
                        // in
                        wheel_fr_goal_vel = rotate_dir_in * vel_value_in;
                        wheel_br_goal_vel = rotate_dir_in * vel_value_in;
                    }
                }
            }  
        
            // if ((abs(abs(steer_fr_deg) - 45.) <= 1.)||(abs(abs(steer_fl_deg) - 45.) <= 1.)) ROS_INFO("Wheel INFO(Swivel motion)\n\t steer_fr_deg = %.3f\n\t steer_fl_deg = %.3f\n\t steer_br_deg = %.3f\n\t steer_bl_deg = %.3f\n\t wheel_fr_goal_vel = %.3f\n\t wheel_fl_goal_vel = %.3f\n\t wheel_br_goal_vel = %.3f\n\t wheel_bl_goal_vel = %.3f", steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg, wheel_fr_goal_vel, wheel_fl_goal_vel, wheel_br_goal_vel, wheel_bl_goal_vel );
            // Hardware limit
            if (abs(steer_fr_deg) > 90 || abs(steer_fl_deg) > 90) {
                ROS_ERROR("Over Hardware limit!!\n\t steer_fr_deg = %.3f\n\t steer_fl_deg = %.3f\n\t steer_br_deg = %.3f\n\t steer_bl_deg = %.3f\n", steer_fr_deg, steer_fl_deg, steer_br_deg, steer_bl_deg);
                steer_fr_deg = 0.; steer_fl_deg = 0.; steer_br_deg = 0.; steer_bl_deg = 0.;
            }
            steer_fr_goal_angle = steer_fr_deg * 4096. / 360.;
            steer_fl_goal_angle = steer_fl_deg * 4096. / 360.;
            steer_br_goal_angle = steer_br_deg * 4096. / 360.;
            steer_bl_goal_angle = steer_bl_deg * 4096. / 360.;

            // ROS_INFO("Wheel INFO(Swivel motion)\n\t steer_fr_deg = %.3f\n\t steer_fl_deg = %.3f\n\t steer_br_deg = %.3f\n\t steer_bl_deg = %.3f\n\t wheel_fr_goal_vel = %.3f\n\t wheel_fl_goal_vel = %.3f\n\t wheel_br_goal_vel = %.3f\n\t wheel_bl_goal_vel = %.3f", steer_fr_goal_angle, steer_fl_goal_angle, steer_br_goal_angle, steer_bl_goal_angle, wheel_fr_goal_vel, wheel_fl_goal_vel, wheel_br_goal_vel, wheel_bl_goal_vel );
            
            break;
        }

        // Other motion
        default:
            wheel_fr_goal_vel = 0.; wheel_fl_goal_vel = 0.; wheel_br_goal_vel = 0.; wheel_bl_goal_vel = 0.;
            break;
    }
}

int SobitProControl::showMode(){
    return int(motion_mode);
}

int32_t *SobitProControl::setSteerAngle(){
    steer_angle[0] = int32_t(steer_fr_goal_angle + 2048);
    steer_angle[1] = int32_t(steer_fl_goal_angle + 2048);
    steer_angle[2] = int32_t(steer_br_goal_angle + 2048);
    steer_angle[3] = int32_t(steer_bl_goal_angle + 2048);

    return steer_angle;
}

int32_t *SobitProControl::setWheelVel(){
    wheel_vel[0] = int32_t(wheel_fr_goal_vel);
    wheel_vel[1] = int32_t(wheel_fl_goal_vel);
    wheel_vel[2] = int32_t(wheel_br_goal_vel);
    wheel_vel[3] = int32_t(wheel_bl_goal_vel);

    return wheel_vel;
}
