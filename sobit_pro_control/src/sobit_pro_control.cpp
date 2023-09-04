#include "sobit_pro_control/sobit_pro_control.hpp"

void SobitProControl::setParams(geometry_msgs::Twist vel_twist){

    switch (motion_mode){
        // Stop motion
        case STOP_MOTION_MODE:{
            wheel_fr_goal_vel = 0.; wheel_fl_goal_vel = 0.; wheel_br_goal_vel = 0.; wheel_bl_goal_vel = 0.;
            break;
        }
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
            float base_vel = sqrtf(powf(vel_twist.linear.x, 2.) + powf(vel_twist.linear.y, 2.));
            float r = base_vel / abs(vel_twist.angular.z);
            float base_angle = atan2(vel_twist.linear.y , vel_twist.linear.x);
            geometry_msgs::Point base_center;
            base_center.x = r * cos(base_angle + (M_PI/2) * (vel_twist.angular.z/abs(vel_twist.angular.z)));
            base_center.y = r * sin(base_angle + (M_PI/2) * (vel_twist.angular.z/abs(vel_twist.angular.z)));
            geometry_msgs::Point wheel_point_fr, wheel_point_fl, wheel_point_br, wheel_point_bl;
            wheel_point_fr.x = TRACK / 2.;
            wheel_point_fr.y = TRACK / 2. * (-1);
            wheel_point_fl.x = TRACK / 2.;
            wheel_point_fl.y = TRACK / 2.;
            wheel_point_br.x = TRACK / 2. * (-1);
            wheel_point_br.y = TRACK / 2. * (-1);
            wheel_point_bl.x = TRACK / 2. * (-1);
            wheel_point_bl.y = TRACK / 2.;
            float r_wheel_fr, r_wheel_fl, r_wheel_br, r_wheel_bl;
            r_wheel_fr = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2*(TRACK / sqrtf(2.))*r*(((wheel_point_fr.x * base_center.x) + (wheel_point_fr.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
            r_wheel_fl = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2*(TRACK / sqrtf(2.))*r*(((wheel_point_fl.x * base_center.x) + (wheel_point_fl.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
            r_wheel_br = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2*(TRACK / sqrtf(2.))*r*(((wheel_point_br.x * base_center.x) + (wheel_point_br.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
            r_wheel_bl = sqrtf(powf(TRACK / sqrtf(2.), 2.) + powf(r, 2.) - 2*(TRACK / sqrtf(2.))*r*(((wheel_point_bl.x * base_center.x) + (wheel_point_bl.y * base_center.y))/((TRACK / sqrtf(2.)) * r)));
            geometry_msgs::Point wheel_base_fr, wheel_base_fl, wheel_base_br, wheel_base_bl;
            float wheel_to_base_dist = 1.3;
            wheel_base_fr.x = wheel_point_fr.x * wheel_to_base_dist;
            wheel_base_fr.y = wheel_point_fr.y * wheel_to_base_dist;
            wheel_base_fl.x = wheel_point_fl.x * wheel_to_base_dist;
            wheel_base_fl.y = wheel_point_fl.y * wheel_to_base_dist;
            wheel_base_br.x = wheel_point_br.x * wheel_to_base_dist;
            wheel_base_br.y = wheel_point_br.y * wheel_to_base_dist;
            wheel_base_bl.x = wheel_point_bl.x * wheel_to_base_dist;
            wheel_base_bl.y = wheel_point_bl.y * wheel_to_base_dist;
            float steer_fr_rad, steer_fl_rad, steer_br_rad, steer_bl_rad;
            steer_fr_rad = acos(((-1)*(wheel_point_fr.x) * (base_center.x-wheel_point_fr.x) + (-1)*(wheel_point_fr.y) * (base_center.y-wheel_point_fr.y))/((TRACK / sqrtf(2.)) * r_wheel_fr)) * (((-1)*(wheel_point_fr.x)*(base_center.y-wheel_point_fr.y) - (-1)*(wheel_point_fr.y)*(base_center.x-wheel_point_fr.x))/abs((-1)*(wheel_point_fr.x)*(base_center.y-wheel_point_fr.y) - (-1)*(wheel_point_fr.y)*(base_center.x-wheel_point_fr.x)));
            steer_fl_rad = acos(((-1)*(wheel_point_fl.x) * (base_center.x-wheel_point_fl.x) + (-1)*(wheel_point_fl.y) * (base_center.y-wheel_point_fl.y))/((TRACK / sqrtf(2.)) * r_wheel_fl)) * (((-1)*(wheel_point_fl.x)*(base_center.y-wheel_point_fl.y) - (-1)*(wheel_point_fl.y)*(base_center.x-wheel_point_fl.x))/abs((-1)*(wheel_point_fl.x)*(base_center.y-wheel_point_fl.y) - (-1)*(wheel_point_fl.y)*(base_center.x-wheel_point_fl.x)));
            steer_br_rad = acos(((-1)*(wheel_point_br.x) * (base_center.x-wheel_point_br.x) + (-1)*(wheel_point_br.y) * (base_center.y-wheel_point_br.y))/((TRACK / sqrtf(2.)) * r_wheel_br)) * (((-1)*(wheel_point_br.x)*(base_center.y-wheel_point_br.y) - (-1)*(wheel_point_br.y)*(base_center.x-wheel_point_br.x))/abs((-1)*(wheel_point_br.x)*(base_center.y-wheel_point_br.y) - (-1)*(wheel_point_br.y)*(base_center.x-wheel_point_br.x)));
            steer_bl_rad = acos(((-1)*(wheel_point_bl.x) * (base_center.x-wheel_point_bl.x) + (-1)*(wheel_point_bl.y) * (base_center.y-wheel_point_bl.y))/((TRACK / sqrtf(2.)) * r_wheel_bl)) * (((-1)*(wheel_point_bl.x)*(base_center.y-wheel_point_bl.y) - (-1)*(wheel_point_bl.y)*(base_center.x-wheel_point_bl.x))/abs((-1)*(wheel_point_bl.x)*(base_center.y-wheel_point_bl.y) - (-1)*(wheel_point_bl.y)*(base_center.x-wheel_point_bl.x)));
            if (std::isnan(steer_fr_rad))
            {
                steer_fr_rad = 0.;
            }
            if (std::isnan(steer_fl_rad))
            {
                steer_fl_rad = 0.;
            }
            if (std::isnan(steer_br_rad))
            {
                steer_br_rad = 0.;
            }
            if (std::isnan(steer_bl_rad))
            {
                steer_bl_rad = 0.;
            }
            while ((steer_fr_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_fr_rad))
            {
                if (steer_fr_rad > (M_PI / 2.))
                {
                    steer_fr_rad -= M_PI;
                }
                else if (steer_fr_rad < ((-1) * M_PI / 2.))
                {
                    steer_fr_rad += M_PI;
                }
            }
            while ((steer_fl_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_fl_rad))
            {
                if (steer_fl_rad > (M_PI / 2.))
                {
                    steer_fl_rad -= M_PI;
                }
                else if (steer_fl_rad < ((-1) * M_PI / 2.))
                {
                    steer_fl_rad += M_PI;
                }
            }
            while ((steer_br_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_br_rad))
            {
                if (steer_br_rad > (M_PI / 2.))
                {
                    steer_br_rad -= M_PI;
                }
                else if (steer_br_rad < ((-1) * M_PI / 2.))
                {
                    steer_br_rad += M_PI;
                }
            }
            while ((steer_bl_rad < ((-1) * M_PI / 2.)) || ((M_PI / 2.) < steer_bl_rad))
            {
                if (steer_bl_rad > (M_PI / 2.))
                {
                    steer_bl_rad -= M_PI;
                }
                else if (steer_bl_rad < ((-1) * M_PI / 2.))
                {
                    steer_bl_rad += M_PI;
                }
            }
            

            float temp_x, temp_y;
            temp_x = wheel_base_fr.x;
            temp_y = wheel_base_fr.y;
            wheel_base_fr.x = (temp_x - wheel_point_fr.x) * cos(steer_fr_rad) - (temp_y - wheel_point_fr.y) * sin(steer_fr_rad) + wheel_point_fr.x;
            wheel_base_fr.y = (temp_x - wheel_point_fr.x) * sin(steer_fr_rad) + (temp_y - wheel_point_fr.y) * cos(steer_fr_rad) + wheel_point_fr.y;

            temp_x = wheel_base_fl.x;
            temp_y = wheel_base_fl.y;
            wheel_base_fl.x = (temp_x - wheel_point_fl.x) * cos(steer_fl_rad) - (temp_y - wheel_point_fl.y) * sin(steer_fl_rad) + wheel_point_fl.x;
            wheel_base_fl.y = (temp_x - wheel_point_fl.x) * sin(steer_fl_rad) + (temp_y - wheel_point_fl.y) * cos(steer_fl_rad) + wheel_point_fl.y;

            temp_x = wheel_base_br.x;
            temp_y = wheel_base_br.y;
            wheel_base_br.x = (temp_x - wheel_point_br.x) * cos(steer_br_rad) - (temp_y - wheel_point_br.y) * sin(steer_br_rad) + wheel_point_br.x;
            wheel_base_br.y = (temp_x - wheel_point_br.x) * sin(steer_br_rad) + (temp_y - wheel_point_br.y) * cos(steer_br_rad) + wheel_point_br.y;

            temp_x = wheel_base_bl.x;
            temp_y = wheel_base_bl.y;
            wheel_base_bl.x = (temp_x - wheel_point_bl.x) * cos(steer_bl_rad) - (temp_y - wheel_point_bl.y) * sin(steer_bl_rad) + wheel_point_bl.x;
            wheel_base_bl.y = (temp_x - wheel_point_bl.x) * sin(steer_bl_rad) + (temp_y - wheel_point_bl.y) * cos(steer_bl_rad) + wheel_point_bl.y;


            float vel_value = (base_vel / (M_PI*WHEEL_DIAMETER) * 60. / VEL_UNIT);
            if(LIMIT_VEL_VALUE < vel_value) {
                wheel_fr_goal_vel = LIMIT_VEL_VALUE * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_fl_goal_vel = LIMIT_VEL_VALUE * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_br_goal_vel = LIMIT_VEL_VALUE * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_bl_goal_vel = LIMIT_VEL_VALUE * (vel_twist.angular.z / abs(vel_twist.angular.z));
            }
            else {
                wheel_fr_goal_vel = vel_value * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_fl_goal_vel = vel_value * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_br_goal_vel = vel_value * (vel_twist.angular.z / abs(vel_twist.angular.z));
                wheel_bl_goal_vel = vel_value * (vel_twist.angular.z / abs(vel_twist.angular.z));
            }

            if ((acos(((wheel_base_fr.x - wheel_point_fr.x) * (base_center.x - wheel_point_fr.x)) + ((wheel_base_fr.y - wheel_point_fr.y) * (base_center.y - wheel_point_fr.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_fr.x - wheel_point_fr.x) * (base_center.x - wheel_point_fr.x)) + ((wheel_base_fr.y - wheel_point_fr.y) * (base_center.y - wheel_point_fr.y)))))) {
                wheel_fr_goal_vel *= -1;
            }
            if ((acos(((wheel_base_fl.x - wheel_point_fl.x) * (base_center.x - wheel_point_fl.x)) + ((wheel_base_fl.y - wheel_point_fl.y) * (base_center.y - wheel_point_fl.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_fl.x - wheel_point_fl.x) * (base_center.x - wheel_point_fl.x)) + ((wheel_base_fl.y - wheel_point_fl.y) * (base_center.y - wheel_point_fl.y)))))) {
                wheel_fl_goal_vel *= -1;
            }
            if ((acos(((wheel_base_br.x - wheel_point_br.x) * (base_center.x - wheel_point_br.x)) + ((wheel_base_br.y - wheel_point_br.y) * (base_center.y - wheel_point_br.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_br.x - wheel_point_br.x) * (base_center.x - wheel_point_br.x)) + ((wheel_base_br.y - wheel_point_br.y) * (base_center.y - wheel_point_br.y)))))) {
                wheel_br_goal_vel *= -1;
            }
            if ((acos(((wheel_base_bl.x - wheel_point_bl.x) * (base_center.x - wheel_point_bl.x)) + ((wheel_base_bl.y - wheel_point_bl.y) * (base_center.y - wheel_point_bl.y))) < (M_PI/2)) || (std::isnan(acos(((wheel_base_bl.x - wheel_point_bl.x) * (base_center.x - wheel_point_bl.x)) + ((wheel_base_bl.y - wheel_point_bl.y) * (base_center.y - wheel_point_bl.y)))))) {
                wheel_bl_goal_vel *= -1;
            }

            wheel_fr_goal_vel *= (r_wheel_fr / r);
            wheel_fl_goal_vel *= (r_wheel_fl / r);
            wheel_br_goal_vel *= (r_wheel_br / r);
            wheel_bl_goal_vel *= (r_wheel_bl / r);

            steer_fr_goal_angle = steer_fr_rad * 180. / M_PI * 4096. / 360.;
            steer_fl_goal_angle = steer_fl_rad * 180. / M_PI * 4096. / 360.;
            steer_br_goal_angle = steer_br_rad * 180. / M_PI * 4096. / 360.;
            steer_bl_goal_angle = steer_bl_rad * 180. / M_PI * 4096. / 360.;

            // ROS_INFO("Wheel INFO(Swivel motion)\n\t steer_fr_deg = %.3f\n\t steer_fl_deg = %.3f\n\t steer_br_deg = %.3f\n\t steer_bl_deg = %.3f\n\t wheel_fr_goal_vel = %.3f\n\t wheel_fl_goal_vel = %.3f\n\t wheel_br_goal_vel = %.3f\n\t wheel_bl_goal_vel = %.3f", steer_fr_rad*180./M_PI, steer_fl_rad*180./M_PI, steer_br_rad*180./M_PI, steer_bl_rad*180./M_PI, wheel_fr_goal_vel, wheel_fl_goal_vel, wheel_br_goal_vel, wheel_bl_goal_vel );
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
