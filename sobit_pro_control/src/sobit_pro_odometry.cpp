#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Calculate Odometry
bool SobitProOdometry::odom(int32_t steer_fr_curt_position, int32_t steer_fl_curt_position,
                            int32_t steer_br_curt_position, int32_t steer_bl_curt_position,
                            int32_t wheel_fr_curt_velocity, int32_t wheel_fl_curt_velocity,
                            int32_t wheel_br_curt_velocity, int32_t wheel_bl_curt_velocity,
                            int32_t wheel_fr_curt_position, int32_t wheel_fl_curt_position,
                            int32_t wheel_br_curt_position, int32_t wheel_bl_curt_position,
                            int32_t wheel_fr_init_position, int32_t wheel_fl_init_position,
                            int32_t wheel_br_init_position, int32_t wheel_bl_init_position,
                            int32_t prev_motion,
                            nav_msgs::Odometry prev_odom, nav_msgs::Odometry* result_odom,
                            ros::Time prev_time){

    float fr_distance_m    = distance_calculation(wheel_fr_curt_position - wheel_fr_init_position); // Calculation distance[m]
    float fl_distance_m    = distance_calculation(wheel_fl_curt_position - wheel_fl_init_position); // Calculation distance[m]
    float br_distance_m    = distance_calculation(wheel_br_curt_position - wheel_br_init_position); // Calculation distance[m]
    float bl_distance_m    = distance_calculation(wheel_bl_curt_position - wheel_bl_init_position); // Calculation distance[m]

    float fr_direction_deg = position_calculation(steer_fr_curt_position); // Record present position
    float fl_direction_deg = position_calculation(steer_fl_curt_position); // Record present position
    float br_direction_deg = position_calculation(steer_br_curt_position); // Record present position
    float bl_direction_deg = position_calculation(steer_bl_curt_position); // Record present position

    double prev_odom_orientation_x = 0., prev_odom_orientation_y = 0., prev_odom_orientation_z = 0.;
    float distance_m;
    nav_msgs::Odometry calculation_odom = *result_odom;
    tf::Quaternion quat_tf; 

    if     ( prev_motion == 0 ){
        motion_mode = STOP_MOTION_MODE;
    }
    else if( prev_motion == 1 ){
        motion_mode = TRANSLATIONAL_MOTION_MODE;
    }
    else if( prev_motion == 2 ){
        motion_mode = ROTATIONAL_MOTION_MODE;
    }
    else if( prev_motion == 3 ){
        motion_mode = SWIVEL_MOTION_MODE;
    }

    switch( motion_mode ){
        // Translational motion
        case TRANSLATIONAL_MOTION_MODE:{

            // Convert Wheel Coordinates to Robot Coordinates
            fr_direction_deg = fr_direction_deg + 45.;
            fl_direction_deg = fl_direction_deg - 45.;
            // Check!!
            br_direction_deg = fr_direction_deg - 45.;
            bl_direction_deg = fl_direction_deg + 45.;
            
            // Check the calculation
            // if(true){
            if(0 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1){

                // Positive distance or Negative distance
                if(-45 <= fr_direction_deg && fr_direction_deg <= 90){
                    if(0. <= fr_distance_m){
                        distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                    }else{
                        distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                    }
                }
                else if(90 < fr_direction_deg && fr_direction_deg <= 135){
                    if(0. <= fr_distance_m){
                        distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                    }else{
                        distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                    }
                }
                else ROS_ERROR("Calculation ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
            }
            else {
                ROS_ERROR("Odometry ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
            }

            // Change euler (prev_odom)
            quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            tf::Matrix3x3(quat_tf).getRPY(prev_odom_orientation_x, prev_odom_orientation_y, prev_odom_orientation_z);

            // Add the amount of movement to the odometry
            calculation_odom.pose.pose.position.x = prev_odom.pose.pose.position.x
                                                    + distance_m * cos(fr_direction_deg * (M_PI / 180.)) * cos(prev_odom_orientation_z)
                                                    + distance_m * sin(fr_direction_deg * (M_PI / 180.)) * cos(prev_odom_orientation_z + 1.5708);

            calculation_odom.pose.pose.position.y = prev_odom.pose.pose.position.y
                                                    + distance_m * cos(fr_direction_deg * (M_PI / 180.)) * sin(prev_odom_orientation_z)
                                                    + distance_m * sin(fr_direction_deg * (M_PI / 180.)) * sin(prev_odom_orientation_z + 1.5708);

            calculation_odom.pose.pose.orientation = prev_odom.pose.pose.orientation;

            // Info
            // ROS_INFO("calculation_odom.pose.pose.position.x    = %.3f", calculation_odom.pose.pose.position.x);
            // ROS_INFO("calculation_odom.pose.pose.position.y    = %.3f", calculation_odom.pose.pose.position.y);
            // ROS_INFO("calculation_odom.pose.pose.position.z    = %.3f", calculation_odom.pose.pose.position.z);
            // ROS_INFO("calculation_odom.pose.pose.orientation.x = %.3f", calculation_odom.pose.pose.orientation.x);
            // ROS_INFO("calculation_odom.pose.pose.orientation.y = %.3f", calculation_odom.pose.pose.orientation.y);
            // ROS_INFO("calculation_odom.pose.pose.orientation.z = %.3f", calculation_odom.pose.pose.orientation.z);
            // ROS_INFO("calculation_odom.pose.pose.orientation.w = %.3f", calculation_odom.pose.pose.orientation.w);

            // Debug
            if (std::isnan(calculation_odom.pose.pose.position.x)    || std::isnan(calculation_odom.pose.pose.position.y)) ROS_ERROR("------ Odom calculation : Nan error in TRANSLATIONAL_MOTION_MODE pose ------");
            if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) ROS_ERROR("------ Odom calculation : Nan error in TRANSLATIONAL_MOTION_MODE orientation------");

            *result_odom = calculation_odom;

            return true;
        }

        // Rotational motion
        case ROTATIONAL_MOTION_MODE:{
            float rotational_position_deg;
            float rotational_position_rad;

            // Check the calculation
            // if(0. <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1.){
            if(0 <= std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) && std::abs(std::abs(fr_direction_deg) - std::abs(fl_direction_deg)) <= 1){

                // Positive distance or Negative distance
                if(0 <= fr_distance_m){
                    distance_m = (std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                }
                else{
                    distance_m = -(std::abs(fr_distance_m) + std::abs(fl_distance_m)) / 2.;
                }
            }
            else ROS_ERROR("Odometry ERROR : Rotational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);

            // Change euler (prev_odom)
            quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            tf::Matrix3x3(quat_tf).getRPY(prev_odom_orientation_x, prev_odom_orientation_y, prev_odom_orientation_z);

            // Add the amount of movement to the odometry
            rotational_position_deg = (distance_m * 360.) / (BODY_DIAMETER * M_PI);
            rotational_position_rad = prev_odom_orientation_z + (rotational_position_deg * (M_PI / 180.)); // rad = deg * (M_PI / 180.)

            // Change quaternion (calculation_odom)
            tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., rotational_position_rad);
            quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);
            if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) {
                ROS_ERROR("------ Odom calculation : Nan error !! : Rotational motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
                *result_odom = prev_odom;
                return true;
            }
            if (std::isinf(calculation_odom.pose.pose.orientation.x) || std::isinf(calculation_odom.pose.pose.orientation.y) || std::isinf(calculation_odom.pose.pose.orientation.z) || std::isinf(calculation_odom.pose.pose.orientation.w)) {
                ROS_ERROR("------ Odom calculation : Inf error !! : Rotational motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
                *result_odom = prev_odom;
                return true;
            }
            *result_odom = calculation_odom;

            return true;
        }

        // Swivel motion
        case SWIVEL_MOTION_MODE:{
            float svl_orientation_rad;
            float svl_r, svl_r_in, svl_r_out, svl_deg, svl_dis;
            float pose_x = 0., pose_y = 0., ori_z =0.;
            float d = BODY_DIAMETER / 2. * cos(M_PI / 4.); // Manhattan distance (cos value shud be change)

            // Change euler (prev_odom)
            quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            tf::Matrix3x3(quat_tf).getRPY(prev_odom_orientation_x, prev_odom_orientation_y, prev_odom_orientation_z);

            if( ( fl_direction_deg >= -45 && fl_direction_deg < 0 ) && ( fr_direction_deg >= -22.5 && fr_direction_deg < 0 )){
                svl_r_in  = d / cos((45. + fl_direction_deg) * M_PI / 180.);
                svl_r_out = d / cos((45. - fr_direction_deg) * M_PI / 180.);
                svl_r     = (svl_r_out * sin((45. - fr_direction_deg) * M_PI / 180.) - svl_r_in * sin((45. + fl_direction_deg) * M_PI / 180.)) / 2.;
                // ROS_ERROR("fl_deg    : %f\n", fl_direction_deg);
                // ROS_ERROR("fr_deg    : %f\n", fr_direction_deg);
                // ROS_ERROR("svl_r_in  : %f\n", svl_r_in);
                // ROS_ERROR("svl_r_out : %f\n", svl_r_out);
                // ROS_ERROR("svl_r     : %f\n", svl_r);
                svl_deg = (std::abs(fl_distance_m) / (2. * svl_r_in * M_PI) * 360. + std::abs(fr_distance_m) / (2. * svl_r_out * M_PI) * 360.) / 2.;
                svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * M_PI / 180.)));
                if(fr_distance_m >= 0){
                    pose_x = svl_dis * sin((180. - svl_deg) / 2 * M_PI / 180. - prev_odom_orientation_z);
                    pose_y = svl_dis * cos((180. - svl_deg) / 2 * M_PI / 180. - prev_odom_orientation_z);
                    ori_z  = svl_deg;
                }
                else{
                    pose_x = -svl_dis * sin((180. - svl_deg) / 2 * M_PI / 180. - prev_odom_orientation_z);
                    pose_y = -svl_dis * cos((180. - svl_deg) / 2 * M_PI / 180. - prev_odom_orientation_z);
                    ori_z  = -svl_deg;
                }
            }
            else if(fl_direction_deg >= 0 && fl_direction_deg < 22.5 && fr_direction_deg >= 0 && fr_direction_deg < 45){
                svl_r_in  = d / cos((45. - fr_direction_deg) * M_PI / 180.);
                svl_r_out = d / cos((45. + fl_direction_deg) * M_PI / 180.);
                svl_r     = (svl_r_out * sin((45. + fl_direction_deg) * M_PI / 180.) - svl_r_in * sin((45. - fr_direction_deg) * M_PI / 180.)) / 2.;
                // ROS_ERROR("fl_deg    : %f\n", fl_direction_deg);
                // ROS_ERROR("fr_deg    : %f\n", fr_direction_deg);
                // ROS_ERROR("svl_r_in  : %f\n", svl_r_in);
                // ROS_ERROR("svl_r_out : %f\n", svl_r_out);
                // ROS_ERROR("svl_r     : %f\n", svl_r);
                svl_deg = (std::abs(fr_distance_m) / (2. * svl_r_in * M_PI) * 360. + std::abs(fl_distance_m) / (2. * svl_r_out * M_PI) * 360.) / 2.;
                svl_dis = sqrt(2. * powf(svl_r, 2.) * (1 - cos(svl_deg * M_PI / 180.)));
                // ROS_ERROR("svl_deg : %f\n", svl_deg);
                // ROS_ERROR("svl_dis : %f\n", svl_dis);
                if(fr_distance_m >= 0){
                    pose_x = -svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                    pose_y = -svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                    ori_z  = svl_deg;
                }
                else{
                    pose_x = svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                    pose_y = svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                    ori_z  = -svl_deg;
                }
            }

            else if(abs(fr_direction_deg) <= 45 && abs(fl_direction_deg) >= 45){
                if(fl_direction_deg > -45){
                    fl_direction_deg += 45.;
                    fr_direction_deg += 135.;
                }
                else{
                    fl_direction_deg += 225.;
                    fr_direction_deg += 135.;
                }

                svl_r_in  = d / sin((fl_direction_deg - 90.) * M_PI / 180.);
                svl_r_out = d / sin((fr_direction_deg - 90.) * M_PI / 180.);
                svl_r     = (svl_r_in + svl_r_out) / 2.;
                svl_deg   = (std::abs(fl_distance_m) / (2. * svl_r_in * M_PI) * 360. + std::abs(fr_distance_m) / (2. * svl_r_out * M_PI) * 360.) / 2.;
                svl_dis   = sqrt(2. * powf(svl_r, 2.) * (1. - cos(svl_deg * M_PI / 180.)));

                if(90 <= fl_direction_deg && fl_direction_deg < 135){
                    if(fl_distance_m <= 0){
                        pose_x = svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = svl_deg;
                    }
                    else{
                        pose_x = -svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = -svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = -svl_deg;
                    }
                }
                else if(135 <= fl_direction_deg && fl_direction_deg <= 180){
                    if(fl_distance_m >= 0){
                        pose_x = svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = svl_deg;
                    }
                    else{
                        pose_x = -svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = -svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = -svl_deg;
                    }
                }
                else ROS_ERROR("Calculation ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
            }
            else if(abs(fr_direction_deg) >= 45 && abs(fl_direction_deg) <= 45){
                if(fr_direction_deg < 45){
                    fr_direction_deg = 45.  - fr_direction_deg;
                    fl_direction_deg = 135. - fl_direction_deg;
                }
                else{
                    fr_direction_deg = 225. - fr_direction_deg;
                    fl_direction_deg = 135. - fl_direction_deg;
                }

                svl_r_in  = d / sin((fr_direction_deg - 90.) * M_PI / 180.);
                svl_r_out = d / sin((fl_direction_deg - 90.) * M_PI / 180.);
                svl_r     = (svl_r_in + svl_r_out) / 2.;
                svl_deg   = (std::abs(fr_distance_m) / (2. * svl_r_in * M_PI) * 360. + std::abs(fl_distance_m) / (2. * svl_r_out * M_PI) * 360.) / 2.;
                svl_dis   = sqrt(2. * powf(svl_r, 2.) * (1. - cos(svl_deg * M_PI / 180.)));
                if(90 <= fr_direction_deg && fr_direction_deg < 135){
                    if(fr_distance_m >= 0){
                        pose_x = svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = -svl_deg;
                    }
                    else{
                        pose_x = -svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = -svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = svl_deg;
                    }
                }
                else if(135 <= fr_direction_deg && fr_direction_deg <= 180){
                    if(fr_distance_m >= 0){
                        pose_x = -svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = -svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = svl_deg;
                    }
                    else{
                        pose_x = svl_dis * sin((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        pose_y = svl_dis * cos((180. - svl_deg) / 2. * M_PI / 180. - prev_odom_orientation_z);
                        ori_z  = -svl_deg;
                    }
                }
                else ROS_ERROR("Calculation ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
            }
            else{
                    ROS_ERROR("Odometry ERROR : Swivel motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
                    pose_x = 0.; pose_y = 0.; ori_z = 0.;
                    *result_odom = prev_odom;
                    return true;
            }
            // debug
            if (std::isnan(pose_x) || std::isnan(pose_y) || std::isnan(ori_z)) {
                ROS_ERROR("------ Odom calculation : Nan error !! : Swivel motion ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", pose_x, pose_y, ori_z );
                *result_odom = prev_odom;
                return true;
            }
            if (std::isinf(pose_x) || std::isinf(pose_y) || std::isinf(ori_z)) {
                ROS_ERROR("------ Odom calculation : Inf error !! : Swivel motion ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", pose_x, pose_y, ori_z );
                *result_odom = prev_odom;
                return true;
            }

            // Add the amount of movement to the odometry
            calculation_odom.pose.pose.position.x = prev_odom.pose.pose.position.x + pose_x;
            calculation_odom.pose.pose.position.y = prev_odom.pose.pose.position.y + pose_y;
            calculation_odom.pose.pose.position.z = 0.;
            svl_orientation_rad = prev_odom_orientation_z + (ori_z * (M_PI / 180.)); // rad = deg * (M_PI / 180.)

            // debug
            if (std::isnan(calculation_odom.pose.pose.position.x) || std::isnan(calculation_odom.pose.pose.position.y) || std::isnan(svl_orientation_rad)) {
                ROS_ERROR("------ Odom calculation : Nan error !! : Swivel motion(calculation_odom) ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, svl_orientation_rad );
                *result_odom = prev_odom;
                return true;
            }
            if (std::isinf(calculation_odom.pose.pose.position.x) || std::isinf(calculation_odom.pose.pose.position.y) || std::isinf(svl_orientation_rad)) {
                ROS_ERROR("------ Odom calculation : Inf error !! : Swivel motion(calculation_odom) ------\nx = %.3f,\ty = %.3f,\tyaw = %.3f,\t", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, svl_orientation_rad );
                *result_odom = prev_odom;
                return true;
            }
            // Change quaternion (calculation_odom)
            tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., svl_orientation_rad);
            quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

            // debug
            if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) {
                ROS_ERROR("------ Odom calculation : Nan error !! : Swivel motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
                *result_odom = prev_odom;
                return true;
            }
            if (std::isinf(calculation_odom.pose.pose.orientation.x) || std::isinf(calculation_odom.pose.pose.orientation.y) || std::isinf(calculation_odom.pose.pose.orientation.z) || std::isinf(calculation_odom.pose.pose.orientation.w)) {
                ROS_ERROR("------ Odom calculation : Inf error !! : Swivel motion(orientation) ------\nx = %.3f,\ty = %.3f,\tz = %.3f,\tw = %.3f", calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, calculation_odom.pose.pose.orientation.z, calculation_odom.pose.pose.orientation.w );
                *result_odom = prev_odom;
                return true;
            }
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
float SobitProOdometry::distance_calculation(float wheel_curt_position){
    float distance_m;

    distance_m = WHEEL_LENGTH * wheel_curt_position / 4096.;

    return distance_m;
}

// Position calculation
float SobitProOdometry::position_calculation(float steer_curt_position){
    float direction_deg;

    direction_deg = (steer_curt_position - 2048.) * 360. / 4096.;

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