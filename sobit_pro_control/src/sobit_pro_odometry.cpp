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

    double prev_roll = 0., prev_pitch = 0., prev_yaw = 0.;
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
            tf::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

            // Add the amount of movement to the odometry
            calculation_odom.pose.pose.position.x = prev_odom.pose.pose.position.x
                                                    + distance_m * cos(fr_direction_deg * (M_PI / 180.)) * cos(prev_yaw)
                                                    + distance_m * sin(fr_direction_deg * (M_PI / 180.)) * cos(prev_yaw + 1.5708);

            calculation_odom.pose.pose.position.y = prev_odom.pose.pose.position.y
                                                    + distance_m * cos(fr_direction_deg * (M_PI / 180.)) * sin(prev_yaw)
                                                    + distance_m * sin(fr_direction_deg * (M_PI / 180.)) * sin(prev_yaw + 1.5708);

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
            // Change euler (prev_odom)
            quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            tf::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            float yaw = 0.;
            yaw = ((fr_distance_m + fl_distance_m + br_distance_m + bl_distance_m) / 4.) / (TRACK / sqrtf(2.));

            // Change quaternion (calculation_odom)
            tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., (prev_yaw + yaw));
            quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

            *result_odom = calculation_odom;
            return true;
        }

        // Swivel motion
        case SWIVEL_MOTION_MODE:{
            // Change euler (prev_odom)
            quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            tf::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            float pose_x = 0., pose_y = 0., yaw = 0.;
            geometry_msgs::Point wheel_point_fr, wheel_point_fl, wheel_point_br, wheel_point_bl;
            wheel_point_fr.x = TRACK / 2.;
            wheel_point_fr.y = TRACK / 2. * (-1);
            wheel_point_fl.x = TRACK / 2.;
            wheel_point_fl.y = TRACK / 2.;
            wheel_point_br.x = TRACK / 2. * (-1);
            wheel_point_br.y = TRACK / 2. * (-1);
            wheel_point_bl.x = TRACK / 2. * (-1);
            wheel_point_bl.y = TRACK / 2.;
            float fr_direction_rad = fr_direction_deg * M_PI / 180. ;
            float fl_direction_rad = fl_direction_deg * M_PI / 180. ;
            float br_direction_rad = br_direction_deg * M_PI / 180. ;
            float bl_direction_rad = bl_direction_deg * M_PI / 180. ;
            float a_fr = tan(atan2(wheel_point_fr.y, wheel_point_fr.x) + fr_direction_rad);
            float a_fl = tan(atan2(wheel_point_fl.y, wheel_point_fl.x) + fl_direction_rad);
            float a_br = tan(atan2(wheel_point_br.y, wheel_point_br.x) + br_direction_rad);
            float a_bl = tan(atan2(wheel_point_bl.y, wheel_point_bl.x) + bl_direction_rad);
            geometry_msgs::Point base_center;
            if (abs(a_fr - a_bl) > abs(a_fl - a_br))
            {
                // fr and bl
                base_center.x = (a_fr * wheel_point_fr.x - a_bl * wheel_point_bl.x + wheel_point_bl.y - wheel_point_fr.y) / (a_fr - a_bl);
                base_center.y = a_fr * (base_center.x - wheel_point_fr.x) + wheel_point_fr.y;
                if (sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.)))
                {
                    yaw = fr_distance_m / (sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.)));
                }
                else
                {
                    yaw = bl_distance_m / (sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.)));
                }
            }
            else
            {
                // fl and br
                base_center.x = (a_fl * wheel_point_fl.x - a_br * wheel_point_br.x + wheel_point_br.y - wheel_point_fl.y) / (a_fl - a_br);
                base_center.y = a_fl * (base_center.x - wheel_point_fl.x) + wheel_point_fl.y;
                if (sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.)))
                {
                    yaw = fl_distance_m / (sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.)));
                }
                else
                {
                    yaw = br_distance_m / (sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.)));
                }
            }
            pose_x = (0. - base_center.x) * cos(yaw) - (0. - base_center.y) * sin(yaw) + base_center.x;
            pose_y = (0. - base_center.x) * sin(yaw) + (0. - base_center.y) * cos(yaw) + base_center.y;
            if (std::isnan(pose_x))
            {
                pose_x = 0.;
            }
            if (std::isnan(pose_y))
            {
                pose_y = 0.;
            }
            if (std::isnan(yaw))
            {
                yaw = 0.;
            }

            calculation_odom.pose.pose.position.x = pose_x * cos(prev_yaw) - pose_y * sin(prev_yaw) + prev_odom.pose.pose.position.x;
            calculation_odom.pose.pose.position.y = pose_x * sin(prev_yaw) + pose_y * cos(prev_yaw) + prev_odom.pose.pose.position.y;
            calculation_odom.pose.pose.position.z = 0.;

            // Change quaternion (calculation_odom)
            tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., (prev_yaw + yaw));
            quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);


            ROS_INFO("base_center = %.4f, %.4f,  yaw = %.4f", base_center.x, base_center.y, yaw);
            ROS_INFO("odom = %.4f, %.4f, %.4f\n",calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, (prev_yaw + yaw));

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