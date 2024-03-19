#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Calculate Odometry
bool SobitProOdometry::odom(int32_t steer_fl_curt_pos, int32_t steer_fr_curt_pos,
                            int32_t steer_bl_curt_pos, int32_t steer_br_curt_pos,
                            // int32_t wheel_fl_curt_vel, int32_t wheel_fr_curt_vel,
                            // int32_t wheel_bl_curt_vel, int32_t wheel_br_curt_vel,
                            int32_t wheel_fl_curt_pos, int32_t wheel_fr_curt_pos,
                            int32_t wheel_bl_curt_pos, int32_t wheel_br_curt_pos,
                            int32_t wheel_fl_init_pos, int32_t wheel_fr_init_pos,
                            int32_t wheel_bl_init_pos, int32_t wheel_br_init_pos,
                            int32_t prev_motion,
                            nav_msgs::Odometry prev_odom, nav_msgs::Odometry* result_odom,
                            ros::Time prev_time){

    double fl_distance_m    = distance_calculation(wheel_fl_curt_pos - wheel_fl_init_pos); // Calculation distance[m]
    double fr_distance_m    = distance_calculation(wheel_fr_curt_pos - wheel_fr_init_pos); // Calculation distance[m]
    double bl_distance_m    = distance_calculation(wheel_bl_curt_pos - wheel_bl_init_pos); // Calculation distance[m]
    double br_distance_m    = distance_calculation(wheel_br_curt_pos - wheel_br_init_pos); // Calculation distance[m]

    double fl_direction_deg = position_calculation(steer_fl_curt_pos); // Record present position
    double fr_direction_deg = position_calculation(steer_fr_curt_pos); // Record present position
    double bl_direction_deg = position_calculation(steer_bl_curt_pos); // Record present position
    double br_direction_deg = position_calculation(steer_br_curt_pos); // Record present position

    double prev_roll = 0., prev_pitch = 0., prev_yaw = 0.;
    double distance_m;
    nav_msgs::Odometry calculation_odom = *result_odom;
    tf2::Quaternion quat_tf;

    if     ( prev_motion == STOP_MOTION_MODE )          motion_mode = STOP_MOTION_MODE;
    else if( prev_motion == TRANSLATIONAL_MOTION_MODE ) motion_mode = TRANSLATIONAL_MOTION_MODE;
    else if( prev_motion == ROTATIONAL_MOTION_MODE )    motion_mode = ROTATIONAL_MOTION_MODE;
    else if( prev_motion == SWIVEL_MOTION_MODE )        motion_mode = SWIVEL_MOTION_MODE;

    switch( motion_mode ){
        // Translational motion
        case TRANSLATIONAL_MOTION_MODE:{
            // Convert Wheel Coordinates to Robot Coordinates
            fl_direction_deg = fl_direction_deg - 45.;
            fr_direction_deg = fr_direction_deg + 45.;
            // Check!!
            bl_direction_deg = fl_direction_deg + 45.;
            br_direction_deg = fr_direction_deg - 45.;
            
            // Check the calculation
            // if( (0 <= fabsf(fabsf(fr_direction_deg) - fabsf(fl_direction_deg))) && (fabsf(fabsf(fr_direction_deg) - fabsf(fl_direction_deg))) <= 1) ){
            if( 0 <= fabsf(fabsf(fr_direction_deg) - fabsf(fl_direction_deg)) <= 1 ){
                // Positive distance or Negative distance
                if( (-45 <= fr_direction_deg) && (fr_direction_deg <= 90) ){
                    if( 0. <= fr_distance_m ) distance_m =  (fabsf(fr_distance_m) + fabsf(fl_distance_m)) / 2.;
                    else                      distance_m = -(fabsf(fr_distance_m) + fabsf(fl_distance_m)) / 2.;
                }
                else if(90 < fr_direction_deg && fr_direction_deg <= 135){
                    if( 0. <= fr_distance_m ) distance_m = -(fabsf(fr_distance_m) + fabsf(fl_distance_m)) / 2.;
                    else                      distance_m = (fabsf(fr_distance_m) + fabsf(fl_distance_m)) / 2.;
                }
                else ROS_ERROR("Calculation ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);
            }
            else ROS_ERROR("Odometry ERROR : Translational motion\nfr_distance_m = %.3f\tfl_distance_m = %.3f\tfr_direction_deg = %.3f\tfl_direction_deg = %.3f", fr_distance_m, fl_distance_m, fr_direction_deg, fl_direction_deg);

            // Transform euler to RPY (prev_odom)
            tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
            tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            // quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            // tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

            // Add the amount of movement to the odometry
            calculation_odom.pose.pose.position.x = prev_odom.pose.pose.position.x
                                                    + distance_m * cosf(fr_direction_deg * (M_PI / 180.)) * cosf(prev_yaw)
                                                    + distance_m * sinf(fr_direction_deg * (M_PI / 180.)) * cosf(prev_yaw + 1.5708);

            calculation_odom.pose.pose.position.y = prev_odom.pose.pose.position.y
                                                    + distance_m * cosf(fr_direction_deg * (M_PI / 180.)) * sinf(prev_yaw)
                                                    + distance_m * sinf(fr_direction_deg * (M_PI / 180.)) * sinf(prev_yaw + 1.5708);

            calculation_odom.pose.pose.position.z = prev_odom.pose.pose.position.z; 

            calculation_odom.pose.pose.orientation.x = prev_odom.pose.pose.orientation.x;
            calculation_odom.pose.pose.orientation.y = prev_odom.pose.pose.orientation.y;
            calculation_odom.pose.pose.orientation.z = prev_odom.pose.pose.orientation.z;
            calculation_odom.pose.pose.orientation.w = prev_odom.pose.pose.orientation.w;

            // Info
            // ROS_INFO("calculation_odom.pose.pose.position.x    = %.3f", calculation_odom.pose.pose.position.x);
            // ROS_INFO("calculation_odom.pose.pose.position.y    = %.3f", calculation_odom.pose.pose.position.y);
            // ROS_INFO("calculation_odom.pose.pose.position.z    = %.3f", calculation_odom.pose.pose.position.z);
            // ROS_INFO("calculation_odom.pose.pose.orientation.x = %.3f", calculation_odom.pose.pose.orientation.x);
            // ROS_INFO("calculation_odom.pose.pose.orientation.y = %.3f", calculation_odom.pose.pose.orientation.y);
            // ROS_INFO("calculation_odom.pose.pose.orientation.z = %.3f", calculation_odom.pose.pose.orientation.z);
            // ROS_INFO("calculation_odom.pose.pose.orientation.w = %.3f", calculation_odom.pose.pose.orientation.w);

            // Debug
            if (std::isnan(calculation_odom.pose.pose.position.x)    || std::isnan(calculation_odom.pose.pose.position.y)) ROS_ERROR("------ Odom calculation : Nan error in TRANSLATIONAL_MOTION_MODE (pose) ------");
            if (std::isnan(calculation_odom.pose.pose.orientation.x) || std::isnan(calculation_odom.pose.pose.orientation.y) || std::isnan(calculation_odom.pose.pose.orientation.z) || std::isnan(calculation_odom.pose.pose.orientation.w)) ROS_ERROR("------ Odom calculation : Nan error in TRANSLATIONAL_MOTION_MODE (orientation) ------");

            *result_odom = calculation_odom;


            return true;
        }

        // Rotational motion
        case ROTATIONAL_MOTION_MODE:{
            // Transform euler->RPY (prev_odom)
            tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
            tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            // quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            // tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

            double yaw = 0.;
            yaw = ((fl_distance_m + fr_distance_m + bl_distance_m + br_distance_m) / 4.) / (SobitProControl::TRACK / sqrtf(2.));

            // Transform quaternion->msg (calculation_odom)
            quat_tf.setRPY(0., 0., (prev_yaw + yaw));
            tf2::convert(quat_tf, calculation_odom.pose.pose.orientation);
            // tf2::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., (prev_yaw + yaw));
            // quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

            *result_odom = calculation_odom;


            return true;
        }

        // Swivel motion
        case SWIVEL_MOTION_MODE:{
            // Transform euler->RPY (prev_odom)
            tf2::fromMsg(prev_odom.pose.pose.orientation, quat_tf);
            tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            // quaternionMsgToTF(prev_odom.pose.pose.orientation, quat_tf);
            // tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);
            
            double pose_x = 0., pose_y = 0., yaw = 0.;
            geometry_msgs::Point wheel_point_fl, wheel_point_fr, wheel_point_bl, wheel_point_br;

            wheel_point_fl.x = SobitProControl::TRACK / 2.;
            wheel_point_fl.y = SobitProControl::TRACK / 2.;
            wheel_point_fr.x = SobitProControl::TRACK / 2.;
            wheel_point_fr.y = SobitProControl::TRACK / 2. * (-1);
            wheel_point_bl.x = SobitProControl::TRACK / 2. * (-1);
            wheel_point_bl.y = SobitProControl::TRACK / 2.;
            wheel_point_br.x = SobitProControl::TRACK / 2. * (-1);
            wheel_point_br.y = SobitProControl::TRACK / 2. * (-1);

            double fl_direction_rad = fl_direction_deg * M_PI / 180. ;
            double fr_direction_rad = fr_direction_deg * M_PI / 180. ;
            double bl_direction_rad = bl_direction_deg * M_PI / 180. ;
            double br_direction_rad = br_direction_deg * M_PI / 180. ;

            double a_fl = tanf(atan2f(wheel_point_fl.y, wheel_point_fl.x) + fl_direction_rad);
            double a_fr = tanf(atan2f(wheel_point_fr.y, wheel_point_fr.x) + fr_direction_rad);
            double a_bl = tanf(atan2f(wheel_point_bl.y, wheel_point_bl.x) + bl_direction_rad);
            double a_br = tanf(atan2f(wheel_point_br.y, wheel_point_br.x) + br_direction_rad);

            geometry_msgs::Point base_center;

            if( fabsf(a_fr - a_bl) > fabsf(a_fl - a_br) ){
                // fr and bl
                base_center.x = (a_fr * wheel_point_fr.x - a_bl * wheel_point_bl.x + wheel_point_bl.y - wheel_point_fr.y) / (a_fr - a_bl);
                base_center.y = a_fr * (base_center.x - wheel_point_fr.x) + wheel_point_fr.y;
                
                if( sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.)) ){
                    yaw = fr_distance_m / (sqrtf(powf((wheel_point_fr.x - base_center.x), 2.) + powf((wheel_point_fr.y - base_center.y), 2.)));
                }else{
                    yaw = bl_distance_m / (sqrtf(powf((wheel_point_bl.x - base_center.x), 2.) + powf((wheel_point_bl.y - base_center.y), 2.)));
                }
            } else{
                // fl and br
                base_center.x = (a_fl * wheel_point_fl.x - a_br * wheel_point_br.x + wheel_point_br.y - wheel_point_fl.y) / (a_fl - a_br);
                base_center.y = a_fl * (base_center.x - wheel_point_fl.x) + wheel_point_fl.y;

                if( sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.)) > sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.)) ){
                    yaw = fl_distance_m / (sqrtf(powf((wheel_point_fl.x - base_center.x), 2.) + powf((wheel_point_fl.y - base_center.y), 2.)));
                } else{
                    yaw = br_distance_m / (sqrtf(powf((wheel_point_br.x - base_center.x), 2.) + powf((wheel_point_br.y - base_center.y), 2.)));
                }
            }

            pose_x = (0. - base_center.x) * cosf(yaw) - (0. - base_center.y) * sinf(yaw) + base_center.x;
            pose_y = (0. - base_center.x) * sinf(yaw) + (0. - base_center.y) * cosf(yaw) + base_center.y;
            
            if ( std::isnan(pose_x) ) pose_x = 0.;
            if ( std::isnan(pose_y) ) pose_y = 0.;
            if ( std::isnan(yaw) )    yaw = 0.;

            calculation_odom.pose.pose.position.x = pose_x * cosf(prev_yaw) - pose_y * sinf(prev_yaw) + prev_odom.pose.pose.position.x;
            calculation_odom.pose.pose.position.y = pose_x * sinf(prev_yaw) + pose_y * cosf(prev_yaw) + prev_odom.pose.pose.position.y;
            calculation_odom.pose.pose.position.z = prev_odom.pose.pose.position.z;

            // Change quaternion (calculation_odom)
            quat_tf.setRPY(0., 0., (prev_yaw + yaw));
            tf2::convert(quat_tf, calculation_odom.pose.pose.orientation);
            // tf2::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., (prev_yaw + yaw));
            // quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

            // Info
            // ROS_INFO("base_center = %.4f, %.4f,  yaw = %.4f", base_center.x, base_center.y, yaw);
            // ROS_INFO("odom = %.4f, %.4f, %.4f\n",calculation_odom.pose.pose.position.x, calculation_odom.pose.pose.position.y, (prev_yaw + yaw));

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
double SobitProOdometry::distance_calculation(double wheel_curt_pos){
    double distance_m;

    distance_m = SobitProControl::WHEEL_LENGTH * wheel_curt_pos / 4096.;


    return distance_m;
}

// Position calculation
double SobitProOdometry::position_calculation(double steer_curt_pos){
    double direction_deg;

    direction_deg = (steer_curt_pos - 2048.) * 360. / 4096.;


    return direction_deg;
}

// Pose broadcaster (Generate a pose from Odometry)
void SobitProOdometry::pose_broadcaster(nav_msgs::Odometry tf_odom){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp            = ros::Time::now();
    transformStamped.header.frame_id         = "odom";
    transformStamped.child_frame_id          = "base_footprint";
    transformStamped.transform.translation.x = tf_odom.pose.pose.position.x;
    transformStamped.transform.translation.y = tf_odom.pose.pose.position.y;
    transformStamped.transform.translation.z = tf_odom.pose.pose.position.z;
    transformStamped.transform.rotation.x    = tf_odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y    = tf_odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z    = tf_odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w    = tf_odom.pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}