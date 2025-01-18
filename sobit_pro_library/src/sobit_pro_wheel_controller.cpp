#include "sobit_pro_library/sobit_pro_wheel_controller.hpp"

using namespace sobit_pro;

SobitProWheelController::SobitProWheelController ( const std::string& name ) : ROSCommonNode( name ), nh_(), pnh_("~"){
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "mobile_base/commands/velocity", 1 );
    sub_odom_ = nh_.subscribe( "odom", 1, &SobitProWheelController::callbackOdometry, this );

    ros::spinOnce();
    ros::Duration(3.0).sleep();
}

SobitProWheelController::SobitProWheelController () : ROSCommonNode( ), nh_(), pnh_("~"){ 
    sub_odom_ = nh_.subscribe( "odom", 1, &SobitProWheelController::callbackOdometry, this );
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "mobile_base/commands/velocity", 1 );

    ros::spinOnce();
    ros::Duration(3.0).sleep();
}

bool SobitProWheelController::controlWheelLinear( const double distance_x, const double distance_y ){
    try{
        // wait for odom
        do{
            ros::spinOnce();    
        } while(curt_odom_.pose.pose.orientation.x == 0 &&
                curt_odom_.pose.pose.orientation.y == 0 &&
                curt_odom_.pose.pose.orientation.z == 0 &&
                curt_odom_.pose.pose.orientation.w == 0);

        geometry_msgs::Twist output_vel, initial_vel;
        nav_msgs::Odometry init_odom = curt_odom_;

        double curt_distance   = 0.;
        double target_distance = hypotf( distance_x, distance_y );
        double Distance_x      = fabsf(distance_x);
        double Distance_y      = fabsf(distance_y);
        
        // TODO: get PID parameters from launch file
        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;

        double vel_differential = Kp * target_distance;

        double init_time = ros::Time::now().toSec();
        ros::Rate loop_rate(20);

        while( curt_distance < target_distance ){
            ros::spinOnce();

            double curt_time    = ros::Time::now().toSec();
            double elapsed_time = curt_time - init_time;

            double vel_linear = 0.0;

            // TODO: fix overshoot (change actuator with higher torque or improve PID controller)
            if( (target_distance < 0.01) || (target_distance-curt_distance < 0.01) )  break;
            if( (target_distance < 0.60) && (fabsf(vel_differential) > MAX_VEL_DIF) ) vel_differential = copysignf(MAX_VEL_DIF, vel_differential);
            if( target_distance <= 0.1 ) vel_linear = Kp * ( target_distance + 0.001 - curt_distance )
                                                    - Kd * vel_differential
                                                    + Ki / 0.8 * ( target_distance + 0.001 - curt_distance ) * powf( elapsed_time, 2. );
            else                         vel_linear =  Kp * ( target_distance + 0.001 - curt_distance )
                                                    - Kd * vel_differential
                                                    + Ki / ( 8.0 / target_distance ) * ( target_distance + 0.001 - curt_distance ) * powf( elapsed_time, 2. );

            output_vel.linear.x = ( distance_x > 0 ) ? vel_linear * ( Distance_x / ( Distance_x + Distance_y ) ) : -vel_linear * ( Distance_x / ( Distance_x + Distance_y ) );
            output_vel.linear.y = ( distance_y > 0 ) ? vel_linear * ( Distance_y / ( Distance_x + Distance_y ) ) : -vel_linear * ( Distance_y / ( Distance_x + Distance_y ) );
            pub_cmd_vel_.publish( output_vel );

            vel_differential = vel_linear;

            double x_dif = curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x;
            double y_dif = curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y;
            curt_distance = hypotf( x_dif, y_dif );
            // ROS_INFO("target_distance = %f\tcurt_distance = %f", target_distance, curt_distance );

            loop_rate.sleep();
        }

        pub_cmd_vel_.publish( initial_vel );
        ros::Duration(0.5).sleep();

        return true;
    } catch( const std::exception& ex ){
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProWheelController::controlWheelRotateRad( const double angle_rad ){
    try {
        // wait for odom
        do{
            ros::spinOnce();    
        } while(curt_odom_.pose.pose.orientation.x == 0 &&
                curt_odom_.pose.pose.orientation.y == 0 &&
                curt_odom_.pose.pose.orientation.z == 0 &&
                curt_odom_.pose.pose.orientation.w == 0);

        geometry_msgs::Twist output_vel, initial_vel;

        int loop_cnt = 1;

        double init_yaw         = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
        double curt_angle_rad   = 0.0;
        double target_angle_rad = fabsf( angle_rad );
        double target_angle_deg = rad2Deg( target_angle_rad );

        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;

        double vel_differential = Kp * angle_rad;

        double init_time = ros::Time::now().toSec();
        ros::Rate loop_rate(20);

        while( curt_angle_rad < target_angle_rad ){
            ros::spinOnce();

            double curt_time    = ros::Time::now().toSec();
            double elapsed_time = curt_time - init_time;

            double vel_angular = 0.0;

            // PID
            if( target_angle_deg <= 30 ) vel_angular = Kp * ( target_angle_rad + 0.001 - curt_angle_rad )
                                                     - Kd * vel_differential
                                                     + Ki * ( target_angle_rad + 0.001 - curt_angle_rad ) * powf( elapsed_time, 2. );
            else                         vel_angular = Kp * ( target_angle_rad + 0.001 - curt_angle_rad )
                                                     - Kd * vel_differential
                                                     + Ki * ( target_angle_rad + 0.001 - curt_angle_rad ) * powf( elapsed_time, 2. ) * 0.75 * 30 / target_angle_deg;
            
            output_vel.angular.z = ( angle_rad > 0 ) ? vel_angular : - vel_angular;
            pub_cmd_vel_.publish( output_vel );

            vel_differential = vel_angular;

            double curt_yaw     = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
            double prev_ang_rad = curt_angle_rad;

            if     ( (-0.00314 < curt_yaw - init_yaw) && (curt_yaw - init_yaw < 0) && (0 < angle_rad) ) continue;
            else if( (0 < curt_yaw - init_yaw) && (curt_yaw - init_yaw < 0.00314) && (angle_rad < 0) )  continue;

            if     ( (curt_yaw - init_yaw < 0) && (0 < angle_rad) ) curt_angle_rad = fabsf(curt_yaw - init_yaw + deg2Rad(360. * loop_cnt));
            else if( (0 < curt_yaw - init_yaw) && (angle_rad < 0) ) curt_angle_rad = fabsf(curt_yaw - init_yaw - deg2Rad(360. * loop_cnt));
            else if( 0 < angle_rad )                                curt_angle_rad = fabsf(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
            else                                                    curt_angle_rad = fabsf(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));

            if( rad2Deg(curt_angle_rad) < (rad2Deg(prev_ang_rad)-0.0314) ){
                loop_cnt++;

                if( 0 < angle_rad ) curt_angle_rad = fabsf(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
                else                curt_angle_rad = fabsf(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));
            }

            // ROS_INFO("target_angle_rad = %f\tcurt_angle_rad = %f", target_angle_rad, curt_angle_rad );

            loop_rate.sleep();
        }

        pub_cmd_vel_.publish( initial_vel );
        ros::Duration(0.5).sleep();

        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitProWheelController::controlWheelRotateDeg( const double angle_deg ){
    return controlWheelRotateRad( deg2Rad(angle_deg) );
}
