#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

using namespace sobit_pro;

SobitProWheelController::SobitProWheelController ( const std::string &name ) : ROSCommonNode( name ), nh_(), pnh_("~") {
    sub_odom_ = nh_.subscribe( "/odom", 1, &SobitProWheelController::callbackOdometry, this );
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity", 1 );
    ros::spinOnce();
    ros::Duration(3.0).sleep();
}

SobitProWheelController::SobitProWheelController ( ) : ROSCommonNode( ), nh_(), pnh_("~") { 
    sub_odom_ = nh_.subscribe( "/odom", 1, &SobitProWheelController::callbackOdometry, this );
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity", 1 );
    ros::spinOnce();
    ros::Duration(3.0).sleep();
}

bool SobitProWheelController::controlWheelLinear( const double dist_x, const double dist_y ) {
    try {
        // ros::spinOnce();
        // double start_time = ros::Time::now().toSec();
        // geometry_msgs::Twist output_vel, init_vel;
        while ( curt_odom_.pose.pose.orientation.x == 0 &
                curt_odom_.pose.pose.orientation.y == 0 &
                curt_odom_.pose.pose.orientation.z == 0 &
                curt_odom_.pose.pose.orientation.w == 0 ) {

            ros::spinOnce();    
        }

        nav_msgs::Odometry init_odom = curt_odom_;
        geometry_msgs::Twist output_vel, init_vel;

        // Assign Linear and Angular value
        init_vel.linear.x  = init_vel.linear.y  = init_vel.linear.z  = 0.0;
        init_vel.angular.x = init_vel.angular.y = init_vel.angular.z = 0.0;
        output_vel.linear.x   = output_vel.linear.y   = output_vel.linear.z   = 0.0;
        output_vel.angular.x  = output_vel.angular.y  = output_vel.angular.z  = 0.0;

        double curt_dist = 0.0;
        double goal_dist = std::hypotf( dist_x, dist_y );

        // PID params
        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;
        double vel_differential = Kp * goal_dist;

        ros::Rate loop_rate( 20 );

        // Get the absolute value of the distance
        double dist_x_abs = std::abs( dist_x );
        double dist_y_abs = std::abs( dist_y );

        double vel_linear_max = 0.075; // This the is the max. vel. that SOBIT PRO can output 
        double vel_linear = 0.0;

        double start_time = ros::Time::now().toSec();

        while ( curt_dist < goal_dist  ) {
            ros::spinOnce();

            double curt_time    = ros::Time::now().toSec();
            double elapsed_time = curt_time - start_time;
            
            // TODO: fix avoid overshoot (change actuator with higher torque or improve PID controller)
            if ( goal_dist < 0.01 || goal_dist-curt_dist < 0.01 ){ break; }
            // if ( goal_dist < 0.60 && std::abs(vel_differential) > vel_linear_max) { vel_differential = std::copysign(vel_linear_max, vel_differential); }
            if ( goal_dist <= 0.1 ) {
                ROS_INFO("Distance is under 0.1[m]: %f", goal_dist);
                // vel_linear =  Kp * ( goal_dist + 0.001 - curt_dist ) - Kd * vel_differential + Ki / 0.8 * ( goal_dist + 0.001 - curt_dist ) * std::pow( elapsed_time, 2 );
                vel_linear =  Kp * ( goal_dist - curt_dist ) - Kd * vel_differential + Ki / 0.8 * ( goal_dist - curt_dist ) * std::pow( elapsed_time, 2 );
            } else {
                ROS_INFO("Distance is over 0.1[m]: %f", goal_dist);
                // vel_linear =  Kp * ( goal_dist + 0.001 - curt_dist ) - Kd * vel_differential + Ki / ( 8.0 / goal_dist ) * ( goal_dist + 0.001 - curt_dist ) * std::pow( elapsed_time, 2 );
                vel_linear =  Kp * ( goal_dist - curt_dist ) - Kd * vel_differential + Ki / ( 8.0 / goal_dist ) * ( goal_dist - curt_dist ) * std::pow( elapsed_time, 2 );
            }

            // Select orientation based on the goal
            output_vel.linear.x = ( dist_x > 0.0 ) ? vel_linear * ( dist_x_abs / ( dist_x_abs + dist_y_abs ) ) : -vel_linear * ( dist_x_abs / ( dist_x_abs + dist_y_abs ) );
            output_vel.linear.y = ( dist_y > 0.0 ) ? vel_linear * ( dist_y_abs / ( dist_x_abs + dist_y_abs ) ) : -vel_linear * ( dist_y_abs / ( dist_x_abs + dist_y_abs ) );

            // Clamp output limits
            // output_vel.linear.x = ( dist_x > 0.0 ) ? std::min(output_vel.linear.x, vel_linear_max) : std::max(output_vel.linear.x, -vel_linear_max) ;
            // output_vel.linear.y = ( dist_y > 0.0 ) ? std::min(output_vel.linear.y, vel_linear_max) : std::max(output_vel.linear.y, -vel_linear_max) ;

            // Limit output based on a 'sigmoidal saturation function'
            output_vel.linear.x = vel_linear_max * (2.0 / (1.0 + exp(-output_vel.linear.x)) - 1.0);
            output_vel.linear.y = vel_linear_max * (2.0 / (1.0 + exp(-output_vel.linear.y)) - 1.0);

            // Publish final velocity
            pub_cmd_vel_.publish( output_vel );

            // Update current distance
            double curt_dist_x = curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x;
            double curt_dist_y = curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y;
            curt_dist = std::hypotf( curt_dist_x, curt_dist_y );
            vel_differential = vel_linear;

            // Debug log
            ROS_INFO("vel_linear = %f", vel_linear );
            ROS_INFO("output_vel.linear.x = %f", output_vel.linear.x );
            ROS_INFO("output_vel.linear.y = %f", output_vel.linear.y );
            ROS_INFO("curt_dist = %f/%f", curt_dist, goal_dist );
            // ROS_INFO("goal_dist = %f\tcurt_dist = %f", goal_dist, curt_dist );

            loop_rate.sleep();
        }

        pub_cmd_vel_.publish( init_vel );
        ros::Duration(0.5).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );

        return false;
    }
}

bool SobitProWheelController::controlWheelRotateRad( const double angle_rad ) {
    try {
        while ( curt_odom_.pose.pose.orientation.x == 0 &
                curt_odom_.pose.pose.orientation.y == 0 &
                curt_odom_.pose.pose.orientation.z == 0 &
                curt_odom_.pose.pose.orientation.w == 0 ) {

            ros::spinOnce();    
        }

        int loop_cnt = 1;
        geometry_msgs::Twist output_vel, init_vel;

        // Assign Linear and Angular value
        init_vel.linear.x  = init_vel.linear.y  = init_vel.linear.z  = 0.0;
        init_vel.angular.x = init_vel.angular.y = init_vel.angular.z = 0.0;
        output_vel.linear.x   = output_vel.linear.y   = output_vel.linear.z   = 0.0;
        output_vel.angular.x  = output_vel.angular.y  = output_vel.angular.z  = 0.0;

        double init_yaw = geometryQuat2Yaw ( curt_odom_.pose.pose.orientation );
        double curt_angle_rad = 0.0;
        double curt_angle_deg = rad2Deg( curt_angle_rad );
        double goal_angle_rad = std::fabs( angle_rad );
        double goal_angle_deg = rad2Deg ( goal_angle_rad );

        // PID params
        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;
        double vel_differential = Kp * angle_rad;

        ros::Rate loop_rate(20);

        double vel_angular_max = 0.075; // This the is the max. vel. that SOBIT PRO can output 
        double vel_angular = 0.0;

        double start_time = ros::Time::now().toSec();

        while ( curt_angle_rad < goal_angle_rad ) {
            ros::spinOnce();

            double curt_time    = ros::Time::now().toSec();
            double elapsed_time = curt_time - start_time;

            // if ( goal_angle_deg < 1.0 || goal_angle_deg-curt_angle_deg < 1.0 ){ break; }
            if ( goal_angle_deg <= 30.0 ) {
                vel_angular = Kp * ( goal_angle_rad - curt_angle_rad )
                            - Kd * vel_differential
                            + Ki * ( goal_angle_rad - curt_angle_rad ) * std::pow( elapsed_time, 2 );
            } else {
                vel_angular = Kp * ( goal_angle_rad - curt_angle_rad )
                            - Kd * vel_differential
                            + Ki * ( goal_angle_rad - curt_angle_rad ) * std::pow( elapsed_time, 2 ) * 0.75 * 30.0 / goal_angle_deg;
            }

            // Select orientation based on the goal
            output_vel.angular.z = ( angle_rad > 0.0 ) ? vel_angular : - vel_angular;

            // Clamp output limits
            output_vel.angular.z = ( angle_rad > 0.0 ) ? std::min(output_vel.angular.z, vel_angular_max) : std::max(output_vel.angular.z, -vel_angular_max) ;

            // Limit output based on a 'sigmoidal saturation function'
            // output_vel.angular.z = vel_angular_max * (2.0 / (1.0 + exp(-output_vel.angular.z)) - 1.0);

            // Publish final velocity
            pub_cmd_vel_.publish( output_vel );

            // Update variables
            double curt_yaw = geometryQuat2Yaw ( curt_odom_.pose.pose.orientation );
            double pre_ang_rad = curt_angle_rad;
            
            if      ( -0.00314<(curt_yaw-init_yaw) && (curt_yaw-init_yaw)<0.0 && 0.0<angle_rad ) continue;
            else if ( 0.0<(curt_yaw-init_yaw) && (curt_yaw-init_yaw)<0.00314 && angle_rad<0.0 )  continue;

            if      ( (curt_yaw-init_yaw)<0.0 && 0.0<angle_rad ) curt_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * loop_cnt));
            else if ( 0.0<(curt_yaw-init_yaw) && angle_rad<0.0 ) curt_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * loop_cnt));
            else if ( 0.0<angle_rad )                            curt_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
            else                                                 curt_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));

            if ( rad2Deg(curt_angle_rad) < (rad2Deg(pre_ang_rad)-0.0314) ) {
                loop_cnt++;
                if ( 0.0<angle_rad ) curt_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
                else                 curt_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));
            }

            curt_angle_deg = rad2Deg( curt_angle_rad );
            vel_differential = vel_angular;

            // Debug log
            ROS_INFO("vel_angular = %f", vel_angular );
            ROS_INFO("output_vel.angular.z = %f", output_vel.angular.z );
            ROS_INFO("curt_dist = %f/%f", curt_angle_deg, goal_angle_deg );
            // ROS_INFO("angle_deg = %f/%f", curt_angle_deg, goal_angle_deg );

            loop_rate.sleep();
        }

        pub_cmd_vel_.publish( init_vel );
        ros::Duration(0.5).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );

        return false;
    }
}

bool SobitProWheelController::controlWheelRotateDeg( const double angle_deg ) {
    return controlWheelRotateRad( deg2Rad(angle_deg) );
}