#include "sobit_pro_control/sobit_pro_main.hpp"
#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_motor_driver.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Create the instance
SobitProControl     sobit_pro_control;
SobitProMotorDriver sobit_pro_motor_driver;
SobitProOdometry    sobit_pro_odometry;

// Twist callback
void SobitProMain::callback(const geometry_msgs::Twist vel_twist){
    // Not supported motion
    // if ((vel_twist.linear.z != 0.0) || (vel_twist.angular.x != 0.0) || (vel_twist.angular.y != 0.0))
    // {
    //     ROS_WARN("vellocity error... fix the value zero.");
    //     vel_twist.linear.z = 0.0;
    //     vel_twist.angular.x = 0.0;
    //     vel_twist.angular.y = 0.0;
    // }
    // Translational
    if (((std::abs(vel_twist.linear.x) > 0.000) || (std::abs(vel_twist.linear.y) > 0.000)) && (std::abs(vel_twist.angular.z) <= 0.001))
    {
        ROS_INFO("Translational motion.\n");
        motion = TRANSLATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Rotational
    else if (((std::abs(vel_twist.linear.x) <= 0.001) && (std::abs(vel_twist.linear.y) <= 0.001)) && (std::abs(vel_twist.angular.z) > 0.000))
    {
        ROS_INFO("Rotational motion.\n");
        motion = ROTATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Swivel
    else if (((std::abs(vel_twist.linear.x) > 0.000) || (std::abs(vel_twist.linear.y) > 0.000)) && (std::abs(vel_twist.angular.z) > 0.000))
    {
        const double steering_track = 0.32060706;
        // if(fabs(2.0*vel_twist.linear.x) > fabs(vel_twist.angular.z*steering_track) && fabs(vel_twist.linear.y) < 0.001){
        if(fabs(2.0*(sqrtf(powf(vel_twist.linear.x,2.) + powf(vel_twist.linear.y,2.)))) > fabs(vel_twist.angular.z*steering_track)){
            ROS_INFO("Swivel motion.\n");
            motion = SWIVEL_MOTION;

        }
        else{
            ROS_INFO("Swivel motion to Rotational motion.\n");
            motion = ROTATIONAL_MOTION;
        }
        // ROS_INFO("Swivel motion.\n");
        // motion = SWIVEL_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Stop motion
    else{
        ROS_INFO("Stop motion.\n");
        motion = STOP_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // if((std::abs(vel_twist.linear.x > 0.001) || std::abs(vel_twist.linear.y > 0.001)) && std::abs(vel_twist.linear.z <= 0.001) && std::abs(vel_twist.angular.x <= 0.001) && std::abs(vel_twist.angular.y <= 0.001) && std::abs(vel_twist.angular.z <= 0.001)){
    //     ROS_INFO("Translational motion.\n");
    //     motion = TRANSLATIONAL_MOTION;
    //     wheels_error.data = false;
    //     pub_wheels_error.publish(wheels_error);
    // }
    // else if(std::abs(vel_twist.linear.x <= 0.001) && std::abs(vel_twist.linear.y <= 0.001) && std::abs(vel_twist.linear.z <= 0.001) && std::abs(vel_twist.angular.x <= 0.001) && std::abs(vel_twist.angular.y <= 0.001) && std::abs(vel_twist.angular.z > 0.001)){
    //     ROS_INFO("Rotational motion.\n");
    //     motion = ROTATIONAL_MOTION;
    //     wheels_error.data = false;
    //     pub_wheels_error.publish(wheels_error);
    // }
    // else if((std::abs(vel_twist.linear.x > 0.001) || std::abs(vel_twist.linear.y > 0.001)) && std::abs(vel_twist.angular.z > 0.001)){
    //     if(std::abs(vel_twist.linear.y) > 0.001){
    //         ROS_ERROR("Motion (rotation in y axis) is not supported yet!\n");
    //         wheels_error.data = true;
    //         pub_wheels_error.publish(wheels_error);
    //         return;
    //     }
    //     // Motion can be added

    //     const double steering_track = 0.32060706;
    //     if(fabs(2.0*vel_twist.linear.x) > fabs(vel_twist.angular.z*steering_track) && fabs(vel_twist.linear.y) < 0.001){
    //         ROS_INFO("Swivel motion.\n");
    //         motion = SWIVEL_MOTION;

    //     }
    //     else{
    //         ROS_INFO("Swivel motion to Rotational motion");
    //         motion = ROTATIONAL_MOTION;
    //     }
        
    //     // motion = SWIVEL_MOTION;
        
    //     wheels_error.data = false;
    //     pub_wheels_error.publish(wheels_error);
    // }
    // // Not yet supported motion
    // else if(vel_twist.linear.z != 0 || vel_twist.angular.x != 0 || vel_twist.angular.y != 0){
    //     ROS_ERROR("Failed to change the motion!\n");
    //     ROS_ERROR("This motion is not supported.\n");
    //     wheels_error.data = true;
    //     pub_wheels_error.publish(wheels_error);
    //     return;
    // }
    // // Stop motion
    // else{
    //     ROS_INFO("Stop motion.\n");
    //     motion = 0;
    //     wheels_error.data = false;
    //     pub_wheels_error.publish(wheels_error);
    // }

    sobit_pro_control.getMotion(motion);
    sobit_pro_odometry.getMotion(motion);
    sobit_pro_control.setParams(vel_twist);

    return;
}

// Start up sound
void SobitProMain::start_up_sound(){
    std::random_device rnd;
    std::mt19937 mt(rnd()); 
    std::uniform_real_distribution<> rand10(1, 100);
    int rand_sound  = rand10(mt);
    int sound_param = 95;
    int system_ret  = -1;

    pnh.getParam("sound_param", sound_param);

    std::cout << "\nsound_param :" << sound_param << std::endl;

    if(rand_sound <= sound_param){
        std::cout << "\nStart Up: Regular Sound" << std::endl;
        system_ret = system("mpg321 ~/catkin_ws/src/sobit_pro/sobit_pro_control/mp3/start_up.mp3");
        ros::Duration(2.).sleep();
    }
    else{
        std::cout << "\nStart Up: Soka Univ. Song" << std::endl;
        system_ret = system("mpg321 ~/catkin_ws/src/sobit_pro/sobit_pro_control/mp3/soka_univ_gakuseika.mp3");
        ros::Duration(2.).sleep();
    }

    if(system_ret == -1) ROS_ERROR("There was an error reproducing the start up sound.");

    return;
}

// Shut down sound
void SobitProMain::shut_down_sound(){
    int system_ret = -1;

    std::cout << "\nShut Down" << std::endl;
    system_ret = system("mpg321 ~/catkin_ws/src/sobit_pro/sobit_pro_control/mp3/shut_down.mp3");
    ros::Duration(2.).sleep();

    if(system_ret == -1) ROS_ERROR("There was an error reproducing the start up sound.");
    
    return;
}

// Control wheel
void SobitProMain::control_wheel(){
    // Set the initial position of the wheel
    wheel_fr_init_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_F_R);
    wheel_fl_init_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_F_L);
    wheel_br_init_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_B_L);
    wheel_bl_init_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_B_L);

    // Set the initial Odometry
    prev_odom.pose.pose.orientation.w = 1;

    ros::Rate rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    prev_time = ros::Time::now();

    while(ros::ok()){
        // Write goal position value
        set_steer_angle = sobit_pro_control.setSteerAngle();

        steer_fr_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_R);
        steer_fl_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_L);
        steer_br_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_B_R);
        steer_bl_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_B_L);

        if( (1024 <= std::abs(set_steer_angle[0] - steer_fr_curt_position)) || (1024 <= std::abs(set_steer_angle[1] - steer_fl_curt_position)) || (1024 <= std::abs(set_steer_angle[2] - steer_br_curt_position)) || (1024 <= std::abs(set_steer_angle[3] - steer_bl_curt_position)) ){
            ROS_INFO("Changing the direction of the wheel");
            set_wheel_vel[0] = set_wheel_vel[1] = set_wheel_vel[2] = set_wheel_vel[3] = 0.;
            sobit_pro_motor_driver.controlWheels(set_wheel_vel);
            ros::Duration(0.5).sleep();
        }

        // while ( (1024 <= std::abs(set_steer_angle[0] - steer_fr_curt_position)) || (1024 <= std::abs(set_steer_angle[1] - steer_fl_curt_position)) ) {
        //   ROS_INFO("Change the direction of the wheel");
        //   sobit_pro_motor_driver.controlWheels(set_wheel_vel);
        //   ros::Duration(0.1).sleep();
        //   // Write goal position value
        //   set_steer_angle = sobit_pro_control.setSteerAngle();
        //   steer_fr_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_R);
        //   steer_fl_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_L);
        // }

        sobit_pro_motor_driver.controlSteers(set_steer_angle);

        // Continue until the steering position reaches the goal
        do{
            steer_fr_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_R);
            steer_fl_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_L);
            steer_br_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_B_R);
            steer_bl_curt_position = sobit_pro_motor_driver.feedbackSteer(STEER_B_L);
        }while( (DXL_MOVING_STATUS_THRESHOLD < std::abs(set_steer_angle[0] - steer_fr_curt_position)) && (DXL_MOVING_STATUS_THRESHOLD < std::abs(set_steer_angle[1] - steer_fl_curt_position)) && (DXL_MOVING_STATUS_THRESHOLD < std::abs(set_steer_angle[2] - steer_fl_curt_position)) && (DXL_MOVING_STATUS_THRESHOLD < std::abs(set_steer_angle[3] - steer_fl_curt_position)) );

        // Write goal velocity value
        set_wheel_vel = sobit_pro_control.setWheelVel();
        sobit_pro_motor_driver.controlWheels(set_wheel_vel);

        // Publish JointState
        joint_state.header.stamp = ros::Time::now();

        joint_state.name.resize(8);
        joint_state.name[0] = "wheel_f_r_steer_joint";
        joint_state.name[1] = "wheel_f_l_steer_joint";
        joint_state.name[2] = "wheel_b_r_steer_joint";
        joint_state.name[3] = "wheel_b_l_steer_joint";

        joint_state.name[4] = "wheel_f_r_drive_joint";
        joint_state.name[5] = "wheel_f_l_drive_joint";
        joint_state.name[6] = "wheel_b_r_drive_joint";
        joint_state.name[7] = "wheel_b_l_drive_joint";

        joint_state.position.resize(8);
        joint_state.position[0] = (set_steer_angle[0] - 2048.) * (1.57 / 1024.); // Convert 2048. to 1.57
        joint_state.position[1] = (set_steer_angle[1] - 2048.) * (1.57 / 1024.); // Convert 2048. to 1.57
        joint_state.position[2] = (set_steer_angle[2] - 2048.) * (1.57 / 1024.); // Convert 2048. to 1.57
        joint_state.position[3] = (set_steer_angle[3] - 2048.) * (1.57 / 1024.); // Convert 2048. to 1.57
        joint_state.position[4] = set_wheel_vel[0] * VEL_UNIT * (M_PI*WHEEL_DIAMETER) / 60.; // Convert RPM to m/s
        joint_state.position[5] = set_wheel_vel[1] * VEL_UNIT * (M_PI*WHEEL_DIAMETER) / 60.; // Convert RPM to m/s
        joint_state.position[6] = set_wheel_vel[2] * VEL_UNIT * (M_PI*WHEEL_DIAMETER) / 60.; // Convert RPM to m/s
        joint_state.position[7] = set_wheel_vel[3] * VEL_UNIT * (M_PI*WHEEL_DIAMETER) / 60.; // Convert RPM to m/s

        pub_joint_states.publish(sensor_msgs::JointState(joint_state));
        //std::cout << "\n[ joint_state ]\n" << joint_state << std::endl;

        // Set the present position of the wheel
        wheel_fr_curt_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_F_R);
        wheel_fl_curt_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_F_L);
        wheel_br_curt_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_B_R);
        wheel_bl_curt_position = sobit_pro_motor_driver.feedbackWheelPos(WHEEL_B_L);

        // Odometry calculation
        sobit_pro_odometry.odom(sobit_pro_motor_driver.feedbackSteer(STEER_F_R), sobit_pro_motor_driver.feedbackSteer(STEER_F_L),
                                sobit_pro_motor_driver.feedbackSteer(STEER_B_R), sobit_pro_motor_driver.feedbackSteer(STEER_B_L),
                                sobit_pro_motor_driver.feedbackWheelVel(WHEEL_F_R), sobit_pro_motor_driver.feedbackWheelVel(STEER_F_L),
                                sobit_pro_motor_driver.feedbackWheelVel(STEER_B_R), sobit_pro_motor_driver.feedbackWheelVel(STEER_B_L),
                                wheel_fr_curt_position, wheel_fl_curt_position,
                                wheel_br_curt_position, wheel_bl_curt_position,
                                wheel_fr_init_position, wheel_fl_init_position,
                                wheel_br_init_position, wheel_bl_init_position,
                                prev_motion,
                                prev_odom, &result_odom,
                                prev_time);

        // Update the initial position of the wheel
        wheel_fr_init_position = wheel_fr_curt_position;
        wheel_fl_init_position = wheel_fl_curt_position;
        wheel_br_init_position = wheel_br_curt_position;
        wheel_bl_init_position = wheel_bl_curt_position;

        // Update
        prev_odom = result_odom;
        prev_time = ros::Time::now();

        // Publish Odometry
        sobit_pro_odometry.pose_broadcaster(result_odom);
        pub_odometry.publish(result_odom);
        // pub_odometry.publish(nav_msgs::Odometry(result_odom));

        // std::cout << "\n[ Odometry ]\n" << result_odom << std::endl;
        // std::cout << "\n[ Odometry position ]\n" << result_odom.pose.pose.position << std::endl;
        // std::cout << "\n[ Odometry orientation ]\n" << result_odom.pose.pose.orientation << std::endl;
        // pub_hz.publish(std_msgs::Empty());

        rate.sleep();
    }

    spinner.stop();
}

// main
int main(int argc, char **argv){
    ros::init(argc, argv, "sobit_pro_control");

    // Create SobitProMain instance
    SobitProMain sobit_pro_main;

    // Start up motor
    sobit_pro_motor_driver.init();
    sobit_pro_motor_driver.addPresentParam();

    // Start up sound
    sobit_pro_main.start_up_sound();

    // Control wheel
    sobit_pro_main.control_wheel();

    // Shut down sound
    sobit_pro_main.shut_down_sound();

    // Shut down motor
    sobit_pro_motor_driver.closeDynamixel();

    return 0;
}
