#include "sobit_pro_control.hpp"
#include "sobit_pro_motor_driver.hpp"
#include "sobit_pro_odometry.hpp"

SobitProControl sobit_pro_control;
SobitProMotorDriver sobit_pro_motor_driver;
SobitProOdometry sobit_pro_odometry;

int motion;
// uint8_t sobit_pro_steer[4] = {STEER_F_R, STEER_F_L, STEER_B_R, STEER_B_L};
// uint8_t sobit_pro_wheel[4] = {WHEEL_F_R, WHEEL_F_L, WHEEL_B_R, WHEEL_B_L};

// Twist callback
void callback(const geometry_msgs::Twist vel_twist){

  // Translational motion
  if(vel_twist.linear.x != 0 || vel_twist.linear.y != 0 && vel_twist.linear.z == 0 && vel_twist.angular.x == 0 && vel_twist.angular.y == 0 && vel_twist.angular.z == 0){
    // printf("Translational motion\n");
    motion = TRANSLATIONAL_MOTION;
  }
  // Rotational motion
  else if(vel_twist.linear.x == 0 && vel_twist.linear.y == 0 && vel_twist.linear.z == 0 && vel_twist.angular.x == 0 && vel_twist.angular.y == 0 && vel_twist.angular.z != 0){
    // printf("Rotational motion\n");
    motion = ROTATIONAL_MOTION;
  }
  // Swivel motion(This motion can not move)
  else if(vel_twist.linear.x != 0 || vel_twist.linear.y != 0 && vel_twist.angular.z != 0){
    printf("Failed to change the motion!\n");
    // Motion can be added
    return;
  }
  // ERROR
  else if(vel_twist.linear.z != 0 || vel_twist.angular.x != 0 || vel_twist.angular.y != 0){
    printf("Failed to change the motion!\n");
    return;
  }
  // Stop motion
  else{
    // printf("Stop motion\n");
    motion = 0;
  }

  sobit_pro_control.getMotion(motion);
  sobit_pro_odometry.getMotion(motion);
  sobit_pro_control.setParams(vel_twist);
}

// main
int main(int argc, char **argv){
  ros::init(argc, argv, "sobit_pro_control");
  ros::NodeHandle nh;
  ros::Subscriber sub_velocity = nh.subscribe("/mobile_base/commands/velocity", 1, callback);
  ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  // ros::Publisher pub_hz = nh.advertise<std_msgs::Empty>("/hz", 1);
  int32_t wheel_fr_initial_position;
  int32_t wheel_fl_initial_position;
  int32_t steer_fr_present_position;
  int32_t set_steer_angle;
  nav_msgs::Odometry result_odom;
  nav_msgs::Odometry old_odom;
  int old_motion = 3; // Non 0, 1, 2 motion

  sobit_pro_motor_driver.init();
  sobit_pro_motor_driver.addPresentParam();

  // Set the initial position of the wheel
  wheel_fr_initial_position = sobit_pro_motor_driver.feedbackWheel(WHEEL_F_R);
  wheel_fl_initial_position = sobit_pro_motor_driver.feedbackWheel(WHEEL_F_L);

  ros::Rate rate(50);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  while(ros::ok()){

    // Write goal position value
    set_steer_angle = *sobit_pro_control.setSteerAngle();
    sobit_pro_motor_driver.controlSteers(sobit_pro_control.setSteerAngle());    

    // Continue until the steering position reaches the goal
    do{
      steer_fr_present_position = sobit_pro_motor_driver.feedbackSteer(STEER_F_R);
    }while(abs(set_steer_angle - steer_fr_present_position) > DXL_MOVING_STATUS_THRESHOLD);

    // Update the initial position of the wheel when the motion changes
    if(motion != old_motion && motion != 0){
      wheel_fr_initial_position = sobit_pro_motor_driver.feedbackWheel(WHEEL_F_R);
      wheel_fl_initial_position = sobit_pro_motor_driver.feedbackWheel(WHEEL_F_L);
      old_odom = result_odom;
    }
    if(motion != 0){
      old_motion = motion;
    }

    // Write goal velocity value
    sobit_pro_motor_driver.controlWheels(sobit_pro_control.setWheelVel());

    // Odometry calculation
    sobit_pro_odometry.odom(sobit_pro_motor_driver.feedbackSteer(STEER_F_R),
                            sobit_pro_motor_driver.feedbackSteer(STEER_F_L),
                            sobit_pro_motor_driver.feedbackWheel(WHEEL_F_R),
                            sobit_pro_motor_driver.feedbackWheel(WHEEL_F_L),
                            wheel_fr_initial_position,
                            wheel_fl_initial_position,
                            old_odom,
                            &result_odom);

    // std::cout << "\n[ Odometry ]" << result_odom << std::endl;

    pub_odometry.publish(nav_msgs::Odometry(result_odom));
    // pub_hz.publish(std_msgs::Empty());

    rate.sleep();
  }

  sobit_pro_motor_driver.closeDynamixel();

  return 0;
}
