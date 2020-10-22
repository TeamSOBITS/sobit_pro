#include "sobit_pro_control.hpp"
#include "sobit_pro_motor_driver.hpp"
#include "sobit_pro_odometry.hpp"

SobitProControl sobit_pro_control;
SobitProMotorDriver sobit_pro_motor_driver;
SobitProOdometry sobit_pro_odometry;

uint8_t sobit_pro_steer[4] = {STEER_F_R, STEER_F_L, STEER_B_R, STEER_B_L};
uint8_t sobit_pro_wheel[4] = {WHEEL_F_R, WHEEL_F_L, WHEEL_B_R, WHEEL_B_L};

void callback(const geometry_msgs::Twist vel_twist){
  int motion;

  // Translational motion
  if(vel_twist.linear.x != 0 || vel_twist.linear.y != 0 && vel_twist.linear.z == 0 && vel_twist.angular.x == 0 && vel_twist.angular.y == 0 && vel_twist.angular.z == 0){
    printf("Translational motion\n");
    motion = TRANSLATIONAL_MOTION;
  }
  // Rotational motion
  else if(vel_twist.linear.x == 0 && vel_twist.linear.y == 0 && vel_twist.linear.z == 0 && vel_twist.angular.x == 0 && vel_twist.angular.y == 0 && vel_twist.angular.z != 0){
    printf("Rotational motion\n");
    motion = ROTATIONAL_MOTION;
  }
  // Swivel motion(This motion can not move)
  else if(vel_twist.linear.x != 0 || vel_twist.linear.y != 0 && vel_twist.angular.z != 0){
    printf("Failed to change the motion!\n");
    //Motion can be added
    return;
  }
  // ERROR
  else if(vel_twist.linear.z != 0 || vel_twist.angular.x != 0 || vel_twist.angular.y != 0){
    printf("Failed to change the motion!\n");
    return;
  }
  // Stop motion
  else{
    printf("Stop motion\n");
    motion = 0;
  }

  sobit_pro_control.getMotion(motion);
  sobit_pro_odometry.getMotion(motion);
  sobit_pro_control.setParams(vel_twist);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "sobit_pro_control");
  ros::NodeHandle nh;
  ros::Subscriber sub_velocity = nh.subscribe("/mobile_base/commands/velocity", 1, callback);
  ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  //ros::Publisher pub_hz = nh.advertise<std_msgs::Empty>("/hz", 1);
  int32_t steer_fr_present_angle;
  nav_msgs::Odometry result_odom;

  sobit_pro_motor_driver.init();
  sobit_pro_motor_driver.addPresentParam();

  ros::Rate rate(50);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){
    // Write goal position value
    sobit_pro_motor_driver.controlSteers(sobit_pro_control.setSteerAngle());

    // Continue until the steering position reaches the goal
    do{
      steer_fr_present_angle = sobit_pro_motor_driver.feedbackSteer(STEER_F_R);
    }while(abs(*sobit_pro_control.setSteerAngle() - steer_fr_present_angle) > DXL_MOVING_STATUS_THRESHOLD);

    // Write goal velocity value
    sobit_pro_motor_driver.controlWheels(sobit_pro_control.setWheelVel());

    // Odometry calculation
    sobit_pro_odometry.odom(sobit_pro_motor_driver.feedbackSteer(STEER_F_R),
                            sobit_pro_motor_driver.feedbackSteer(STEER_F_L),
                            sobit_pro_motor_driver.feedbackWheel(WHEEL_F_R),
                            sobit_pro_motor_driver.feedbackWheel(WHEEL_F_L),
                            &result_odom);

    // std::cout << "\n[ Odometry ]" << result_odom << std::endl;

    // printf("position.x : %f\n", result_odom.pose.pose.position.x);
    // printf("position.y : %f\n", result_odom.pose.pose.position.y);

    pub_odometry.publish(nav_msgs::Odometry(result_odom));
    //pub_hz.publish(std_msgs::Empty());

    rate.sleep();
  }

  sobit_pro_motor_driver.closeDynamixel();

  return 0;
}
