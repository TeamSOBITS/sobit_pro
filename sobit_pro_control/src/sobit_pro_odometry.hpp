#ifndef SOBIT_PRO_ODOMETRY_H_
#define SOBIT_PRO_ODOMETRY_H_

#include <math.h>
#include <nav_msgs/Odometry.h>

#define WHEEL_LENGTH                    0.439823 //Circumference
#define PAI                             acos(-1.0)

class SobitProOdometry{
  public:
    nav_msgs::Odometry odom(float steer_fr_present_angle,
                            float steer_fl_present_angle,
                            float wheel_fr_present_vel,
                            float wheel_fl_present_vel);
    float velocity_calculation(float wheel_present_vel);
    float position_calculation(float steer_present_angle);

  private:

};

#endif // SOBIT_PRO_ODOMETRY_H_
