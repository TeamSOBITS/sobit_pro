#ifndef SOBIT_PRO_ODOMETRY_H_
#define SOBIT_PRO_ODOMETRY_H_

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h> // printf etc.

// Define motion key value
#define TRANSLATIONAL_MOTION            1
#define ROTATIONAL_MOTION               2 // Motion can be added

#define WHEEL_LENGTH                    0.144 //0.439823 //Circumference
#define SOBIT_CAR_DIAMETER              0.448051
#define PAI                             acos(-1.0)

class SobitProOdometry{
  private:
    enum MODE{
      NONE = 0,
      TRANSLATIONAL_MOTION_MODE, ROTATIONAL_MOTION_MODE // Motion can be added
    } motion_mode;

  public:
    bool odom(float steer_fr_present_angle,
              float steer_fl_present_angle,
              float wheel_fr_present_vel,
              float wheel_fl_present_vel,
              nav_msgs::Odometry* output_odom);
    float translational_velocity_calculation(float wheel_present_vel);
    float translational_position_calculation(float steer_present_angle);
    float rotational_velocity_calculation(float wheel_present_vel);
    float rotational_position_calculation(float steer_present_angle);

    MODE getMotion(int motion){
      switch (motion){
        case (TRANSLATIONAL_MOTION): motion_mode = TRANSLATIONAL_MOTION_MODE; break;
        case (ROTATIONAL_MOTION): motion_mode = ROTATIONAL_MOTION_MODE; break; // Motion can be added

        default: motion_mode = NONE; break;
      }
      return motion_mode;
    }

};

#endif // SOBIT_PRO_ODOMETRY_H_
