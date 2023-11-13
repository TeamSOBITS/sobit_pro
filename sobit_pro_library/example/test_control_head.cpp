#include <ros/ros.h>
#include "sobit_pro_library/sobit_pro_joint_controller.h"


int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_pro_test_control_head");
    
    sobit_pro::SobitProJointController pro_joint_ctrl;

    const double MAX_ANGLE =  1.57;
    const double MIN_ANGLE = -1.57;
    double target_angle    =  0.0;
    double increment       =  0.05;

    while( ros::ok() ){
        target_angle += increment;
        if (target_angle > MAX_ANGLE || target_angle < MIN_ANGLE) increment *= -1.0;

        // Option 1: Move the head joints simultaneously
        pro_joint_ctrl.moveHeadPanTilt( target_angle, target_angle, 0.5, false );
        ros::Duration(0.5).sleep();

        /***
        // Option 2: Move the head joints individually
        pro_joint_ctrl.moveJoint( sobit_pro::Joint::HEAD_PAN_JOINT , target_angle, 0.5, false );
        pro_joint_ctrl.moveJoint( sobit_pro::Joint::HEAD_TILT_JOINT, target_angle, 0.5, false );
        ***/
    }


    return 0;
}
