#include <ros/ros.h>
#include "sobit_pro_library/sobit_pro_joint_controller.h"


int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_pro_test_control_arm");
    
    sobit_pro::SobitProJointController pro_joint_ctrl;

    // Move all the arm joints
    pro_joint_ctrl.moveArm( 1.0, 1.0, -1.0, 0.0, -1.0, 3.0, true );

    // Open the hand
    pro_joint_ctrl.moveJoint( sobit_pro::Joint::HAND_JOINT, -1.57, 5.0, true );

    // Set the initial pose
    pro_joint_ctrl.moveToPose( "initial_pose", 5.0, true );

    return 0;
}
