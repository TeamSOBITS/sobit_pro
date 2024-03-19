#include <ros/ros.h>
#include "sobit_pro_library/sobit_pro_joint_controller.h"


int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_pro_test_grasp_on_floor");

    sobit_pro::SobitProJointController pro_joint_ctrl;

    std::string target_name = "potato_chips";
    bool is_done = false;

    // Set the detecting_pose
    pro_joint_ctrl.moveToPose("detecting_pose", 5.0, true);

    // Open the hand
    pro_joint_ctrl.moveJoint( sobit_pro::Joint::HAND_JOINT, -1.57, 5.0, true );

    // Option 1: Grasp the target on the given TF position
    // Move the hand to the target position
    is_done = (pro_joint_ctrl.moveHandToTargetTF( target_name, -0.15,0.0,0.05 ))
    // Close the hand
           && (pro_joint_ctrl.moveJoint( sobit_pro::Joint::HAND_JOINT, 0.0, 5.0, true ))
    // Check if grasped based on the force sensor
           && (pro_joint_ctrl.graspDecision( 300, 1000 ));

    /***
    // Option 2: Grasp the target on the given coordinates (x,y,z) position
    // Check if grasped based on the force sensor
    is_done = (pro_joint_ctrl.moveHandToTargetCoord( 0.0,0.0,0.0, -0.15,0.0,0.05 ))
    // Close the hand
           && (pro_joint_ctrl.moveJoint( sobit_pro::Joint::HAND_JOINT, 0.0, 5.0, true ))
    // Check if grasped based on the force sensor
           && (pro_joint_ctrl.graspDecision( 300, 1000 ));
    ***/

    std::cout << "Is "<< target_name << " grasped? " << is_done << std::endl;

    if( is_done ){
        // Set the put_high_pose pose to avoid collision
        pro_joint_ctrl.moveToPose("grasp_high_pose", 5.0, true);
    } else {
        ROS_ERROR("Failed to grasp the object");
    }

    // Set the initial pose
    pro_joint_ctrl.moveToPose("initial_pose", 5.0, true);

    return 0;
}
