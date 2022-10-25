#include "ros/ros.h"
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>
#include <sobit_pro_library/sobit_pro_joint_controller.h>


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test");

    sobit_pro::SobitProJointController pro_joint_ctrl;
    sobit_pro::SobitProWheelController pro_wheel_ctrl;
    double grasp_flag = false;
    std::string target_name = "potato_chips";

    // Pose: detecting_pose
    pro_joint_ctrl.moveToPose("detecting_pose");
    ros::Duration(3.0).sleep();

    // Open the hand
    pro_joint_ctrl.moveJoint( sobit_pro::Joint::HAND_JOINT, 1.57, 2.0, true );
    ros::Duration(2.0).sleep();

    // Move the hand towards the target "target_name" and check if grasped
    grasp_flag = (pro_joint_ctrl.moveGripperToTargetTF(target_name, -0.15, 0.0, 0.05)) & (pro_joint_ctrl.graspDecision());

    // Move the hand towards the target depending on its coords
    // grasp_flag = pro_joint_ctrl.moveGripperToPlaceablePositionCoord(0.5, 0.5, 0.5, -0.15, 0.0, 0.05);
    std::cout << "Is grasped? " << (grasp_flag ? "True":"False") << std::endl;

    if (grasp_flag){
        // Close Hand
        pro_joint_ctrl.moveJoint(sobit_pro::Joint::HAND_JOINT, 0.0, 2.0, true);

        //Pose: grasp_high_pose
        pro_joint_ctrl.moveToPose("grasp_high_pose");
        ros::Duration(2.0).sleep();
    }

    // Pose: "initial pose"
    pro_joint_ctrl.moveToPose("initial_pose");
    ros::Duration(2.0).sleep();

    return 0;
}
