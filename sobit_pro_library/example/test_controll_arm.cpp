#include <sobit_pro_library/sobit_pro_joint_controller.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_pro_joiunt_controller_test");
    sobit_pro::SobitProJointController pro_joint_ctr;

    // アームを動かす
    pro_joint_ctr.moveArm( 1.0, 1.0, -1.0, -1.0 );

    // ハンドを動かす
    pro_joint_ctr.moveJoint( sobit_pro::Joint::GRIPPER_JOINT, -1.57, 2.0, true );

    // 決められたポーズをする
    pro_joint_ctr.moveToPose( "initial_pose" );

    return 0;
}
