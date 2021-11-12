#include <sobit_pro_library/sobit_pro_joint_controller.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_pro_joiunt_controller_test");
    sobit_pro::SobitProJointController sobit_pro_ctr;

    sobit_pro_ctr.moveToRegisterdMotion( "detecting_pose" );
    ros::Duration(5.0).sleep();

    sobit_pro_ctr.moveGripperToPlaceablePosition("placeable_point", -0.15, 0.0, 0.2);

    return 0;
}
