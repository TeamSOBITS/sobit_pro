#include <sobit_pro_library/sobit_pro_joint_controller.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_pro_joiunt_controller_test");
    sobit::SobitProJointController sobit_pro_ctr;

    /*  arm controll  */
    /* arm1    =  1.0 */
    /* arm2    =  1.0 */
    /* arm3    = -1.0 */
    /* arm4    = -1.0 */
    sobit_pro_ctr.moveArm( 1.0, 1.0, -1.0, -1.0);

    return 0;
}
