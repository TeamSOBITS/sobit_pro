#include <sobit_pro_library/sobit_pro_joint_controller.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_pro_joiunt_controller_test");
    sobit_pro::SobitProJointController pro_joint_ctr;

    // 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "detecting_pose" );
    ros::Duration(5.0).sleep();

    /* Python版
    # 置ける位置を検出するノードを立てる
    Popen(['roslaunch','sobit_pro_bringup','placeable_position_estimator.launch'])
    rospy.sleep(10.0)

    # 検出モードをオンにする
    rospy.wait_for_service('placeable_position_estimator/execute_ctrl')
    try:
        client = rospy.ServiceProxy('calculate_two_numbers', SetBool)
        client(True)
    except rospy.ServiceException as e:
        print("Failed to call service calculate_two_numbers : %s" %e)
    rospy.sleep(10.0)
    */

    // 物体をおける位置のTF(placeable_point)があった場合、
    // そこの位置までアームを移動させる
    bool res = pro_joint_ctr.moveGripperToTargetTF( "placeable_point", -0.15, 0.0, 0.07 );

    // 物体をおける位置の座標を指定して、
    // そこの位置までアームを移動させる
    // bool res = pro_joint_ctr.moveGripperToTargetCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.07 );

    // 物体をおける位置のTF(placeable_point)があった場合、
    // そこの位置までアームを下げながら移動させる
    // ただし、物体がおける位置に触れた時はその位置で停止する
    // bool res = pro_joint_ctr.moveGripperToPlaceablePositionTF( "placeable_point", -0.15, 0.0, 0.2 );

    // 物体をおける位置のTF(placeable_point)があった場合、
    // そこの位置までアームを下げながら移動させる
    // ただし、物体がおける位置に触れた時はその位置で停止する
    // bool res = pro_joint_ctr.moveGripperToPlaceablePositionCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.2 );

    

    if ( res == true ) {

        // 決められたポーズをする
        pro_joint_ctr.moveJoint( sobit_pro::Joint::GRIPPER_JOINT, -1.57, 2.0, true );

        // 決められたポーズをする
        pro_joint_ctr.moveToRegisterdMotion( "put_high_pose" );
    }

    // 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "initial_pose" );

    /* Python版
    Popen(['rosnode','kill','/placeable_position_estimator/placeable_position_estimater_node'])
    rospy.sleep(6)
    */

    return 0;
}