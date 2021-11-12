#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys
from subprocess import Popen
from std_srvs.srv import SetBool

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_joint_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0])       # args[0] : C++上でros::init()を行うための引数

    # 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "detecting_pose" )
    rospy.sleep(5.0)

    """
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
    """

    # 物体を置ける位置に置く処理をする
    res = pro_joint_ctr.moveGripperToPlaceablePosition("placeable_point", -0.15, 0.0, 0.2)
    print("result : ", res)

    """
    # 物体を置くことができる位置があった場合、
    # そこの位置までアームを移動させる
    res = pro_joint_ctr.moveGripperToTarget("placeable_point", -0.15, 0.0, 0.08)
    print("result : ", res)
    rospy.sleep(2.0)
    """

    # ハンドを動かす
    pro_joint_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

    # 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "put_high_pose" )
    pro_joint_ctr.moveToRegisterdMotion( "initial_pose" )

    Popen(['rosnode','kill','/placeable_position_estimator/placeable_position_estimater_node'])
    rospy.sleep(6)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
