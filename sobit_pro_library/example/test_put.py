#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys
from subprocess import Popen

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_arm_pantilt_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0])       # args[0] : C++上でros::init()を行うための引数

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "detecting_pose" )
    rospy.sleep(5.0)

    Popen(['roslaunch','sobit_pro_bringup','sobit_pro_placeable_position_estimator.launch'])
    rospy.sleep(6.0)

    # 物体を置くことができる位置があった場合、
    # そこの位置までアームを移動させる
    res = pro_arm_pantilt_ctr.moveGripperToTarget("placeable_point", -0.15, 0.0, 0.08)
    print("result : ", res)
    rospy.sleep(2.0)

    # ハンドを動かす
    pro_arm_pantilt_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "put_high_pose" )
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "initial_pose" )

    Popen(['rosnode','kill','/placeable_position_estimator/placeable_position_estimater_node'])
    rospy.sleep(6)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
