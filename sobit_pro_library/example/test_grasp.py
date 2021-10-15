#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_arm_pantilt_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0])       # args[0] : C++上でros::init()を行うための引数

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "detecting_pose" )
    rospy.sleep(5.0)

    # ハンドを動かす
    pro_arm_pantilt_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

    # 把持する対象の物体があった場合、
    # そこの位置までアームを移動させる
    res = pro_arm_pantilt_ctr.moveGripperToTarget("beans", -0.15, 0.0, 0.05)
    print("result : ", res)
    rospy.sleep(2.0)
    
    # ハンドを動かす
    pro_arm_pantilt_ctr.moveJoint( Joint.GRIPPER_JOINT, 0.0, 2.0, True )

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "grasp_high_pose" )
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
