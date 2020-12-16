#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    pro_arm_pantilt_ctr = SobitProJointController(args[0])       # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "initial_pose" )

    # カメラパンチルトを動かす
    pro_arm_pantilt_ctr.moveHeadPanTilt( 0.0, -0.8, 2.0, True )
    rospy.sleep(5.0)

    # 把持する対象の物体が合った場合、
    # そこの位置までアームを移動させる
    res = pro_arm_pantilt_ctr.moveGripperToTarget("onion_soup", -0.15, 0.0, 0.03)
    print("result : ", res)
    rospy.sleep(2.0)
    
    # ハンドを動かす
    pro_arm_pantilt_ctr.moveJoint( Joint.GRIPPER_JOINT, 0.0, 2.0, True )

    # 決められたポーズをする
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "high_pose" )
    rospy.sleep(0.5)
    pro_arm_pantilt_ctr.moveToRegisterdMotion( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
