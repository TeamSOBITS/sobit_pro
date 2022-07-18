#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    pro_joint_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # アームを動かす
    pro_joint_ctr.moveArm( 1.0, 1.0, -1.0, -1.0)

    # ハンドを動かす
    pro_joint_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

    # 決められたポーズをする
    pro_joint_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
