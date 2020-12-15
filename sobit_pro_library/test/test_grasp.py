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
    pro_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数

    pro_ctr.moveToRegisterdMotion( "initial_pose" )
    pro_ctr.moveHeadPanTilt( 0.0, -0.8, 2.0, True )
    pro_ctr.moveArm( 1.0, 1.0, -1.0, -1.0, -1.0 )
    rospy.sleep(5.0)

    res = pro_ctr.moveGripperToTarget("onion_soup")
    print("result : ", res)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
