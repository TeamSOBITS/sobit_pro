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

    ###  arm controll  ###
    ### arm1    =  1.0 ###
    ### arm2    =  1.0 ###
    ### arm3    = -1.0 ###
    ### arm4    = -1.0 ###
    ### gripper = -1.0 ###
    pro_ctr.moveArm( 1.0, 1.0, -1.0, -1.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
