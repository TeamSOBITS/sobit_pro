#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProLibrary
from sobit_pro_module import Joint
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    edu_ctr = SobitProLibrary(args[0]) # args[0] : C++上でros::init()を行うための引数
    while not rospy.is_shutdown():
        ang = -1.0 * ang
        edu_ctr.moveJoint( Joint.HEAD_CAMERA_PAN_JOINT, ang, 0.8, False )
        r.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
