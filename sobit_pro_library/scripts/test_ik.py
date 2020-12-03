#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys


def test():
    rospy.init_node('test')
    r = rospy.Rate(10)  # 10hz
    args = sys.argv
    spjc = SobitProJointController(args[0])  # args[0] : C++上でros::init()を行うための引数
    shift = Point()
    print(shift)
    res = spjc.moveGripperToTarget("object")
    print("result : ", res)


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
