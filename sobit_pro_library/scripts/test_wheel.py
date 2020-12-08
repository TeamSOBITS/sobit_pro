#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProWheelController
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr.controlWheelLinear(1.0)
    #pro_wheel_ctr.controlWheelRotateRad(1.57)
    #pro_wheel_ctr.controlWheelRotateDeg(-90)
    #pro_wheel_ctr.controlWheelLinear(-1.0)
    r.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
