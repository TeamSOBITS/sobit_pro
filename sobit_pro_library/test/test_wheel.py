#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProWheelController
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
    
    # タイヤ車輪をを動かす
    pro_wheel_ctr.controlWheelLinear(1.0, 0.0)
    pro_wheel_ctr.controlWheelRotateRad(1.57)
    pro_wheel_ctr.controlWheelRotateDeg(-90)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
