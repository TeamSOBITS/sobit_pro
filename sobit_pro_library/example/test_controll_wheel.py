#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProWheelController
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
    
    # タイヤ車輪をを動かす(並進運動)
    pro_wheel_ctr.controlWheelLinear(1.0, 0.0)

    # タイヤ車輪をを動かす(回転運動：Radian)
    pro_wheel_ctr.controlWheelRotateRad(1.57)

    # タイヤ車輪をを動かす(回転運動：Degree)
    pro_wheel_ctr.controlWheelRotateDeg(-90)

    # タイヤ車輪をを動かす(並進運動)
    pro_wheel_ctr.controlWheelLinear(-1.0, 0.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
