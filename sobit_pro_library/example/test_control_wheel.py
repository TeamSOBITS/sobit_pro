#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProWheelController
import sys
import numpy as np
import math
from geometry_msgs.msg import Twist


def test():
    rospy.init_node('test')
    args = sys.argv
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=1)
    vel = Twist()

    # for linear in np.arange(-1.0,1,0.1):
    vel.linear.x = 0.1
    for angular in np.arange(-math.pi/2,math.pi/2,0.05):
        vel.angular.z = angular
        pub.publish(vel)
        print(f"linear = \t{vel.linear.x}\tangular = \t{vel.angular.z}")
        rospy.sleep(1.0)
    for angular in np.arange(math.pi/2,-math.pi/2,0.05):
        vel.angular.z = angular
        pub.publish(vel)
        print(f"linear = \t{vel.linear.x}\tangular = \t{vel.angular.z}")
        rospy.sleep(1.0)
    zero_vel = Twist()
    pub.publish(zero_vel)

    # # タイヤ車輪をを動かす(並進運動)
    # pro_wheel_ctr.controlWheelLinear(1.0, 0.0)

    # # タイヤ車輪をを動かす(回転運動：Radian)
    # pro_wheel_ctr.controlWheelRotateRad(1.57)

    # # タイヤ車輪をを動かす(回転運動：Degree)
    # pro_wheel_ctr.controlWheelRotateDeg(-90)

    # # タイヤ車輪をを動かす(並進運動)
    # pro_wheel_ctr.controlWheelLinear(-1.0, 0.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
