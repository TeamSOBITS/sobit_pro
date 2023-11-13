#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import math

import rospy
from sobit_pro_module import SobitProWheelController

from geometry_msgs.msg import Twist


def test_control_wheel():
    rospy.init_node('sobit_pro_test_control_wheel')

    args = sys.argv
    pro_wheel_ctrl = SobitProWheelController(args[0])

    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=1)
    vel = Twist()
    vel.linear.x = 0.1

    for angular in np.arange(-math.pi/2, math.pi/2, 0.05):
        vel.angular.z = angular
        print("linear = %f\tangular = %f", vel.linear.x, vel.angular.z)
        pub.publish(vel)
        rospy.sleep(1.0)
    
    for angular in np.arange(math.pi/2, -math.pi/2, 0.05):
        vel.angular.z = angular
        print("linear = %f\tangular = %f", vel.linear.x, vel.angular.z)
        pub.publish(vel)
        rospy.sleep(1.0)
    
    zero_vel = Twist()
    pub.publish(zero_vel)

    """
    # Move the wheels (linear motion)
    pro_wheel_ctrl.controlWheelLinear(1.0, 0.0)

    # Move the wheels (rotational motion: Radian)
    pro_wheel_ctrl.controlWheelRotateRad(1.57)

    # Move the wheels (rotational motion: Degree)
    pro_wheel_ctrl.controlWheelRotateDeg(-90)

    # Move the wheels (linear motion)
    pro_wheel_ctrl.controlWheelLinear(-1.0, 0.0)
    """

if __name__ == '__main__':
    try:
        test_control_wheel()
    except rospy.ROSInterruptException:
        pass
