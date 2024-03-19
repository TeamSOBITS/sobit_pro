#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint


def test_control_head():
    rospy.init_node('sobit_pro_test_control_head')

    args = sys.argv
    pro_joint_ctrl = SobitProJointController(args[0])

    MAX_ANGLE    =  1.57
    MIN_ANGLE    = -1.57
    target_angle =  0.0
    increment    =  0.05

    while not rospy.is_shutdown():
        target_angle += increment
        if target_angle > MAX_ANGLE or target_angle < MIN_ANGLE:
            increment *= -1.0

        # Option 1: Move the head joints simultaneously
        pro_joint_ctrl.moveHeadPanTilt( target_angle, target_angle, 0.5, False )
        rospy.sleep(0.5)

        """
        # Option 2: Move the head joints individually
        pro_joint_ctrl.moveJoint( Joint.HEAD_PAN_JOINT , target_angle, 0.5, False )
        pro_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, target_angle, 0.5, False )
        """

    del pro_joint_ctrl
    del args


if __name__ == '__main__':
    try:
        test_control_head()
    except rospy.ROSInterruptException:
        pass
