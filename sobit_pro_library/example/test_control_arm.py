#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint


def test_control_arm():
    rospy.init_node('sobit_pro_test_control_arm')

    args = sys.argv
    pro_joint_ctrl = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # Move all the arm joints
    pro_joint_ctrl.moveArm( 1.0, 1.0, -1.0, 0.0, -1.0, 3.0, True )

    # Open the hand
    pro_joint_ctrl.moveJoint( Joint.HAND_JOINT, -1.57, 5.0, True )

    # Set the initial pose
    pro_joint_ctrl.moveToPose( "initial_pose", 5.0, True )

    del pro_joint_ctrl
    del args


if __name__ == '__main__':
    try:
        test_control_arm()
    except rospy.ROSInterruptException:
        pass
