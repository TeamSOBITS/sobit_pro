#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint


def test_grasp_on_floor():
    rospy.init_node('sobit_pro_test_grasp_on_floor')

    args = sys.argv
    pro_joint_ctrl = SobitProJointController(args[0])

    target_name = "potato_chips"
    is_done     = False

    # Set the detecting_pose
    pro_joint_ctrl.moveToPose( "detecting_pose", 5.0, True )

    # Open the hand
    pro_joint_ctrl.moveJoint( Joint.HAND_JOINT, -1.57, 5.0, True )

    # Option 1: Grasp the target on the given TF position
    # Move the hand to the target position
    is_done  = pro_joint_ctrl.moveHandToTargetTF( target_name, -0.15,0.0,0.05 )
    # Close the hand
    is_done *= pro_joint_ctrl.moveJoint( Joint.HAND_JOINT, 0.0, 5.0, True )
    # Check if grasped based on the force sensor
    is_done *= pro_joint_ctrl.graspDecision( 300, 1000 )

    """
    # Option 2: Grasp the target on the given coordinates (x,y,z) position
    # Check if grasped based on the force sensor
    is_done  = pro_joint_ctrl.moveHandToTargetCoord( 0.0,0.0,0.0, -0.15,0.0,0.05 )
    # Close the hand
    is_done *= pro_joint_ctrl.moveJoint( Joint.HAND_JOINT, 0.0, 5.0, true )
    # Check if grasped based on the force sensor
    is_done *= pro_joint_ctrl.graspDecision( 300, 1000 )
    """

    if( is_done ):
        # Set the put_high_pose pose to avoid collision
        pro_joint_ctrl.moveToPose("grasp_high_pose", 5.0, True)
    else:
        rospy.logerr("Failed to grasp the object")

    # Set the initial pose
    pro_joint_ctrl.moveToPose( "initial_pose", 5.0, True )

    del pro_joint_ctrl
    del args


if __name__ == '__main__':
    try:
        test_grasp_on_floor()
    except rospy.ROSInterruptException:
        pass
