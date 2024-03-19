#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from subprocess import Popen

import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint

from std_srvs.srv import SetBool


def test_place_on_table():
    rospy.init_node('sobit_pro_test_place_on_table')
    
    args = sys.argv
    pro_joint_ctrl = SobitProJointController(args[0])
    pro_wheel_ctrl = SobitProWheelController(args[0])

    # Set the detecting_pose
    pro_joint_ctrl.moveToPose( "detecting_pose", 5.0, True )

    """
    # Lauch the placeable_position_estimator node
    package_name = "sobit_pro_bringup"
    node_name    = "placeable_position_estimator"
    launch_name  = node_name + ".launch"
    comand       = "roslaunch " + package_name + " " + launch_name
    p = Popen(comand.split(), shell=True)
    p.wait()

    if( not p.returncode ){
        rospy.logerr("Failed to launch the placeable_position_estimator node");
    }

    # Turn on detection service
    rospy.wait_for_service('placeable_position_estimator/execute_ctrl')
    try:
        client = rospy.ServiceProxy('placeable_position_estimator/execute_ctrl', SetBool)
        client(True)
    except rospy.ServiceException as e:
        rospy.logerr( "Failed to call service placeable_position_estimator : %s" % e )

    rospy.sleep(10.0)
    """

    # Option 1: Place object on the given TF position
    # Arm will move down until it touches the placeable_point
    is_done = pro_joint_ctrl.moveHandToPlaceTF( "placeable_point", -0.15, 0.0, 0.2 )

    # Option 2: Place object on the given coordinates (x,y,z) position
    # Arm will move down until it touches the placeable_point
    # res = pro_joint_ctrl.moveHandToPlaceCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.2 )

    if( is_done ):
        # Open the hand
        pro_joint_ctrl.moveJoint( Joint.HAND_JOINT, -1.57, 2.0, True )

        # Set the put_high_pose pose to avoid collision
        pro_joint_ctrl.moveToPose( "put_high_pose", 5.0, True )
    else:
        rospy.logerr( "Failed to place the object" )

    # Set the initial pose
    pro_joint_ctrl.moveToPose( "initial_pose", 5.0, True )

    del pro_joint_ctrl
    del pro_wheel_ctrl
    del args

    """
    # Kill the placeable_position_estimator node
    command = "rosnode kill /" + node_name + "/" + node_name + "_node"
    p = Popen(command.split(), shell=True)
    p.wait()

    if( not p.returncode ){
        rospy.logerr("Failed to kill the placeable_position_estimator node");
    }

    """


if __name__ == '__main__':
    try:
        test_place_on_table()
    except rospy.ROSInterruptException:
        pass
