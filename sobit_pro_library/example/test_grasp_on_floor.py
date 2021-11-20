#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
from subprocess import *
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_joint_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
      
    # 物体認識Launchさせる
    Popen(['gnome-terminal', '--', 'roslaunch', 'sobit_pro_bringup', 'object_detection_azure_kinect.launch'])
    rospy.sleep(5.0)


    # 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "detecting_pose" )
    rospy.sleep(5.0)

    # ハンドを動かす（開く）
    pro_joint_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

    # 把持する物体のTF(例：beans)があった場合、
    # そこの位置までアームを移動させる
    res = pro_joint_ctr.moveGripperToTargetTF( "potato_chips", -0.15, 0.0, 0.05 )

    # 把持する対象の物体の座標を指定して、
    # そこの位置までアームを移動させる
    # res = pro_joint_ctr.moveGripperToTargetCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.05 )

    # 物体を掴むことが出来たかの確認
    grasp = pro_joint_ctr.graspDecision()

    if( (res == True) and (grasp == True) ):
        # ハンドを動かす（閉じる）
        pro_joint_ctr.moveJoint( Joint.GRIPPER_JOINT, 0.0, 2.0, True )

        # 決められたポーズをする
        pro_joint_ctr.moveToRegisterdMotion( "grasp_high_pose" )

    # 決められたポーズをする
    pro_joint_ctr.moveToRegisterdMotion( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
