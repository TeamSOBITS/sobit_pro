#!/usr/bin/env python3
import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import Joint
from geometry_msgs.msg import Point
import sys
from subprocess import Popen
from std_srvs.srv import SetBool

def test():
    rospy.init_node('test')
    args = sys.argv
    pro_joint_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pro_wheel_ctr = SobitProWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # 決められたポーズをする
    pro_joint_ctr.moveToPose( "detecting_pose" )
    rospy.sleep(5.0)

    """
    # 置ける位置を検出するノードを立てる
    Popen(['roslaunch','sobit_pro_bringup','placeable_position_estimator.launch'])
    rospy.sleep(10.0)

    # 検出モードをオンにする
    rospy.wait_for_service('placeable_position_estimator/execute_ctrl')
    try:
        client = rospy.ServiceProxy('calculate_two_numbers', SetBool)
        client(True)
    except rospy.ServiceException as e:
        print("Failed to call service calculate_two_numbers : %s" %e)
    rospy.sleep(10.0)
    """

    # 物体をおける位置のTF(placeable_point)があった場合、
    # そこの位置までアームを移動させる
    res = pro_joint_ctr.moveGripperToTargetTF( "placeable_point", -0.45, 0.0, 0.07 )

    # 物体をおける位置の座標を指定して、
    # そこの位置までアームを移動させる
    # res = pro_joint_ctr.moveGripperToTargetCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.07 )

    # 物体をおける位置のTF(placeable_point)があった場合、
    # そこの位置までアームを下げながら移動させる
    # ただし、物体がおける位置に触れた時はその位置で停止する
    # res = pro_joint_ctr.moveGripperToPlaceTF( "placeable_point", -0.15, 0.0, 0.2 )

    # 物体をおける位置のTF(placeable_point)があった場合、
    # そこの位置までアームを下げながら移動させる
    # ただし、物体がおける位置に触れた時はその位置で停止する
    # res = pro_joint_ctr.moveGripperToPlaceCoord( 0.0, 0.0, 0.0, -0.15, 0.0, 0.2 )

    if( res == True ){

        # タイヤ車輪をを動かす(並進運動)
        pro_wheel_ctr.controlWheelLinear(0.3, 0.0)

        # ハンドを動かす
        pro_joint_ctr.moveJoint( Joint.GRIPPER_JOINT, -1.57, 2.0, True )

        # 決められたポーズをする
        pro_joint_ctr.moveToPose( "put_high_pose" )
    }

    # 決められたポーズをする
    pro_joint_ctr.moveToPose( "initial_pose" )

    """
    Popen(['rosnode','kill','/placeable_position_estimator/placeable_position_estimater_node'])
    rospy.sleep(6)
    """

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
