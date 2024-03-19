#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
from geometry_msgs.msg import Twist
from sobit_pro_module import SobitProWheelController
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint

import sensor_msgs.msg


class Ps3_Control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.subscribe_joy, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.args = sys.argv
        self.pro_wheel_ctrl = SobitProWheelController(self.args[0])
        self.pro_joint_ctrl = SobitProJointController(self.args[0])

        self.tilt_ang = 0.0
        self.pan_ang  = 0.0
        self.time     = 0.5

        # rate
        self.rate = rospy.Rate(10)

        # subscriberのメッセージを受け取る変数
        self.joy_button        = [0] * 17
        self.left_joystick_lr  = 0
        self.left_joystick_ud  = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications    = 0.2


    def subscribe_joy(self, msg):
        self.joy_button        = msg.buttons
        self.left_joystick_lr  = msg.axes[0] * self.magnifications
        self.left_joystick_ud  = msg.axes[1] * self.magnifications
        self.right_joystick_lr = msg.axes[3] * self.magnifications
        self.right_joystick_ud = msg.axes[4] * self.magnifications

        # L2ボタンが押される
        if self.joy_button[6]:
            self.move_wheel_stop_motion()

            # print("回転運動(左回り)")
            self.move_wheel_rotational_motion(0.3)

        # R2ボタンが押される
        elif self.joy_button[7]:
            self.move_wheel_stop_motion()

            # print("回転運動(右回り)")
            self.move_wheel_rotational_motion(-0.3)

        # ○ボタンが押される
        elif self.joy_button[1]:
            self.move_wheel_stop_motion()

            # print("１回転(右回り)")
            for i in range(4):
                self.pro_wheel_ctrl.controlWheelRotateDeg(90)

        # ↑ボタンが押される
        elif self.joy_button[13]:
            self.move_wheel_stop_motion()

            # print("カメラパンチルト（上）")
            if self.tilt_ang >= 0.6: self.tilt_ang = 0.6
            else: self.tilt_ang += 0.015
            # カメラパンチルトを動かす
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, self.tilt_ang, self.time, False )
            self.rate.sleep()
            # print(self.tilt_ang)

        # ↓ボタンが押される
        elif self.joy_button[14]:
            self.move_wheel_stop_motion()

            # print("カメラパンチルト（下）")
            if self.tilt_ang <= -0.6: self.tilt_ang = -0.6
            else: self.tilt_ang += -0.015

            # カメラパンチルトを動かす
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, self.tilt_ang, self.time, False )
            self.rate.sleep()
            # print(self.tilt_ang)

        # ←ボタンが押される
        elif self.joy_button[15]:
            self.move_wheel_stop_motion()

            # print("カメラパンチルト（左）")
            if self.pan_ang >= 0.6: self.pan_ang = 0.6
            else: self.pan_ang += 0.015

            # カメラパンチルトを動かす
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_PAN_JOINT, self.pan_ang, self.time, False )
            self.rate.sleep()
            # print(self.pan_ang)

        # →ボタンが押される
        elif self.joy_button[16]:
            self.move_wheel_stop_motion()

            # print("カメラパンチルト（右）")
            if self.pan_ang <= -0.6: self.pan_ang = -0.6
            else: self.pan_ang += -0.015

            # カメラパンチルトを動かす
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_PAN_JOINT, self.pan_ang, self.time, False )
            self.rate.sleep()
            # print(self.pan_ang)

        # それ以外
        else:
            self.move_wheel_stop_motion()

            # print("並進運動 or 旋回運動")
            self.move_wheel_translational_motion(0.8)
            self.rate.sleep()

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)

        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    # Stop motion
    def move_wheel_stop_motion(self):
        speed = Twist()
        speed.angular.z = 0
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(speed)

    # Rotational motion
    def move_wheel_rotational_motion(self, angular):
        speed = Twist()
        speed.angular.z = angular
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(speed)

    # Translational motion
    def move_wheel_translational_motion(self, linear):
        speed = Twist()

        if diagonal_path:
            if abs(self.left_joystick_ud) < abs(self.left_joystick_lr) :
                speed.linear.x = 0
                speed.linear.y = self.left_joystick_lr * linear
            elif abs(self.left_joystick_ud) >= abs(self.left_joystick_lr) :
                speed.linear.x = self.left_joystick_ud * linear
                speed.linear.y = 0
                speed.angular.z = self.right_joystick_lr * 1.57
                if abs(speed.angular.z) <= 0.1:
                    speed.angular.z = 0

            self.check_publishers_connection(self.pub_wheel_control)
            self.pub_wheel_control.publish(speed)
        else:
            speed.linear.x  = self.left_joystick_ud  * linear
            speed.linear.y  = self.left_joystick_lr  * linear
            speed.angular.z = self.right_joystick_lr * 1.57

            if abs(speed.angular.z) <= 0.1: speed.angular.z = 0
            else: speed.linear.y = 0

            self.check_publishers_connection(self.pub_wheel_control)
            self.pub_wheel_control.publish(speed)


if __name__ == '__main__':
    rospy.init_node('sobit_pro_ps3_control_node')
    diagonal_path = rospy.get_param("diagonal_path",None)

    ps3_control = Ps3_Control()
    rospy.spin()


############ メモ ############

# ×ボタンが押される
# self.joy_button[0]

# ○ボタンが押される
# self.joy_button[1]

# △ボタンが押される
# self.joy_button[2]

# □ボタンが押される
# self.joy_button[3]

# L1ボタンが押される
# self.joy_button[4]

# R1ボタンが押される
# self.joy_button[5]

# L2ボタンが押される
# self.joy_button[6]

# R2ボタンが押される
# self.joy_button[7]

# セレクトボタンが押される
# self.joy_button[8]

# スタートボタンが押される
# self.joy_button[9]

# 左スティック
# self.left_joystick_lr

# 左スティック
# self.left_joystick_ud

# 右スティック
# self.right_joystick_lr

# 右スティック
# self.right_joystick_ud

##############################
# PS3コントローラー

# self.joy_button[0] ×
# self.joy_button[1] ○
# self.joy_button[2] △
# self.joy_button[3] □
# self.joy_button[4] L1
# self.joy_button[5] R1
# self.joy_button[6] L2
# self.joy_button[7] R2
# self.joy_button[8] SELECT
# self.joy_button[9] START
# self.joy_button[10] PS
# self.joy_button[11] L3
# self.joy_button[12] R3
# self.joy_button[13] ↑
# self.joy_button[14] ↓
# self.joy_button[15] ←
# self.joy_button[16] →

# msg.axes[0] 左スティック上下
# msg.axes[1] 左スティック左右
# msg.axes[2] L2
# msg.axes[3] 右スティック上下
# msg.axes[4] 右スティック左右
# msg.axes[5] R2
