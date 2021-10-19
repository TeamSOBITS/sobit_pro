#!/usr/bin/env python
# coding: utf-8

import rospy
import time
import sensor_msgs.msg
import trajectory_msgs.msg
from geometry_msgs.msg import Twist

class PS4_control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.subscribe_joy, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        # rate
        self.rate = rospy.Rate(10)
        
        # subscriberのメッセージを受け取る変数
        self.joy_button = [0] * 17
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        # self.right_joystick_lr = 0
        # self.right_joystick_ud = 0
        self.magnifications = 0.2

    def subscribe_joy(self, msg):
        self.joy_button = msg.buttons
        self.left_joystick_lr = msg.axes[0] * self.magnifications
        self.left_joystick_ud = msg.axes[1] * self.magnifications
        # self.right_joystick_lr = msg.axes[3] * self.magnifications
        # self.right_joystick_ud = msg.axes[4] * self.magnifications
        
        # L2ボタンが押される
        if self.joy_button[6] == True:
            self.move_wheel_stop_motion()
            # print("回転運動(左回り)")
            self.move_wheel_rotational_motion(0.3)

        # R2ボタンが押される
        elif self.joy_button[7] == True:
            self.move_wheel_stop_motion()
            # print("回転運動(右回り)")
            self.move_wheel_rotational_motion(-0.3)

        # それ以外
        else:
            self.move_wheel_stop_motion()
            # print("並進運動")
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
    def move_wheel_translational_motion(self, liner):
        speed = Twist()
        speed.linear.x = self.left_joystick_ud * liner
        speed.linear.y = self.left_joystick_lr * liner
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(speed)

if __name__ == '__main__':
    rospy.init_node('sobit_pro_ps4_control_node')
    ps4_control = PS4_control()
    rospy.spin()

############ メモ ############

# L2ボタンが押される
# self.joy_button[6] == True

# R2ボタンが押される
# self.joy_button[7] == True
        
# L1ボタンが押される
# self.joy_button[4] == True

# R1ボタンが押される
# self.joy_button[5] == True

# ×ボタンが押される
# self.joy_button[1] == True

# ○ボタンが押される
# self.joy_button[2] == True

# △ボタンが押される
# self.joy_button[3] == True

# □ボタンが押される
# self.joy_button[0] == True

# スタートボタン
# self.joy_button[12] == True

# 左スティック
# self.left_joystick_lr

# 左スティック
# self.left_joystick_ud

# 右スティック
# self.right_joystick_lr

# 右スティック
# self.right_joystick_ud

##############################