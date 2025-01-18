#!/usr/bin/env python3
# coding: utf-8

import sys

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from sobit_pro_module import SobitProWheelController
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint


class JoyControl:
    def __init__(self):
        self.sub_joy           = rospy.Subscriber('joy', Joy, self.subscribe_joy, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

        # Init SobitProWheelController and SobitProJointController
        self.args = sys.argv
        self.pro_wheel_ctrl = SobitProWheelController(self.args[0])
        self.pro_joint_ctrl = SobitProJointController(self.args[0])

        # Set initial values
        self.tilt_ang = 0.0
        self.pan_ang  = 0.0
        self.time     = 3.0

        # ROS rate
        self.rate = rospy.Rate(10)

        # Parameters
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

        # L2 button is pressed
        if self.joy_button[6]:
            self.move_wheel_stop_motion()
            self.move_wheel_rotational_motion(0.3)

        # R2 button is pressed
        elif self.joy_button[7]:
            self.move_wheel_stop_motion()
            self.move_wheel_rotational_motion(-0.3)

        # Circle button is pressed
        elif self.joy_button[1]:
            self.move_wheel_stop_motion()

            for i in range(4):
                self.pro_wheel_ctrl.controlWheelRotateDeg(90)

        # Mode: translational motion
        elif msg.axes[1] > 0:
            self.move_wheel_stop_motion()
            self.move_wheel_translational_motion(0.8)
            self.rate.sleep()

        # Right stick is tilted up
        elif msg.axes[4] > 0.8:
            self.move_wheel_stop_motion()

            self.tilt_ang += 0.015 if self.tilt_ang < 0.6 else 0

            # Move the camera tilt
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, self.tilt_ang, self.time, False )
            self.rate.sleep()

        # Right stick is tilted down
        elif msg.axes[4] < -0.8:
            self.move_wheel_stop_motion()

            self.tilt_ang += -0.015 if self.tilt_ang > -0.6 else 0

            # Move the camera tilt
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, self.tilt_ang, self.time, False )
            self.rate.sleep()

        # Right stick is tilted left
        elif msg.axes[3] < -0.8:
            self.move_wheel_stop_motion()

            self.pan_ang += -0.015 if self.pan_ang > -0.6 else 0

            # Move the camera pan
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_PAN_JOINT, self.pan_ang, self.time, False )
            self.rate.sleep()
            # print(self.pan_ang)

        # Right stick is tilted right
        elif msg.axes[3] > 0.8:
            self.move_wheel_stop_motion()

            if self.pan_ang <= 0.6: self.pan_ang = 0.6
            else: self.pan_ang += 0.015

            self.pan_ang += 0.015 if self.pan_ang > 0.6 else 0

            # Move the camera pan tilt
            self.pro_joint_ctrl.moveJoint( Joint.HEAD_PAN_JOINT, self.pan_ang, self.time, False )
            self.rate.sleep()

        # Something else is pressed
        else:
            self.move_wheel_stop_motion()
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

        if enable_diagonal:
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
            speed.linear.x = self.left_joystick_ud * linear
            speed.linear.y = self.left_joystick_lr * linear
            speed.angular.z = self.right_joystick_lr * 1.57

            if abs(speed.angular.z) <= 0.1: speed.angular.z = 0
            else: speed.linear.y = 0

            self.check_publishers_connection(self.pub_wheel_control)
            self.pub_wheel_control.publish(speed)


if __name__ == '__main__':
    rospy.init_node('sobit_pro_ps4_control_node')
    enable_diagonal = rospy.get_param("~enable_diagonal", False)

    ps4_control = JoyControl()
    rospy.spin()


############ Buttons List ############

# cross button
# self.joy_button[0]

# circle button
# self.joy_button[1]

# triangle button
# self.joy_button[2]

# square button
# self.joy_button[3]

# L1 button
# self.joy_button[4]

# R1 button
# self.joy_button[5]

# L2 button
# self.joy_button[6]

# R2 button
# self.joy_button[7]

# Share button
# self.joy_button[8]

# Options button
# self.joy_button[9]

# PS button
# self.joy_button[10]

# L3 button
# self.joy_button[11]

# R3 button
# self.joy_button[12]

# Left stick
# self.left_joystick_lr

# Left stick
# self.left_joystick_ud

# Right stick
# self.right_joystick_lr

# Right stick
# self.right_joystick_ud

##############################