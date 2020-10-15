#!/usr/bin/env python
# coding: utf-8

import rospy
import time
from geometry_msgs.msg import Twist

def right():
    speed = Twist()
    speed.linear.y = 0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def left():
    speed = Twist()
    speed.linear.y = -0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def back():
    speed = Twist()
    speed.linear.x = -0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def forward():
    speed = Twist()
    speed.linear.x = 0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def diagonal_forward():
    speed = Twist()
    speed.linear.x = 0.5
    speed.linear.y = 0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def diagonal_back():
    speed = Twist()
    speed.linear.x = -0.5
    speed.linear.y = -0.5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def rotation_forward():
    speed = Twist()
    speed.angular.z = 5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def rotation_back():
    speed = Twist()
    speed.angular.z = -5

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

    elapsed_time = 0
    while(elapsed_time < 3):
        t2 = time.time() # 処理後の時刻
        elapsed_time = t2-t1 # 経過時間

def initial():
    speed = Twist()
    speed.angular.z = 0

    t1 = time.time() # 処理前の時刻
    pub.publish(speed)

# メイン
if __name__ == '__main__':
    rospy.init_node('test')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

    print "start"
    rospy.sleep(5)

    right()
    initial()
    rospy.sleep(0.5)

    left()
    initial()
    rospy.sleep(1.5)

    back()
    initial()
    rospy.sleep(0.5)

    forward()
    initial()
    rospy.sleep(1.5)

    diagonal_forward()
    initial()
    rospy.sleep(0.5)

    diagonal_back()
    initial()
    rospy.sleep(1.5)

    rotation_forward()
    initial()
    rospy.sleep(0.5)

    rotation_back()
    initial()

    print "finish"

    rospy.spin()
