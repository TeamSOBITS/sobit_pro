#!/usr/bin/env python3
# coding: utf-8

import rospy
import time
import math
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sobit_common_msg.srv import wheel_control
from tf.transformations import euler_from_quaternion

odometry_value = Odometry() # 初期位置
Kp = Float64()
Kv = Float64()
Ki = Float64()
straight_flag = False
trun_flag = False

# サービスでの要求を読み取る
def request(req):
    global odometry_value, straight_flag, turn_flag

    t1 = time.time() # 処理前の時刻
    speed = Twist()
    rate = rospy.Rate(20)
    xt = 0.0

    initial_value = odometry_value # 初期の位置を保存する

    # 並進運動
    if req.straight_line != 0.0 and req.turn_angle == 0.0:
        rospy.loginfo("order_Straight")

        ft_before = Kp * req.straight_line # 速さ(PID制御で使用)

        while xt < abs(req.straight_line):

            straight_flag = True
            result = pid_calculation(abs(req.straight_line), xt, t1, ft_before) # PID制御の計算を行う
            straight_flag = False

            if req.straight_line < 0:
                speed.linear.x = -result
            else:
                speed.linear.x = result

            pub.publish(speed) # トピックを用いて指定の速さを送る

            ft_before = result

            x_diff = odometry_value.pose.pose.position.x - initial_value.pose.pose.position.x
            y_diff = odometry_value.pose.pose.position.y - initial_value.pose.pose.position.y
            xt = math.sqrt(x_diff ** 2 + y_diff ** 2) # ユークリッド距離の計算

            #rospy.loginfo("%s[m] %s[m]", xt, req.straight_line)
            rate.sleep()

    # 回転運動
    elif req.straight_line == 0.0 and req.turn_angle != 0.0:
        rospy.loginfo("order_Trun")

        n = 1
        order_value = math.radians(req.turn_angle) # 要求された角度を度数法から弧度法に変換

        # 初期位置を計算できる角度に変換
        initial_euler = tf.transformations.euler_from_quaternion((initial_value.pose.pose.orientation.x,initial_value.pose.pose.orientation.y,initial_value.pose.pose.orientation.z,initial_value.pose.pose.orientation.w))

        ft_before = Kp * order_value # 速さ(PID制御で使用)

        # 制御
        while xt < abs(order_value):

            turn_flag = True
            result = pid_calculation(abs(order_value), xt, t1, ft_before) #PID制御の計算を行う
            turn_flag = False

            if order_value < 0:
                speed.angular.z = -result
            else:
                speed.angular.z = result

            pub.publish(speed) # トピックを用いて指定の速さを送る

            ft_before = result

            # オドメトリを計算できる角度に変換
            odometry_euler = tf.transformations.euler_from_quaternion((odometry_value.pose.pose.orientation.x,odometry_value.pose.pose.orientation.y,odometry_value.pose.pose.orientation.z,odometry_value.pose.pose.orientation.w))

            before = xt

            if(-0.00314 < odometry_euler[2] - initial_euler[2] and odometry_euler[2] - initial_euler[2] < 0 and 0 < order_value):
                continue;
            elif(0 < odometry_euler[2] - initial_euler[2] and odometry_euler[2] - initial_euler[2] < 0.00314 and order_value < 0):
                continue;

            # コサイン類似度の計算
            if odometry_euler[2] - initial_euler[2] < 0 and 0 < order_value:
                xt = abs(odometry_euler[2] - initial_euler[2] + math.radians(360 * n))
            elif 0 < odometry_euler[2] - initial_euler[2] and order_value < 0:
                xt = abs(odometry_euler[2] - initial_euler[2] - math.radians(360 * n))
            elif 0 < order_value:
                xt = abs(odometry_euler[2] - initial_euler[2] + math.radians(360 * (n-1)))
            elif order_value < 0:
                xt = abs(odometry_euler[2] - initial_euler[2] - math.radians(360 * (n-1)))

            if math.degrees(xt) < (math.degrees(before)-0.0314):
                n += 1
                if 0 < order_value:
                    xt = abs(odometry_euler[2] - initial_euler[2] + math.radians(360 * (n-1)))
                elif order_value < 0:

                    xt = abs(odometry_euler[2] - initial_euler[2] - math.radians(360 * (n-1)))

            #rospy.loginfo("%s[deg] %s[deg]", math.degrees(xt), req.turn_angle)
            rate.sleep()

    xt = 0.0
    return 'finished'

# PID制御の計算を行う
def pid_calculation(xd, xt, t1, ft_before):
    t2 = time.time() # 処理後の時刻
    elapsed_time = t2-t1 # 経過時間

    #並進運動
    if straight_flag == True:
        if xd <= 0.1:
            ft = Kp * (xd + 0.001 - xt) - Kv * ft_before + Ki / 0.8 * (xd + 0.001 - xt) * elapsed_time ** 2
        else:
            ft = Kp * (xd + 0.001 - xt) - Kv * ft_before + Ki / 8 / xd * (xd + 0.001 - xt) * elapsed_time ** 2

    #回転運動
    elif turn_flag == True:
        if math.degrees(xd) <= 30:
            ft = Kp * (xd + 0.001 - xt) - Kv * ft_before + Ki * (xd + 0.001 - xt) * elapsed_time ** 2
        else:
            ft = Kp * (xd + 0.001 - xt) - Kv * ft_before + Ki * 0.75 * 30 / math.degrees(xd) * (xd + 0.001 - xt) * elapsed_time ** 2

    return ft

# 車輪のオドメトリを返す
def odometory_save(odometry):
    global odometry_value
    odometry_value = odometry

# メイン
if __name__ == '__main__':
    rospy.init_node('wheel_control')


    sub = rospy.Subscriber('/odom', Odometry, odometory_save)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 20)
    rospy.Service('wheel_control', wheel_control, request)

    # パラメータの設定
    Kp = rospy.get_param("/proportional_control", 0.1)
    Kv = rospy.get_param("/derivation_control", 0.4)
    Ki = rospy.get_param("/integral_control", 0.8)

    print ("Ready to serve")
    rospy.spin()
