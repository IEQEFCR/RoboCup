#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time  # 引入time库
from adafruit_servokit import ServoKit  # 引入PCA9685库
# 明确PCA9685的舵机控制数
state = '0'  # 货舱状态标示符
cnt = 0


def dropstart(state):  # 投放
    if (state == '0'):  # 货舱全关
        kit.servo[0].angle = 0
        kit.servo[1].angle = 92
        kit.servo[2].angle = 0
        time.sleep(1)

    elif (state == '1'):  # 仅货舱1开

        kit.servo[0].angle = 115
        kit.servo[1].angle = 92
        kit.servo[2].angle = 0
        time.sleep(1)

    elif (state == '2'):  # 仅货舱2开
        kit.servo[0].angle = 0
        kit.servo[1].angle = 0
        kit.servo[2].angle = 0
        time.sleep(1)

    elif (state == '3'):  # 仅货舱3开
        kit.servo[0].angle = 0
        kit.servo[1].angle = 92
        kit.servo[2].angle = 115
        time.sleep(1)

    elif (state == '4'):  # 货舱全开
        kit.servo[0].angle = 115
        kit.servo[1].angle = 0
        kit.servo[2].angle = 115
        time.sleep(1)


def callback(cb_msg):
    cnt = cnt + 1
    if cnt == 1:
        dropstart('1')
    if cnt == 2:
        dropstart('2')
    if cnt == 3:
        dropstart('3')


if __name__ == "__main__":

    kit = ServoKit(channels=16)
    dropstart('0')
    time.sleep(10)
    rospy.init_node("wuhu")
    sub = rospy.Subscriber("drop", String, callback)
    rospy.loginfo('init completed!')
    rospy.spin()
