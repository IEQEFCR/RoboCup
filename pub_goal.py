#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    args = rospy.myargv()[1:]
    x = float(args[0])
    y = float(args[1])
    print(x, " ", y)
    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher(
        'move_base_simple/goal', PoseStamped, queue_size=1)

    mypose = PoseStamped()
    turtle_vel_pub.publish(mypose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)

    mypose = PoseStamped()
    mypose.header.frame_id = 'map'  # 设置自己的目标
    mypose.pose.position.x = x
    mypose.pose.position.y = y
    mypose.pose.position.z = 0
    mypose.pose.orientation.x = 0
    mypose.pose.orientation.y = 0
    mypose.pose.orientation.z = 0
    mypose.pose.orientation.w = 1

    turtle_vel_pub.publish(mypose)  # 发送自己设置的目标点

    time.sleep(5)
