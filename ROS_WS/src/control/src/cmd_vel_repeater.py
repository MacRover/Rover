#! /usr/bin/env python
import rospy
import sys, termios, tty, os, time
from geometry_msgs.msg import Twist

vel_msg = Twist()


def move():
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    publisher.publish(vel_msg)


def callback(data):
    global vel_msg
    vel_msg = data


def start():
    rospy.init_node("heartbeat_cmd_vel_repeater", anonymous=False)
    rospy.Subscriber("vel_state", Twist, callback)
    publish_rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        move()
        publish_rate.sleep()


if __name__ == "__main__":
    try:
        start()
    except rospy.ROSInterruptException:
        pass
