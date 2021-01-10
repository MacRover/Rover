#! /usr/bin/env python
import rospy
import sys, termios, tty, os, time
from geometry_msgs.msg import Twist

publisher = None


def callback(data):
	publisher.publish(data)

def move():
	global publisher
	rospy.init_node("car_controller", anonymous=False)
	publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("/planner/cmd_vel", Twist, callback)

	rospy.spin()

if __name__== '__main__':
	try:
		  move()
	except rospy.ROSInterruptException:
		  pass
    

