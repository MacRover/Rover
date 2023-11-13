#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, os, time

def getch():  # this function is used to get key input from user in the terminal
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)

	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch

def main():
	rclpy.init(args=None)
	node = Node('controller')
	publisher = node.create_publisher(Twist, "vel_state", 10)
	vel_msg = Twist()

	print(
        	"Press w, a, s, d accordingly to move; space for stopping and Esckey for exiting. Be careful not to hold these keys, as these keys control the acceleration of the bot, and u dont want the bot flying out!!"
	)
	while True:
		char = getch()
		if ord(char) == 27:
			print("Exiting")
			exit(0)
		elif char == "w":
			print("Accelerated")
			vel_msg.linear.x += 0.008
		elif char == "s":
			print("Accelerated the opposite direction")
			vel_msg.linear.x -= 0.008
		elif char == "d":
			print("Turning right")
			vel_msg.angular.z -= 0.008
		elif char == "a":
			print("Turning left")
			vel_msg.angular.z += 0.008
		elif ord(char) == 32:
			print("Stopping")
			vel_msg.linear.x = 0.0
			vel_msg.angular.z = 0.0

		print("x vel: {}\tz vel: {}".format(vel_msg.linear.x, vel_msg.angular.z))

		vel_msg.linear.y = 0.0
		vel_msg.linear.z = 0.0
		vel_msg.angular.y = 0.0
		vel_msg.angular.x = 0.0
		publisher.publish(vel_msg)

	rclpy.shutdown()


if __name__ == "__name__":
        main()
