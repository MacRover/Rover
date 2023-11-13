#! /usr/bin/env python
import rclpy
import sys, termios, tty, os, time
from geometry_msgs.msg import Twist

vel_msg = Twist()

def callback(msg):
    global vel_msg
    vel_msg = msg


def main():
    rclpy.init(args=None)
    node = rclpy.create_node("heartbeat_cmd_vel_repeater")
    publisher = node.create_publisher(Twist, "cmd_vel", 10)
    node.create_subscription(Twist, "vel_state", callback, 10)
    publish_rate = node.create_rate(25.0)

    while rclpy.ok():
        publisher.publish(vel_msg)
        rclpy.spin_once(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
