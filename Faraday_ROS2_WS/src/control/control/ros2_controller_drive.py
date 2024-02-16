#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = None
dead_zone = 0.05
TOP_SPEED=0.7

def map_(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def callback(msg):
    global pub
    vel_msg = Twist()
    vel_msg.linear.x = map_(msg.axes[1], -1.0, 1.0, -TOP_SPEED, TOP_SPEED)

    right_stick_direction = msg.axes[3]
    # print(vel_msg.linear.x)
    if abs(right_stick_direction) < dead_zone:
        vel_msg.angular.z = 0.0
    else:
        vel_msg.angular.z = -map_(right_stick_direction, -1.0, 1.0, 0.7, -0.7)

    # press A to stop
    if (msg.buttons[0]):
        vel_msg.linear.x = 0.0


    pub.publish(vel_msg)


def main():
    global pub
    rclpy.init(args=None)
    node = Node("joyController")

    pub = node.create_publisher(Twist, "vel_state", 10)

    node.create_subscription(Joy, "joy", callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
