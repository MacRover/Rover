#! /usr/bin/env python
import rclpy
import threading
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException

vel_msg = Twist()

def callback(msg):
    global vel_msg
    vel_msg = msg


def main():
    rclpy.init(args=None)
    node = rclpy.create_node("heartbeat_cmd_vel_repeater")

    publisher = node.create_publisher(Twist, "cmd_vel", 10)
    node.create_subscription(Twist, "vel_state", callback, 10)
    publish_rate = node.create_rate(5.0)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    while rclpy.ok():
        try:
            publisher.publish(vel_msg)
            # rclpy.spin_once(node)
            publish_rate.sleep()
        except (KeyboardInterrupt, ExternalShutdownException):
            break

    rclpy.try_shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
