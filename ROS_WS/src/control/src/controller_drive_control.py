#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

pub = rospy.Publisher("/vel_state", Twist, queue_size=10)
dead_zone = 0.05
TOP_SPEED=0.7

def map_(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def callback(data):
    # print(data)
    vel_msg = Twist()
    #vel_msg.linear.x = map_(data.axes[5], -1.0, 1.0, TOP_SPEED, 0.0) - map_(
    #    data.axes[4], -1.0, 1.0, TOP_SPEED, 0.0
    #)
    vel_msg.linear.x = map_(data.axes[1], -1.0, 1.0, -TOP_SPEED, TOP_SPEED)

    right_stick_direction = data.axes[2]
    print(vel_msg.linear.x)
    if right_stick_direction < dead_zone and right_stick_direction > -1.0 * dead_zone:
        vel_msg.angular.z = 0
    else:
        vel_msg.angular.z = map_(right_stick_direction, -1.0, 1.0, TOP_SPEED, -TOP_SPEED)
    global pub

    # press A to stop
    if (data.buttons[0]):
        vel_msg.linear.x = 0


    pub.publish(vel_msg)

    # Intializes everything


def start():
    rospy.init_node("joyToDrive", anonymous=False)

    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == "__main__":
    start()
