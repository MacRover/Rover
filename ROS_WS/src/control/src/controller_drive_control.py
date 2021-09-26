#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
dead_zone = 0.05

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

def callback(data):
    # print(data)
    vel_msg = Twist()
    vel_msg.linear.x = valmap(data.axes[5], -1.0, 1.0, 4.0, 0.0) - valmap(data.axes[2], -1.0, 1.0, 4.0, 0.0)

    right_stick_direction = data.axes[0]
    print(right_stick_direction)
    if right_stick_direction < dead_zone and right_stick_direction > -1.0 * dead_zone :
        vel_msg.angular.z = 0
    else:
        vel_msg.angular.z = right_stick_direction
    global pub
    pub.publish(vel_msg)

    # Intializes everything
def start():
    rospy.init_node('joyToDrive', anonymous=False)

    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    start()