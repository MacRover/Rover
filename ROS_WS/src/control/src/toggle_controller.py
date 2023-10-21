#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


pub = rospy.Publisher("/controller_mode", Int16MultiArray, queue_size=10)

# A B X Y buttons used
buttons_active = Int16MultiArray()
buttons_active.data = [1, 0, 0, 0]
last_idx = 0

def callback(data):
    global buttons_active, last_idx, pub
    
    # no button presses
    if not any(data.buttons):
        return
    
    buttons_active.data[last_idx] = 0

    if data.buttons[0]:
        buttons_active.data[0] = 1
        last_idx = 0
    elif data.buttons[1]:
        buttons_active.data[1] = 1
        last_idx = 1
    elif data.buttons[2]:
        buttons_active.data[2] = 1
        last_idx = 2
    elif data.buttons[3]:
        buttons_active.data[3] = 1
        last_idx = 3
    
    pub.publish(buttons_active)


def main():
    global buttons_active
    rospy.init_node("controller_button_toggler", anonymous=False)
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass