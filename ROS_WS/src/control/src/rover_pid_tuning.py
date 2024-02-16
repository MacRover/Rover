#! /usr/bin/env python
import rospy
import sys, termios, tty, os, time
from std_msgs.msg import Float64MultiArray


def getch():  # this function is used to get key input from user in the terminal
    return raw_input("Enter the Motor to tune, kP, kI, and kD. Separate all with a space, type x to exit: ")


button_delay = 0.2


def tune():
    rospy.init_node("motor_pid_tuner", anonymous=False)
    publisher = rospy.Publisher("/cmd_pid", Float64MultiArray, queue_size=10)
    pid_msg = Float64MultiArray()
    
    
    while True:
    	array_to_pub = []
        char = getch()
        charArray = char.split(' ')
        if charArray[0] == "x":
            exit(0)
        elif len(charArray) != 4:
            print("Incorrect Input, try again ")
            continue
        
        for i in range(4):
            array_to_pub.append(float(charArray[i]))


	pid_msg.data = array_to_pub
            
        

        print("Motor: {}\tkP: {}\tkI: {}\tkD: {}".format(pid_msg.data[0], pid_msg.data[1], pid_msg.data[2], pid_msg.data[3]))


        publisher.publish(pid_msg)


if __name__ == "__main__":
    try:
        tune()
    except rospy.ROSInterruptException:
        pass
