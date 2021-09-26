#! /usr/bin/env python
import rospy
import sys, termios, tty, os, time
from std_msgs.msg import Float64

def getch(): #this function is used to get key input from user in the terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
button_delay = 0.2



def move():
    rospy.init_node("keyboard_rover_controller", anonymous=False)
    left_wheel_velocity_controller = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
    front_left_wheel_velocity_controller = rospy.Publisher('/front_left_wheel_velocity_controller/command', Float64, queue_size=10)
    right_wheel_velocity_controller = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
    front_right_wheel_velocity_controller = rospy.Publisher('/front_right_wheel_velocity_controller/command', Float64, queue_size=10)

    x, rotation = 0.0, 0.0

    print("Press w, a, s, d accordingly to move; space for stopping and Esckey for exiting. Be careful not to hold these keys, as these keys control the acceleration of the bot, and u dont want the bot flying out!!")
    while True:
        char = getch()
        if ord(char)==27:
            print("Exiting")
            exit(0)
        elif char=="w":
            print("Accelerated")
            x +=0.1
        elif char=="s":
            print("Accelerated the opposite direction")
            x -=0.1
        elif char=="d":
            print("Turning right")
            rotation +=0.1
        elif char=="a":
            print("Turning left")
            rotation -=0.1
        elif ord(char)==32:
            print("Stopping")
            x = 0
            rotation = 0

        front_left_wheel_velocity_controller.publish(x)
        front_right_wheel_velocity_controller.publish(x)
        right_wheel_velocity_controller.publish(x)
        left_wheel_velocity_controller.publish(x)

if __name__== '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    

