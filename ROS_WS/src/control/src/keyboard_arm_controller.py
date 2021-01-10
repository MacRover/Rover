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
    rospy.init_node("keyboard_arm_controller", anonymous=True)

    base_hinge_position_publisher = rospy.Publisher('/base_hinge_position_controller/command', Float64, queue_size=10)
    lower_hinge_position_publisher = rospy.Publisher('/lower_hinge_position_controller/command', Float64, queue_size=10)
    upper_hinge_position_publisher = rospy.Publisher('/upper_hinge_position_controller/command', Float64, queue_size=10)
    forearm_hinge_position_publisher = rospy.Publisher('/forearm_hinge_position_controller/command', Float64, queue_size=10)
    wrist_hinge_position_publisher = rospy.Publisher('/wrist_hinge_position_controller/command', Float64, queue_size=10)
    claw_right_hinge_position_publisher = rospy.Publisher('/claw_right_hinge_position_controller/command', Float64, queue_size=10)
    claw_left_hinge_position_publisher = rospy.Publisher('/claw_left_hinge_position_controller/command', Float64, queue_size=10)

    angle_base_hinge = Float64()
    angle_lower_hinge = Float64()
    angle_upper_hinge = Float64()
    angle_forearm_hinge = Float64()
    angle_wrist_hinge = Float64()
    angle_claw_right_hinge = Float64()
    angle_claw_left_hinge = Float64()

    angle_base_hinge.data = 0.0
    angle_lower_hinge.data = 0.0
    angle_upper_hinge.data = 0.0
    angle_forearm_hinge.data = 0.0
    angle_wrist_hinge.data = 0.0
    angle_claw_right_hinge.data = 0.0
    angle_claw_left_hinge.data = 0.0

    rate = rospy.Rate(5)

    rate.sleep()
    base_hinge_position_publisher.publish(angle_base_hinge)
    rate.sleep()
    lower_hinge_position_publisher.publish(angle_lower_hinge)
    rate.sleep()
    upper_hinge_position_publisher.publish(angle_upper_hinge)
    rate.sleep()
    forearm_hinge_position_publisher.publish(angle_forearm_hinge)
    rate.sleep()
    wrist_hinge_position_publisher.publish(angle_wrist_hinge)
    rate.sleep()
    claw_right_hinge_position_publisher.publish(angle_claw_right_hinge)
    rate.sleep()
    claw_left_hinge_position_publisher.publish(angle_claw_left_hinge)
    rate.sleep()

    print("These are the keys used to control the arm:\
      W, S for the upper arm,\n\
      A, D for the base hinge,\n\
      T, G for the forearm,\n\
      F, H for the wrist,\n\
      I, K for the claw base,\n\
      J, L for the claws,\n\
    ")

    while True:
        char = getch()
        if ord(char)==27:
            print("Exiting")
            exit(0)
        elif char=="w":
            angle_lower_hinge.data += 0.01
            if angle_lower_hinge.data >= 2:
              angle_lower_hinge.data = 2
            lower_hinge_position_publisher.publish(angle_lower_hinge)
            print("Moved upper arm to "+ str(angle_lower_hinge.data) +" rads")
        elif char=="s":
            angle_lower_hinge.data -= 0.01
            if angle_lower_hinge.data <= -2:
              angle_lower_hinge.data = -2
            lower_hinge_position_publisher.publish(angle_lower_hinge)
            print("Moved upper arm to "+ str(angle_lower_hinge.data) +" rads")
        elif char=="d":
            angle_base_hinge.data -= 0.01
            if angle_base_hinge.data <= -6.28:
              angle_base_hinge.data = -6.28
            base_hinge_position_publisher.publish(angle_base_hinge)
            print("Moved base to "+ str(angle_base_hinge.data) +" rads")
        elif char=="a":
            angle_base_hinge.data += 0.01
            if angle_base_hinge.data >= 6.28:
              angle_base_hinge.data = 6.28
            base_hinge_position_publisher.publish(angle_base_hinge)
            print("Moved base to "+ str(angle_base_hinge.data) +" rads")
        elif char=="t":
            angle_upper_hinge.data += 0.01
            if angle_upper_hinge.data >= 3:
              angle_upper_hinge.data = 3
            upper_hinge_position_publisher.publish(angle_upper_hinge)
            print("Moved forearm to "+ str(angle_upper_hinge.data) +" rads")
        elif char=="g":
            angle_upper_hinge.data -= 0.01
            if angle_upper_hinge.data <= -3:
              angle_upper_hinge.data = -3
            upper_hinge_position_publisher.publish(angle_upper_hinge)
            print("Moved forearm to "+ str(angle_upper_hinge.data) +" rads")
        elif char=="h":
            angle_forearm_hinge.data += 0.01
            if angle_forearm_hinge.data >= 1.57:
              angle_forearm_hinge.data = 1.57
            forearm_hinge_position_publisher.publish(angle_forearm_hinge)
            print("Moved wrist to "+ str(angle_forearm_hinge.data) +" rads")
        elif char=="f":
            angle_forearm_hinge.data -= 0.01
            if angle_forearm_hinge.data <= -1.57:
              angle_forearm_hinge.data = -1.57
            forearm_hinge_position_publisher.publish(angle_forearm_hinge)
            print("Moved wrist to "+ str(angle_forearm_hinge.data) +" rads")
        elif char=="i":
            angle_wrist_hinge.data += 0.01
            if angle_wrist_hinge.data >= 6.28:
              angle_wrist_hinge.data = 6.28
            wrist_hinge_position_publisher.publish(angle_wrist_hinge)
            print("Moved claw base to "+ str(angle_wrist_hinge.data) +" rads")
        elif char=="k":
            angle_wrist_hinge.data -= 0.01
            if angle_wrist_hinge.data <= -6.28:
              angle_wrist_hinge.data = -6.28
            wrist_hinge_position_publisher.publish(angle_wrist_hinge)
            print("Moved claw base to "+ str(angle_wrist_hinge.data) +" rads")
        elif char=="l":
            angle_claw_right_hinge.data += 0.01
            angle_claw_left_hinge.data += 0.01
            if angle_claw_right_hinge.data >= 1.57:
              angle_claw_right_hinge.data = 1.57
              angle_claw_left_hinge.data = 1.57
            claw_right_hinge_position_publisher.publish(angle_claw_right_hinge)
            claw_left_hinge_position_publisher.publish(angle_claw_left_hinge)
            print("Moved claws to "+ str(angle_claw_right_hinge.data) +" rads")
        elif char=="j":
            angle_claw_right_hinge.data -= 0.01
            angle_claw_left_hinge.data -= 0.01
            if angle_claw_right_hinge.data <= 0:
              angle_claw_right_hinge.data = 0
              angle_claw_left_hinge.data = 0
            claw_right_hinge_position_publisher.publish(angle_claw_right_hinge)
            claw_left_hinge_position_publisher.publish(angle_claw_left_hinge)
            print("Moved claws to "+ str(angle_claw_right_hinge.data) +" rads")
        # elif ord(char)==32:
        #     print("Stopping")
        #     vel_msg.linear.x = 0
        #     vel_msg.angular.z = 0

        

if __name__== '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    
