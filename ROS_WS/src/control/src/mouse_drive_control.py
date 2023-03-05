#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import gi
gi.require_version('Gtk', '2.0')
from gi.repository import Gtk, Gdk

rospy.init_node('car_controller2', anonymous=False)
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
is_toggled = False
size = 250

max_speed_mps = 0.5


def on_move(box, event):
	global vel_msg, is_toggled
	y_pos = size - event.y
	x_pos = size - event.x
	if abs(y_pos) <= size and abs(x_pos) <= size and is_toggled:
		vel_msg.linear.x = max_speed_mps / size * y_pos
		vel_msg.angular.z = max_speed_mps / size * x_pos
	else:
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0

	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.y = 0
	vel_msg.angular.x = 0

	publisher.publish(vel_msg)


def on_leave(box, event):
	global vel_msg
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	publisher.publish(vel_msg)

def on_click(box, event):
	global is_toggled
	is_toggled = True
	on_move(box, event)

def on_release(box, event):
	global is_toggled, vel_msg
	is_toggled = False
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	publisher.publish(vel_msg)



if __name__ == '__main__':
	window = Gtk.Window()
	box = Gtk.EventBox()
	window.set_default_size(size * 2, size * 2)
	box.connect('motion-notify-event', on_move)
	box.connect('leave-notify-event', on_leave)
	box.connect('button-press-event', on_click)
	box.connect('button-release-event', on_release)

	box.add_events(Gdk.EventMask.POINTER_MOTION_MASK)
	window.add(box)
	window.show_all()
	print('\nClick and drag at any point in the window to move the robot, hit the x button at the top right to exit')
	window.connect('destroy', Gtk.main_quit)
	Gtk.main()
