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

max_speed = 0.1


def on_move(box, event):
	global vel_msg, is_toggled
	box.window.invalidate_rect(None, False)
	while Gtk.events_pending():
		Gtk.main_iteration()
	cr = box.window.cairo_create()
	cr.set_source_rgb(1.0, 0.0, 0.0)
	cr.move_to(size, size)
	cr.line_to(event.x, event.y)
	cr.stroke()

	y_pos = size - event.y
	x_pos = size - event.x
	if abs(y_pos) <= size and abs(x_pos) <= size and is_toggled:
		vel_msg.linear.x = max_speed / size * y_pos
		vel_msg.angular.z = max_speed / size * x_pos
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
	darea = Gtk.DrawingArea()
	window.set_default_size(size * 2, size * 2)
	darea.connect('motion-notify-event', on_move)
	darea.connect('leave-notify-event', on_leave)
	darea.connect('button-press-event', on_click)
	darea.connect('button-release-event', on_release)

	darea.add_events(
		Gdk.EventMask.POINTER_MOTION_MASK |
		Gdk.EventMask.BUTTON_RELEASE_MASK |
		Gdk.EventMask.BUTTON_PRESS_MASK
	)
	window.add(darea)
	window.show_all()
	print('\nClick and drag at any point in the window to move the robot, hit the x button at the top right to exit')
	window.connect('destroy', Gtk.main_quit)
	Gtk.main()
