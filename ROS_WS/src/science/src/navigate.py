#!/usr/bin/env python

import rospy
import subprocess
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionGoal

def convert_to_xy(lat, lon):
    cmd = 'rosservice call /fromLL "ll_point: {latitude: %f, longitude: %f, altitude: 0.0}"' % (lat, lon)
    result = subprocess.check_output(cmd, shell=True)
    
    lines = result.strip().split('\n')
    x_line = lines[-3]  # Line containing the x-coordinate
    y_line = lines[-2]  # Line containing the y-coordinate
    
    x = float(x_line.split(':')[1].strip())
    y = float(y_line.split(':')[1].strip())
    
    return x, y

def gps_callback(data):
    print("Recieved GPS Coords:")
   # print(data)

    current_lat = data.latitude
    current_lon = data.longitude
    goal_lat = 51.47109
    goal_lon = -112.75290

    goal_x, goal_y = convert_to_xy(goal_lat, goal_lon)
    current_x, current_y = convert_to_xy(current_lat, current_lon)

    
    x_delta = goal_x - current_x
    y_delta = goal_y - current_y

    print (x_delta, y_delta)

    # Create and publish a MoveBaseActionGoal message with the calculated delta values
    goal_msg = MoveBaseActionGoal()
    goal_msg.goal.target_pose.header.frame_id = "ekf_odom"
    goal_msg.goal.target_pose.pose.position.x = x_delta
    goal_msg.goal.target_pose.pose.position.y = y_delta
    goal_msg.goal.target_pose.pose.orientation.w = 1.0

    goal_publisher.publish(goal_msg)

if __name__ == '__main__':
    rospy.init_node('gps_to_move_base_node')

    goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    gps_subscriber = rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)
    # test()
    rospy.spin()
