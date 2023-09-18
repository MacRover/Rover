#!/usr/bin/env python

import rospy
import csv
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetMap
from geographic_msgs.msg import GeoPoint
from geographic_msgs.srv import GetGeoPath
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_from_euler
import subprocess

def read_csv(filename):
    points = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            lat, lon = map(float, row)
            points.append((lat, lon))
    return points

def convert_to_xy(lat, lon):
    cmd = 'rosservice call /fromLL "ll_point: {latitude: %f, longitude: %f, altitude: 0.0}"' % (lat, lon)
    result = subprocess.check_output(cmd, shell=True)
    print (result)
    lines = result.strip().split('\n')
    x_line = lines[-3]  # Line containing the x-coordinate
    y_line = lines[-2]  # Line containing the y-coordinate
    
    x = float(x_line.split(':')[1].strip())
    y = float(y_line.split(':')[1].strip())
    
    return x, y

def main():
    rospy.init_node('csv_to_markers_node')
    print("starting")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/shape', Marker, queue_size=10)
    rate = rospy.Rate(1)  # Publish once per second

    csv_filename = '/home/haashimr/science.csv'
    points = read_csv(csv_filename)
    #print(points)

    marker_id = 0
    for lat, lon in points:
        x, y = convert_to_xy(lat, lon)
       # print (str(x) + ", " + str(y) + "\n")
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "MarkerWithShape"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.0
        marker.color.a = 0.8
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = False


        if (marker_id < 3):
          marker.color.r = 0.0
          marker.color.g = 1.0
          marker.color.b = 0.0 
        else:
          marker.color.r = 1.0
          marker.color.g = 0.0
          marker.color.b = 0.0 

        pub.publish(marker)
        marker_id += 1

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

