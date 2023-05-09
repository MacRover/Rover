#!/usr/bin/env python
import rospy
import time
import sys
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
import qwiic_icm20948

# helper methods
def RawToMeterPerSecond(g):
    return (g / 16.384) * 1000.0 * 9.80665

def RawToRadsPerSecond(dps):
    return (dps / 131.0) * 0.01745

def RawToTesla(mT):
    return (mT * 0.15)

def icm20948_node():

    # Initialize ROS node
    raw_pub = rospy.Publisher('icm20948/raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('icm20948/mag', MagneticField, queue_size=10)
    rospy.init_node('icm20948')

    rate = rospy.Rate(100)
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node launched.")

    IMU = qwiic_icm20948.QwiicIcm20948()

    while IMU.connected == False:
        message = "The Qwiic ICM20948 device isn't connected to the system. Please check your connection"
        rospy.loginfo(message)
        return

    IMU.begin()    

    while not rospy.is_shutdown():
        if IMU.dataReady():
            IMU.getAgmt()
            raw_msg = Imu()
            raw_msg.header.stamp = rospy.Time.now()
	            
            raw_msg.orientation.w = 0
            raw_msg.orientation.x = 0
            raw_msg.orientation.y = 0
            raw_msg.orientation.z = 0
                
            raw_msg.linear_acceleration.x = RawToMeterPerSecond(IMU.axRaw)
            raw_msg.linear_acceleration.y = RawToMeterPerSecond(IMU.ayRaw)
            raw_msg.linear_acceleration.z = RawToMeterPerSecond(IMU.azRaw)
                
            raw_msg.angular_velocity.x = RawToRadsPerSecond(IMU.gxRaw)
            raw_msg.angular_velocity.y = RawToRadsPerSecond(IMU.gyRaw)
            raw_msg.angular_velocity.z = RawToRadsPerSecond(IMU.gzRaw)
                
            raw_msg.orientation_covariance[0] = -1
            raw_msg.linear_acceleration_covariance[0] = -1
            raw_msg.angular_velocity_covariance[0] = -1
                
            raw_pub.publish(raw_msg)
            

            mag_msg = MagneticField()
            mag_msg.header.stamp = rospy.Time.now()
            
            # switch X/Y and invert Z to convert from NED to ENU
            mag_msg.magnetic_field.x = RawToTesla(IMU.myRaw)
            mag_msg.magnetic_field.y = RawToTesla(IMU.mxRaw)
            mag_msg.magnetic_field.z = RawToTesla(-IMU.mzRaw)
            mag_msg.magnetic_field_covariance[0] = -1
            mag_pub.publish(mag_msg)

            rospy.spin()
            rate.sleep()   
    
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
    try:
        icm20948_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node exited with exception.")
