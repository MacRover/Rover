#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    print("=== Starting tf_braodcaster ===")

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf_0 = TransformStamped()


    while not rospy.is_shutdown():
        try:
            static_tf_0.header.stamp = rospy.Time.now()
            static_tf_0.header.frame_id = "t265_pose_frame"
            static_tf_0.child_frame_id = "fake_base_footprint"
        
            static_tf_0.transform.translation.x = float('-0.32')
            static_tf_0.transform.translation.y = float('-0.00')
            static_tf_0.transform.translation.z = float('-0.16')
        
            quat = tf.transformations.quaternion_from_euler(float('0.00'),float('0.00'),float('0.00'))
            static_tf_0.transform.rotation.x = quat[0]
            static_tf_0.transform.rotation.y = quat[1]
            static_tf_0.transform.rotation.z = quat[2]
            static_tf_0.transform.rotation.w = quat[3]
            broadcaster.sendTransform([static_tf_0])
        
        except rospy.ROSInterruptException:
            pass
        
        rospy.spin()