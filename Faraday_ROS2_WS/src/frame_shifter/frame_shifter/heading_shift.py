import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('shifter_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def broadcast_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        # Assuming rotation angle is 45 degrees (in radians)
        rotation_angle = math.pi / 4.0
        transform.transform.rotation.z = math.sin(rotation_angle / 2)
        transform.transform.rotation.w = math.cos(rotation_angle / 2)

        # Publish the transformation
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    try:
        while rclpy.ok():
            node.broadcast_transform()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
