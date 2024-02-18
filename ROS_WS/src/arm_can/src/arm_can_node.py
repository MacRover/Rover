#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from can_msgs.msg import Frame
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from struct import pack


def init_can_frame():
    """Generate CAN frame with default params

    Returns:
        Frame: CAN frame with extended ID
    """
    return Frame(is_extended=True, is_rtr=False, is_error=False)


def send_roll(steps):
    """Generate CAN frame for joint roll

    Args:
        steps (int): number of steps to move

    Returns:
        Frame: populated CAN frame
    """
    can_frame = init_can_frame()
    can_frame.id = 0x111
    # pack int into a signed short (16 bits), little endian
    can_frame.data = pack("<h", steps)
    can_frame.dlc = 2
    return can_frame


def send_pitch(steps):
    """Generate CAN frame for joint pitch

    Args:
        steps (int): number of steps to move

    Returns:
        Frame: populated CAN frame
    """
    can_frame = init_can_frame()
    can_frame.id = 0x222
    can_frame.data = pack("<h", steps)
    can_frame.dlc = 1
    return can_frame


def send_yaw(steps):
    """Generate CAN frame for joint yaw

    Args:
        steps (int): number of steps to move

    Returns:
        Frame: populated CAN frame
    """
    can_frame = init_can_frame()
    can_frame.id = 0x333
    can_frame.data = pack("<h", steps)
    can_frame.dlc = 1
    return can_frame


def send_lat_x(steps):
    pass


def send_lat_y(steps):
    pass


def send_lat_z(steps):
    pass


def callback(data, publisher):
    """Handle new Pose topic

    Args:
        data (Pose): new Pose value from subscribed topic
        publisher (Publisher): topic connected to CAN interface node
    """
    can_frame = Frame()

    quaternion = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    ]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)

    # Only supporting full steps of stepper motor
    roll = int(round(roll))
    pitch = int(round(pitch))
    yaw = int(round(yaw))

    # rospy.logerr("roll:" + str(roll) + " pitch:" + str(pitch) + " yaw:" + str(yaw))

    if roll != 0:
        can_frame = send_roll(roll)
        publisher.publish(can_frame)

    if pitch != 0:
        can_frame = send_pitch(pitch)
        # publisher.publish(can_frame)

    if yaw != 0:
        can_frame = send_yaw(yaw)
        # publisher.publish(can_frame)

    if int(round(data.position.x)) != 0:
        send_lat_x(round(data.position.x))

    if int(round(data.position.y)) != 0:
        send_lat_y(round(data.position.y))

    if int(round(data.position.z)) != 0:
        send_lat_z(round(data.position.z))


def main():
    rospy.init_node("arm_joint_node", anonymous=True)
    joint_name = rospy.get_param("~joint_name")
    if not joint_name:
        raise rospy.exceptions.ROSInitException("joint_name param required")

    can_pub = rospy.Publisher("/arm/can/outbound_messages", Frame, queue_size=10)
    joint_sub = rospy.Subscriber("/arm/joints/" + joint_name, Pose, callback, can_pub)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
