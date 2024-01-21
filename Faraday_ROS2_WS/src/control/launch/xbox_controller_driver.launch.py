from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joystick_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_node"
    )

    heartbeat = Node(
        package="control",
        executable="cmd_repeater",
        name="heartbeat"
    )

    xbox_drive = Node(
        package="control",
        executable="controller_drive",
        name="xbox_drive"
    )

    return LaunchDescription([
        joystick_node,
        heartbeat,
        xbox_drive
    ])