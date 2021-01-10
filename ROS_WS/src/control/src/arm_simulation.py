#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
from nav_msgs.msg import Odometry
import tf_conversions

# 0.122345676898
# -0.787387101292
# 0.587809201096
# 0.139762051967
# -0.0607888543433
# 3.75616821435
# 0.0354568710207

moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)

robot = moveit_commander.robot.RobotCommander()
arm_group = moveit_commander.move_group.MoveGroupCommander("rover_arm")

arm_group.set_pose_reference_frame("/arm_base_link")

arm_group.set_goal_orientation_tolerance(0.005)
arm_group.set_goal_position_tolerance(0.005)

# 0.625274468212
# -4.67942309331e-05
# 0.780404918469
# -2.10244116216e-05
# 0.887728858925
# 1.44993796435e-05
# -0.265716655232
# (-3.141173814561994, 1.3509663476108178, -3.1411371492746563)

q = tf_conversions.transformations.quaternion_from_euler(-3.1410900381062, 1.4846899032278735, -3.141102327247691)
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = 0.888
pose_target.position.y = 0
pose_target.position.z = -0.265
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
print(q)

# q = tf_conversions.transformations.quaternion_from_euler(1.57, -1.57, 3.14)

# current_pose = arm_group.get_current_pose()
# print(arm_group.get_goal_position_tolerance())
# print(current_pose.header.frame_id)
# print(current_pose.pose.orientation.w)
# print(current_pose.pose.orientation.x)
# print(current_pose.pose.orientation.y)
# print(current_pose.pose.orientation.z)
# print(current_pose.pose.position.x)
# print(current_pose.pose.position.y)
# print(current_pose.pose.position.z)
# print(tf_conversions.transformations.euler_from_quaternion([current_pose.pose.orientation.x, 
#                                                             current_pose.pose.orientation.y, 
#                                                             current_pose.pose.orientation.z, 
#                                                             current_pose.pose.orientation.w]))
rospy.sleep(5)
moveit_commander.roscpp_initializer.roscpp_shutdown()


# def move():
#   moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
#   rospy.init_node('move_group_grasp', anonymous=True)
#   rospy.Subscriber("/odom", Odometry, callback)
#   rospy.spin()
# def callback(data):
#   robot = moveit_commander.robot.RobotCommander()

#   arm_group = moveit_commander.move_group.MoveGroupCommander("rover_arm")
#   # hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
#   # arm_group.set_named_target("default")
#   # plan = arm_group.go()
#   # hand_group.set_named_target("open")
#   # plan = hand_group.go()
#   # chassis_x = data.pose.pose.x
#   # chassis_y = data.pose.pose.y
#   chassis_x = data.pose.pose.position.x
#   print(chassis_x)
#   arm_group.set_pose_reference_frame("/odom")
#   pose_target = geometry_msgs.msg.Pose()
#   pose_target.orientation.w = 0.5
#   pose_target.orientation.x = -0.5
#   pose_target.orientation.y = 0.5
#   pose_target.orientation.z = -0.5
#   pose_target.position.x = 0
#   pose_target.position.y = 0.5
#   pose_target.position.z = 0.5
#   arm_group.set_pose_target(pose_target)
#   plan = arm_group.go()

#   # pose_target.position.z = 1.08
#   # arm_group.set_pose_target(pose_target)
#   # plan = arm_group.go()

#   # hand_group.set_named_target("close")
#   # plan = hand_group.go()

#   # pose_target.position.z = 1.5
#   # arm_group.set_pose_target(pose_target)
#   # plan = arm_group.go()

#   # hand_group.set_named_target("open")
#   # plan = hand_group.go()
#   rospy.sleep(5)
#   moveit_commander.roscpp_initializer.roscpp_shutdown()
# if __name__== '__main__':
# 	try:
# 		  move()
# 	except rospy.ROSInterruptException:
# 		  pass