#!/usr/bin/env python
import sys
import numpy as np
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as robot_gripper

# tuck left arm - home base position
def left_tuck(left_arm):
	# create tuck pose
	tuck = PoseStamped()
	tuck.header.frame_id = "base"

	joints = [0.2972087776527989, 0.3777427690167831, -1.3510535789300782, 1.6570827461132183, 0.2841699409557729, 0.8682331259431442, -3.058757691043515]
	joint_names = left_arm.get_joints()
	joint_dict = {}
	for i in range(1,8):
		joint = joint_names[i]
		joint_dict[joint] = joints[i-1]
	try:
		left_arm.set_joint_value_target(joint_dict)
		left_plan = left_arm.plan()
		left_arm.execute(left_plan)
	except:
		print("Joint movement failed")

	# sleep for a moment
	rospy.sleep(1.0)

# move left arm to match right's tuck position, for flipping cards
def left_to_card(left_arm,x,y,z):
	# create pose
	card = PoseStamped()
	card.header.frame_id = "base"
	# x, y, and z position
	card.pose.position.x = x + 0.05
	card.pose.position.y = y - 0.025
	card.pose.position.z = z

	# Orientation as a quaternion
	card.pose.orientation.x = 0.711
	card.pose.orientation.y = 0.063
	card.pose.orientation.z = -0.024
	card.pose.orientation.w = 0.700

	# Plan and execute path from current state
	left_arm.set_pose_target(card)
	left_arm.set_start_state_to_current_state()
	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# brings the arm below the suction gripper so it is ready to raise vertically
def left_below(left_arm,x,y,z):
	# create pose
	below = PoseStamped()
	below.header.frame_id = "base"

	# x, y, and z position
	below.pose.position.x = x-0.04
	below.pose.position.y = y-0.02
	below.pose.position.z = z-0.05

	# Orientation as a quaternion
	below.pose.orientation.x = 0.015
	below.pose.orientation.y = 0.730
	below.pose.orientation.z =  -0.683
	below.pose.orientation.w = 0.004

	# Plan and execute path from current state
	left_arm.set_pose_target(below)
	left_arm.set_start_state_to_current_state()
	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# raise card up until suction gripper can pick it up again
def left_up(left_arm,x,y,z):
	# create pose
	up = PoseStamped()
	up.header.frame_id = "base"

	# x, y, and z position
	up.pose.position.x = x-0.04
	up.pose.position.y = y-0.02
	up.pose.position.z = z-0.002

	# Orientation as a quaternion
	up.pose.orientation.x = 0.015
	up.pose.orientation.y = 0.730
	up.pose.orientation.z =  -0.683
	up.pose.orientation.w = 0.004

	# Plan and execute path from current state
	left_arm.set_pose_target(up)
	left_arm.set_start_state_to_current_state()
	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# flips card over
def left_flip(left_arm):
	
	left_arm.clear_path_constraints()
	left_arm.clear_pose_targets()
	joints = left_arm.get_current_joint_values()
	joints[6] += np.pi-(3*(np.pi/180))
	left_arm.set_joint_value_target(joints)

	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	# sleep for a moment
	rospy.sleep(1.0)


def left_clear_out(left_arm):
	left_arm.clear_path_constraints()
	left_arm.clear_pose_targets()
	joints = left_arm.get_current_joint_values()
	joints[5] -= np.pi / 2
	left_arm.set_joint_value_target(joints)

	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	rospy.sleep(1.0)


def left_avoid_right(left_arm):
	left_arm.clear_path_constraints()
	left_arm.clear_pose_targets()
	joints = left_arm.get_current_joint_values()
	joints[5] += .2
	left_arm.set_joint_value_target(joints)

	left_plan = left_arm.plan()
	left_arm.execute(left_plan)

	rospy.sleep(1.0)