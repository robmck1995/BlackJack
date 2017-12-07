#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as robot_gripper
import numpy as np
import baxter_interface

# global variables
dealer_cards = 0
player_cards = 0
deck_x = 0.545
deck_y = -0.497
deck_z = -0.158+0.003175

# tuck the right arm back to center, home base position
def right_tuck(right_arm):
	# create tuck pose
	tuck = PoseStamped()
	tuck.header.frame_id = "base"

	# x, y, and z position
	joints = [0.21053886313727305, -1.0411894597772247, 1.0461748973378522, 1.8971507394172855, -0.5361262853659521, 1.0250826615044277, 0.7063981528212331]
	joint_names = right_arm.get_joints()
	joint_dict = {}
	for i in range(1,8):
		joint = joint_names[i]
		joint_dict[joint] = joints[i-1]
	right_arm.set_joint_value_target(joint_dict)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# shift right arm in the positive x direction so left gripper can return the card to the suction gripper 
def right_shift(right_arm,x,y,z):
	# create pose
	shift = PoseStamped()
	shift.header.frame_id = "base"

	# x, y, and z position
	shift.pose.position.x = x + 0.1
	shift.pose.position.y = y
	shift.pose.position.z = z

	# Orientation as a quaternion - straight down
	shift.pose.orientation.x = 0.0
	shift.pose.orientation.y = -1.0
	shift.pose.orientation.z = 0.0
	shift.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(shift)
	right_arm.set_start_state_to_current_state()
	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# hover over the deck of cards, so that when moving down we don't mess up deck
def right_hover_deck(right_arm):
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above deck of cards
	hover = PoseStamped()
	hover.header.frame_id = "base"

	# x, y, and z position of deck
	hover.pose.position.x = deck_x
	hover.pose.position.y = deck_y
	hover.pose.position.z = deck_z + .1

	# Orientation as a quaternion - straight down
	hover.pose.orientation.x = 0.0
	hover.pose.orientation.y = -1.0
	hover.pose.orientation.z = 0.0
	hover.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(hover)
	right_arm.set_start_state_to_current_state()
	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# move right gripper to the deck of cards
def right_to_deck(right_arm, right_gripper):
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above deck of cards
	deck = PoseStamped()
	deck.header.frame_id = "base"

	# x, y, and z position of deck
	deck.pose.position.x = deck_x
	deck.pose.position.y = deck_y
	deck.pose.position.z = deck_z

	# Orientation as a quaternion - straight down
	deck.pose.orientation.x = 0.0
	deck.pose.orientation.y = -1.0
	deck.pose.orientation.z = 0.0
	deck.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(deck)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# pick up card
	right_gripper.command_suction(True)

	# increment z position as deck height decreases
	deck_z -= .001

	# sleep for a moment
	rospy.sleep(1.0)

# move right gripper to desired card position for dealer
def right_to_dealer(right_arm, right_gripper):
	# get current number of cards dealer has
	global dealer_cards
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above drop point for dealer's card
	card = PoseStamped()
	card.header.frame_id = "base"

	# x, y, and z position of card
	card.pose.position.x = deck_x
	card.pose.position.y = deck_y + .2 + (.1 * dealer_cards)
	card.pose.position.z = deck_z

	# Orientation as a quaternion - straight down for now
	card.pose.orientation.x = 0.0
	card.pose.orientation.y = -1.0
	card.pose.orientation.z = 0.0
	card.pose.orientation.w = 0.0

	# Plan and execute path to desired card position from current state
	right_arm.set_pose_target(card)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# drop card
	right_gripper.stop()

	# sleep for a moment
	rospy.sleep(1.0)

	# increment number of cards dealer has
	dealer_cards += 1

# move right gripper to desired card position for player
def right_to_player(right_arm, right_gripper):
	# get current number of cards player has
	global player_cards
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above drop point for player's card
	card = PoseStamped()
	card.header.frame_id = "base"

	# x, y, and z position of card
	card.pose.position.x = deck_x + .25
	card.pose.position.y = deck_y + .2 + (.1 * player_cards)
	card.pose.position.z = deck_z

	# Orientation as a quaternion - straight down
	card.pose.orientation.x = 0.0
	card.pose.orientation.y = -1.0
	card.pose.orientation.z = 0.0
	card.pose.orientation.w = 0.0

	# Plan and execute path to desired card position from current state
	right_arm.set_pose_target(card)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# drop card
	right_gripper.stop()

	# sleep for a moment
	rospy.sleep(1.0)

	# increment number of cards player has
	player_cards += 1

# hover over the dealer's face down card, so we don't mess up gripper when moving down since it must make contact with card/table
def right_hover_face_down(right_arm):
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above deck of cards
	hover = PoseStamped()
	hover.header.frame_id = "base"

	# x, y, and z position of deck
	hover.pose.position.x = deck_x
	hover.pose.position.y = deck_y + .2
	hover.pose.position.z = deck_z - 0.015 + .1

	# Orientation as a quaternion - straight down
	hover.pose.orientation.x = 0.0
	hover.pose.orientation.y = -1.0
	hover.pose.orientation.z = 0.0
	hover.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(hover)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

# move to dealer's face down card in order to flip it over
def right_to_face_down(right_arm):
	# get global deck coordinates
	global deck_x
	global deck_y
	global deck_z

	# create pose above dealer's first card
	dealer_1 = PoseStamped()
	dealer_1.header.frame_id = "base"

	# x, y, and z position of dealer's first card
	dealer_1.pose.position.x = deck_x
	dealer_1.pose.position.y = deck_y + .2
	dealer_1.pose.position.z = deck_z - .015

	# Orientation as a quaternion - straight down
	dealer_1.pose.orientation.x = 0.0
	dealer_1.pose.orientation.y = -1.0
	dealer_1.pose.orientation.z = 0.0
	dealer_1.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(dealer_1)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

def hover_for_vision(right_arm, x, y, z):

	# create hover pose
	hover = PoseStamped()
	hover.header.frame_id = "base"

	# keep same x and y, but increment z
	hover.pose.position.x = x - .02
	hover.pose.position.y = y - .05
	hover.pose.position.z = z + .05

	# Orientation as a quaternion - straight down
	hover.pose.orientation.x = 0.0
	hover.pose.orientation.y = -1.0
	hover.pose.orientation.z = 0.0
	hover.pose.orientation.w = 0.0

	# Plan and execute path from current state
	right_arm.set_pose_target(hover)
	right_arm.set_start_state_to_current_state()

	# Add orientation constraint for path - straight down
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;
	consts = Constraints()
	consts.orientation_constraints = [orien_const]
	right_arm.set_path_constraints(consts)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

def rotate_90(right_arm):
	right_arm.clear_path_constraints()
	right_arm.clear_pose_targets()
	joints = right_arm.get_current_joint_values()
	joints[6] += np.pi/2
	right_arm.set_joint_value_target(joints)

	right_plan = right_arm.plan()
	right_arm.execute(right_plan)

	# sleep for a moment
	rospy.sleep(1.0)

def reset():
	global dealer_cards, player_cards
	dealer_cards = 0
	player_cards = 0
