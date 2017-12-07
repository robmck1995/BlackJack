#!/usr/bin/env python
import sys
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import moveit_commander
from baxter_interface import gripper as robot_gripper

from image_saver import *
from recognition_utils import *

from left_arm import *
from right_arm import *

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped


##########################################################################
# Position Polling
##########################################################################

# global variable for position subscriber, for only calling callback once
position_subscriber = None

# position topic to subscribe to
position_topic = None

# current position of right gripper
right_x = 0
right_y = 0
right_z = 0

# function that saves current position of end effector
def position_callback(msg):
	global right_x
	global right_y
	global right_z
	# save position globally
	right_x = msg.pose.position.x
	right_y = msg.pose.position.y
	right_z = msg.pose.position.z
	# unsubscribe so we only save one image
	global position_subscriber
	position_subscriber.unregister()

def poll_position():
	# create subscriber with callback function
	global position_subscriber
	global position_topic
	position_subscriber = rospy.Subscriber(position_topic, EndpointState, position_callback)
	# sleep to allow time for callback to complete
	rospy.sleep(1.0)


##########################################################################
# Card Recognition
##########################################################################

# keep track of both the dealer's and player's hands globally
player_hand = []
dealer_hand = []

# image topic to subscribe to
image_topic = None

# 0 = dealer, 1 = player
def card_recognition(person):
	# take a photo of a card
	global image_topic
	save_image(image_topic)
	# classify card
	try:
		card = recognize_card("/home/cc/ee106a/fa17/class/ee106a-abe/ros_workspaces/BlackJack/src/gameplay/src/card.jpg")
	except:
		print("Card recognition failed. The camera sucks. Sad!")
		print("Defaulting to face card.")
		if person == 0:
			global dealer_hand
			dealer_hand += [10]
		else:
			global player_hand
			player_hand += [10]
		return
	# don't care about suit
	card_value = card[0][0]
	# check for face cards
	if card_value == '*':
		print("Miss-classified card, defaulting to face card")
		card_value = 10
	elif card_value == 'J' or card_value == 'Q' or card_value == 'K':
		card_value = 10
	elif card_value == "A":
		card_value = 11
	else:
		card_value = int(card_value)
	# update proper hand with new card
	if person == 0:
		global dealer_hand
		dealer_hand += [card_value]
	else:
		global player_hand
		player_hand += [card_value]

	print(card_value)


##########################################################################
# Dealer Movement
##########################################################################

# deal a card
# person argument - 0 = dealer, 1 = player
def deal_card(right_arm, right_gripper, left_arm, left_gripper, person):
	# pick up card from deck
	right_to_deck(right_arm, right_gripper)

	# flip card over
	flip_card(right_arm, right_gripper, left_arm, left_gripper)

	# deal face up card to desired person
	if person == 0:
		left_clear_out(left_arm)
		right_to_dealer(right_arm, right_gripper)
	elif person == 1:
		right_to_player(right_arm, right_gripper)
	poll_position()
	global right_x, right_y, right_z
	hover_for_vision(right_arm, right_x, right_y, right_z)
	rotate_90(right_arm)
	card_recognition(person)

# sets up the BlackJack game
def initial_deal(right_arm, right_gripper, left_arm, left_gripper):
	# This loop deals the cards to set up the game
	for i in range(4):
		if i == 0 or i == 2:
			deal_card(right_arm, right_gripper, left_arm, left_gripper, 1)
		elif i == 1:
			right_to_deck(right_arm, right_gripper)
			right_hover_deck(right_arm)
			right_to_dealer(right_arm, right_gripper)
		else:
			deal_card(right_arm, right_gripper, left_arm, left_gripper, 0)
		right_hover_deck(right_arm)

# flips cards during dealing
def flip_card(right_arm, right_gripper, left_arm, left_gripper):
	global right_x, right_y, right_z
	right_tuck(right_arm)
	poll_position()
	left_to_card(left_arm, right_x, right_y, right_z)
	left_gripper.close()
	right_gripper.stop()
	left_tuck(left_arm)
	left_flip(left_arm)
	poll_position()
	right_shift(right_arm, right_x, right_y, right_z)
	poll_position()
	left_below(left_arm, right_x, right_y, right_z)
	left_up(left_arm, right_x, right_y, right_z)
	right_gripper.command_suction(True)
	left_gripper.open()
	rospy.sleep(0.1)
	left_avoid_right(left_arm)
	left_tuck(left_arm)

	# sleep for a moment
	rospy.sleep(1.0)

# flips the dealer's face down card over during dealer's turn
def flip_dealer_card(right_arm, right_gripper, left_arm, left_gripper):
	right_tuck(right_arm)
	right_hover_face_down(right_arm)
	right_to_face_down(right_arm)
	right_gripper.command_suction(True)
	flip_card(right_arm, right_gripper, left_arm, left_gripper)
	right_to_face_down(right_arm)
	right_gripper.stop()

	poll_position()
	global right_x, right_y, right_z
	hover_for_vision(right_arm, right_x, right_y, right_z)
	rotate_90(right_arm)
	card_recognition(0)

	right_hover_deck(right_arm)

	# sleep for a moment
	rospy.sleep(1.0)

def gameplay(right_arm, right_gripper, left_arm, left_gripper):
	global player_hand, dealer_hand
	
	initial_deal(right_arm, right_gripper, left_arm, left_gripper)
	player_value = sum(player_hand)
	dealer_value = sum(dealer_hand)
	
	if player_value == 21:
		print("Blackjack!! You win!!")
		return
	player_bust = False
	while player_value < 21 and player_bust == False:
		user_input = raw_input("Hit or stay?\n")
		user_input = user_input.lower()
		
		if user_input == "hit":
			deal_card(right_arm, right_gripper, left_arm, left_gripper, 1)
			player_value = sum(player_hand)
			player_bust = check_hand(1)
			player_value = sum(player_hand) ####NEWWWWWWWEWEWEWEEWWEWEEWEWEEEW
		elif user_input == "stay":
			break

	right_tuck(right_arm)
	flip_dealer_card(right_arm, right_gripper, left_arm, left_gripper)
	dealer_value = sum(dealer_hand)
	while dealer_value < 17:
		print(dealer_hand)
		deal_card(right_arm, right_gripper, left_arm, left_gripper, 0)
		dealer_value = sum(dealer_hand)
	dealer_bust = check_hand(0)

	if dealer_bust and player_bust:
		print("Push.")
	elif dealer_bust:
		print("DEALER BUSTS")
	elif player_bust:
		print("PLAYER BUSTS SUCKER")
	else:
		if dealer_value > player_value:
			print("PLAYER LOSES")
		elif dealer_value < player_value:
			print("PLAYER WINS")
		else:
			print("Push.")

# Returns true if person busted
def check_hand(person):
	global player_hand, dealer_hand

	if person == 1:
		hand = player_hand
	else:
		hand = dealer_hand
	hand_value = sum(hand)
	if hand_value > 21:
		for j in range(len(hand)):
			if hand[j] == 11:
				hand[j] = 1
				return False
		return True
	return False

def reset2():
	global player_hand, dealer_hand
	player_hand = []
	dealer_hand = []	




def main():
	rospy.init_node('BlackJack_dealer')
	# INITIAL SETUP FOR VISION
	# use right hand to take photos of cards
	global image_topic
	image_topic = "/cameras/right_hand_camera/image"
	# train card recognition, only once
	training()

	# INITIAL SETUP FOR POSITION POLLING
	global position_topic
	position_topic = "/robot/limb/right/endpoint_state"
	global right_x, right_y, right_z
	# INITIAL SETUP FOR MOVEMENT
	moveit_commander.roscpp_initialize(sys.argv)
	# Initialize arms
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	left_arm = moveit_commander.MoveGroupCommander('left_arm')
	right_arm = moveit_commander.MoveGroupCommander('right_arm')
	left_arm.set_planner_id('RRTConnectkConfigDefault')
	left_arm.set_planning_time(10)
	right_arm.set_planner_id('RRTConnectkConfigDefault')
	right_arm.set_planning_time(10)
	# Set up grippers
	right_gripper = robot_gripper.Gripper('right')
	left_gripper = robot_gripper.Gripper('left')
	left_gripper.calibrate()

	play = True
	while play:
		right_tuck(right_arm)
		left_tuck(left_arm)
		right_hover_deck(right_arm)
		reset()
		reset2()
		gameplay(right_arm, right_gripper, left_arm, left_gripper)
		answer = raw_input("Would you like to play another hand?\n")
		if answer.lower() == "no":
			play = False

if __name__ == '__main__':
	main()