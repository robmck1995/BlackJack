#! /usr/bin/python

# Code for this file was taken from:
# https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
# And then modified for our purposes

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

# global variable for subscriber, for only calling callback once
subscriber = None

# function that converts and saves received baxter image
def image_callback(msg):
	# convert ROS Image to OpenVC2
	try:
		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	# check for conversion errors
	except CvBridgeError as error:
		print(error)
		return
	# save image to current directory
	cv2.imwrite('/home/cc/ee106a/fa17/class/ee106a-abe/ros_workspaces/BlackJack/src/gameplay/src/card.jpg', cv2_img)
	# unsubscribe so we only save one image
	global subscriber
	subscriber.unregister()

def save_image(image_topic):
	# create subscriber with callback function
	global subscriber
	subscriber = rospy.Subscriber(image_topic, Image, image_callback)
	# sleep to allow time for callback to complete
	rospy.sleep(1.0)