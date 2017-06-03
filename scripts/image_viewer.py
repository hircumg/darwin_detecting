#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(image_message):
	rospy.loginfo("I got an image  at %s " %rospy.get_time())
	cv_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding="passthrough")
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
	cv2.imshow('frame', cv_image)
	cv2.waitKey(1)

def listener():
	rospy.init_node('image_viewer', anonymous=True)
	rospy.Subscriber("/robotis_op/camera/image_raw",Image, callback)
	rospy.spin()

if __name__  ==  '__main__':
	listener()