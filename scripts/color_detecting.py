#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

z_zones = 0

def callback(image_message):
	#rospy.loginfo("I got an image  at %s " %rospy.get_time())
	cv_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding="passthrough")
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
	z_zones = identify(cv_image)
	#cv2.imshow('frame', cv_image)
	#cv2.waitKey(1)


def identify(frame):
	zones = 1
	konst = 3000
	lower_blue = np.array([85, 30, 50])
    	upper_blue = np.array([156, 255, 255])
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    	mask = cv2.inRange(frame, lower_blue, upper_blue)
	
	cv2.imshow('frame', mask)
	cv2.waitKey(1)
	

    	left_top = mask[0:120:1, 0:80:1]
    	left_bottom = mask[120:240:1, 0:80:1]
    	right_top = mask[0:120:1, 240:320:1]
    	right_bottom = mask[120:240:1, 240:320:1]
    	center_top = mask[0:80:1,80:240:1]
    	center_center = mask[80:160:1, 80:240:1]
    	center_bottom = mask[160:240:1, 80:240:1]
    	summing = [left_top.sum()/255, left_bottom.sum()/255, center_top.sum()/255, center_center.sum()/255, center_bottom.sum()/255, right_top.sum()/255, right_bottom.sum()/255]
    	rospy.loginfo(summing)
    	if summing[0] > konst:
		zones = zones*10 + 0	# robot is on the top left
	if summing[1] > konst:
		zones = zones*10 + 1	# robot is on the bottom left
	if summing[2] > konst:
		zones = zones*10 + 2	# robot is on the top center
	if summing[3] > konst:
		zones = zones*10 + 3	# robot is on the center
	if summing[4] > konst:
		zones = zones*10 + 4	# robot is on the bottom center
	if summing[5] > konst:
		zones = zones*10 + 5	# robot is on the top right
	if summing[6] > konst:
		zones = zones*10 + 6	# robot is on the bottom right
	rospy.loginfo("pub %s" %zones)
	return zones


def listener():
	pub = rospy.Publisher('/robotis_op/camera/camera_detected', Int32, queue_size=10)
	rospy.Subscriber("/robotis_op/camera/image_raw",Image, callback)
	rospy.init_node('color_detecting', anonymous=True)
	pub.publish(z_zones)
	rospy.spin()

if __name__  ==  '__main__':
	listener()