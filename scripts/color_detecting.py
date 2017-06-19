#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import cv2
import numpy as np
from time import time
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from cv_bridge import CvBridge, CvBridgeError

pub = rospy.Publisher('/robotis_op/camera/image_vector', Vector3, queue_size=10)


z_zones = [0,0,0]

templates = []

for i in range(4):
	file = "/home/robotis/catkin_ws/src/darwin_detecting/scripts/templates/template_" + str(i) + ".png"
	templates.append(cv2.imread(file,0))


# w,h = template.shape[::-1]

def callback(image_message):
	#rospy.loginfo("I got an image  at %s " %rospy.get_time())
	cv_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding="passthrough")
	z_zones = identify(cv_image)
	pub.publish(Vector3(z_zones[0],z_zones[1],z_zones[2]))
	#cv2.imshow('frame', cv_image)
	#cv2.waitKey(1)
	


def identify(frame):
	t_in = time()
	zones = [0,0,0]
	konst = 250
	frame = cv2.addWeighted(frame,2,np.zeros(frame.shape,frame.dtype),0.0,0.0)
	hsv= cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	
	#mask = cv2.inRange(hsv, (150,91,100),(200,250,255))
	mask = cv2.inRange(hsv, (0,127,47),(5,255,255))
	
	#mask = cv2.medianBlur(mask,3)
	


	#t_in = rospy.get_time() - t_in	
	#rospy.loginfo("1. I got an image  at %s " %t_in)
	

	M = cv2.moments(mask)
    	area = M['m00']
    	if area > 0:
        	cx = int(M['m10'] / area)
        	cy = int(M['m01'] / area)
        	cv2.circle(frame, (int(cx), int(cy)), 10, (255, 0, 0), -1)
		zones = [cx-160, cy-120, 0]
    	else:
        	cx = None
        	cy = None
		zones = [0, 0, -1]
	
	print("area= %s, cx = %s, cy = %s"%(area, cx, cy))

	t_in = time()- t_in	
	rospy.loginfo("I got an image  at %s " %t_in)



	frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
	cv2.imshow('mask',mask)
	cv2.imshow('frame',frame)
	cv2.waitKey(1)

	rospy.loginfo("pub %s" %zones)
	return zones


def listener():
	rospy.Subscriber("/robotis_op/camera/image_raw",Image, callback)
	rospy.init_node('head_detecting', anonymous=True)
	rospy.spin()

if __name__  ==  '__main__':
	listener()