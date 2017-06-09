#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
pub = rospy.Publisher('/robotis_op/camera/head_detected', Vector3, queue_size=10)

z_zones = [0,0,0]

templates = []

for i in range(4):
	file = "/home/robotis/catkin_ws/src/darwin_fighter/scripts/templates/template_" + str(i) + ".png"
	templates.append(cv2.imread(file,0))


# w,h = template.shape[::-1]

def callback(image_message):
	#rospy.loginfo("I got an image  at %s " %rospy.get_time())
	cv_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding="passthrough")
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
	z_zones = identify(cv_image)
	pub.publish(Vector3(z_zones[0],z_zones[1],z_zones[2]))
	cv2.imshow('frame', cv_image)
	cv2.waitKey(1)


def identify(frame):
	zones = [-1,-1,-2]
	x_avr = -1
	y_avr = -1
	i_all = 0
	konst = 250
	#cv2.imshow('frame', frame)
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    	for template_base in templates:
		for i in range(15):
			k = 0.15 + i/10
			template = cv2.resize(template_base, None, fx=k, fy=k)
            		w, h = template.shape[::-1]
            		res = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
		        threshold = 0.7
		        loc = np.where(res >= threshold)
			for pt in zip(*loc[::-1]):
				x_avr += pt[0] + w/2
				y_avr += pt[1] + h/2
				i_all += 1

	if i_all > 0 :
		x_avr = int(x_avr/i_all) - 160
		y_avr = int(y_avr/i_all) - 120
		cv2.circle(frame, (int(x_avr+160),int(y_avr+120)), 10, (255, 0, 0), -1)
	

	if x_avr ==  -1:
		zones = [ x_avr , y_avr, -1]
	else:
		zones = [x_avr, y_avr, 0]
	cv2.imshow('frame_gray',frame)
	cv2.waitKey(1)

	rospy.loginfo("pub %s" %zones)
	return zones


def listener():
	rospy.Subscriber("/robotis_op/camera/image_raw",Image, callback)
	rospy.init_node('head_detecting', anonymous=True)
	rospy.spin()

if __name__  ==  '__main__':
	listener()