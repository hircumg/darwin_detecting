#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import cv2
import numpy as np
import time

if __name__  ==  '__main__':
	rospy.init_node('test_array', anonymous=True)
	mask  = np.zeros((240,320))
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		x_avr = 0
		y_avr = 0
		i_all = 0
		t_in = rospy.get_time()
		for i in range(mask.shape[0]):
	        	for j in range(mask.shape[1]):
	            		if mask[i][j] < 200:
	                		i_all += 1
			                y_avr += i
			                x_avr += j
		t_in = rospy.get_time() - t_in	
		rospy.loginfo("I got an image  at %s " %t_in)
		rate.sleep()

