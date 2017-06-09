#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
global en_walking, angular
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_enable_walking = rospy.Publisher('/enable_walking', Bool, queue_size=10)

angular = Vector3(0.0, 0.0, 0.5)

en_walking = 0


def sgn(a):
	if a > 0:
		return 1
	elif a < 0:
		return -1
	else:
		return 0

def callback(vector):
	global en_walking, angular
#	rospy.loginfo("vector %s and sgn %s" %(vector, sgn(vector.y)))
	if vector.z != -1:
		if  abs(vector.y) > 10: # we should moving
			if sgn(vector.y) > 0: # we shoud turn right
				if angular.z < 0: # we should start turn right
					angular.z = -angular.z
					pub_cmd_vel.publish( Twist( Vector3(0.0, 0.0, 0.0), angular ) )
					en_walking  = 1
					pub_enable_walking.publish(Bool(en_walking))
				else:
					
					if en_walking != 1:
						en_walking = 1
						rospy.loginfo("en_walking %s" %en_walking)
						pub_enable_walking.publish(Bool(en_walking))
						
			else:	#we should turn left
				if angular.z > 0: # we should start turn left
					angular.z = -angular.z
					pub_cmd_vel.publish( Twist( Vector3(0.0, 0.0, 0.0), angular ) )
					en_walking = 1
					pub_enable_walking.publish(Bool(en_walking))
				else:
					 if en_walking != 1:
						en_walking = 1
						pub_enable_walking.publish(Bool(en_walking))
				
		else:
			if en_walking != 0:
				en_walking = 0
				pub_enable_walking.publish(Bool(en_walking))

def listener():
	rospy.Subscriber('/robotis_op/camera/image_vector',Vector3, callback)
	rospy.init_node('robot_moving', anonymous=True)
	pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pub_enable_walking = rospy.Publisher('/enable_walking', Bool, queue_size=10)
	pub_cmd_vel.publish( Twist( Vector3(0.0, 0.0, 0.0), angular ) )
	pub_enable_walking.publish(Bool(en_walking))
	rospy.loginfo("node robot_moving  has been started")
	
	rospy.spin()

if __name__  ==  '__main__':
	listener()