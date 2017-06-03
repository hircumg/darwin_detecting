#!/usr/bin/env python

# from robotis_op_gazebo.darwin import Darwin 

import rospy
import math
# from darwinvheight.darwin import Darwin
from geometry_msgs.msg import Twist

import dynamic_reconfigure.client
# from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64
import std_srvs.srv
import numpy as np
from sensor_msgs.msg import JointState
import time

from robotis_op_simulation_walking.darwin import Darwin

if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin()
    rospy.sleep(1) 
 
    rospy.loginfo("Darwin Walker Demo Starting")


    darwin.set_walk_velocity(10.4,10,10)
    rospy.sleep(3)
    darwin.set_walk_velocity(0,0,0)
    
    rospy.loginfo("Darwin Walker Demo Finished")
