#!/usr/bin/env python

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

class Darwin:
    """
    Client ROS class for manipulating Darwin OP in Gazebo
    """
    
    def __init__(self,ns="/robotis_op/"):
        self.ns=ns
        self.joints=None
        self.angles=None
        
        self._sub_joints=rospy.Subscriber(ns+"joint_states",JointState,self._cb_joints,queue_size=1)
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None: break
            rospy.sleep(0.1)            
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        
        
        rospy.loginfo("Creating joint command publishers")
        self._pub_joints={}
        for j in self.joints:
            p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64)
            self._pub_joints[j]=p
        
        rospy.sleep(1)
        
        self._pub_cmd_vel=rospy.Publisher(ns+"cmd_vel",Twist)
        

    def set_walk_velocity(self,x,y,t):
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=t
        self._pub_cmd_vel.publish(msg)
        
    def _cb_joints(self,msg):
        if self.joints is None:
            self.joints=msg.name
        self.angles=msg.position
        
    
    def get_angles(self):
        if self.joints is None: return None
        if self.angles is None: return None
        return dict(zip(self.joints,self.angles))

    def set_angles(self,angles):
        for j,v in angles.items():
            if j not in self.joints:
                rospy.logerror("Invalid joint name "+j)
                continue
            self._pub_joints[j].publish(v)

    def set_angles_slow(self,stop_angles,delay=2):
        start_angles=self.get_angles()
        start=time.time()
        stop=start+delay
        r=rospy.Rate(100)
        while not rospy.is_shutdown():
            t=time.time()
            if t>stop: break
            ratio=(t-start)/delay            
            angles=interpolate(stop_angles,start_angles,ratio)                        
            self.set_angles(angles)
            r.sleep()

               

def interpolate(anglesa,anglesb,coefa):
    z={}
    joints=anglesa.keys()
    for j in joints:
        z[j]=anglesa[j]*coefa+anglesb[j]*(1-coefa)
    return z

def get_distance(anglesa,anglesb):
    d=0
    joints=anglesa.keys()
    if len(joints)==0: return 0
    for j in joints:
        d+=abs(anglesb[j]-anglesa[j])
    d/=len(joints)
    return d


#the following array represent the joint mapping from file to real joint names
angle_mappings=[
    {'name': 'j_pelvis_l', 'sign': -1},
    {'name': 'j_thigh2_l', 'sign': 1},
    {'name': 'j_thigh1_l', 'sign': -1},
    {'name': 'j_tibia_l', 'sign': 1},
    {'name': 'j_ankle1_l', 'sign': -1},
    {'name': 'j_ankle2_l', 'sign': 1},
    {'name': 'j_pelvis_r', 'sign': -1},
    {'name': 'j_thigh2_r', 'sign': -1},
    {'name': 'j_thigh1_r', 'sign': -1},
    {'name': 'j_tibia_r', 'sign': -1},
    {'name': 'j_ankle1_r', 'sign': 1},
    {'name': 'j_ankle2_r', 'sign': 1}
]

def reset():
    reset_w=rospy.ServiceProxy('/gazebo/reset_world',std_srvs.srv.Empty)
    reset_w()


def get_angles_transformed(angles):
    """
transforms the angles provided in array into the dictionary and converts degrees to radians
    :param angles:
    :return:
    """
    result = {}

    for i in range(0, len(angles)):
        result[angle_mappings[i]['name']] = angle_mappings[i]['sign']*angles[i]*math.pi/180
    return result


def reset_arms(darwin):
    darwin.set_angles_slow({
        'j_high_arm_r': math.pi/2,
        'j_low_arm_r': math.pi/3,
        'j_shoulder_r': 0,
        'j_high_arm_l': math.pi/2,
        'j_low_arm_l': -math.pi/3,
        'j_shoulder_l': 0,
    }, 2)


def loop_walk(darwin, input_angles):
    rate = rospy.Rate(350)
    i=0
    while (rospy.is_shutdown()!=1):
        if i>2000:
            i=1500
        darwin.set_angles(get_angles_transformed(input_angles[i, :]))
        i=i+1
        rate.sleep()


def initialize():
    rospy.init_node('darwin_file_walker', anonymous=True)
    darwin=Darwin()
    darwin.set_angles_slow(get_angles_transformed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), 2)
    time.sleep(2)
    # reset()
    # reset_arms(darwin)

    angle_array=np.load('/home/robotis/catkin_ws/src/darwinvheight/scripts/angles_darwin_1.npy')
    rospy.loginfo("\n======\narray:\n")
    rospy.loginfo(angle_array)
    rospy.loginfo(type(angle_array))
    rospy.loginfo("======\n")
    np.save('ad.npy', angle_array)
    # rospy.loginfo('Setting the initital position')
    darwin.set_angles_slow(get_angles_transformed(angle_array[0, :]),2)
    time.sleep(4)
    rospy.loginfo('Start walking')
    loop_walk(darwin, angle_array)

if __name__ == '__main__':
    try:
        initialize()

    except rospy.ROSInterruptException:
        pass
