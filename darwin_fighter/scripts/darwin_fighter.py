#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

import dynamic_reconfigure.client
from std_msgs.msg import Float64
import std_srvs.srv
import numpy as np
from sensor_msgs.msg import JointState
import time
from subprocess import call
from subprocess import Popen, PIPE
import threading
import time


class Darwin:

    def __init__(self, ns="/robotis_op/"):
        self.ns = ns
        self.joints = None
        self.angles = None
        
        self._sub_joints = rospy.Subscriber(ns+"joint_states", JointState, self._cb_joints, queue_size=1)
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)            
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        
        rospy.loginfo("Creating joint command publishers")
        self._pub_joints = {}
        for j in self.joints:
            p = rospy.Publisher(self.ns+j+"_position_controller/command", Float64)
            self._pub_joints[j] = p
        
        rospy.sleep(1)
        
        self._pub_cmd_vel = rospy.Publisher(ns+"cmd_vel", Twist)

    def _cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                rospy.logeror("Invalid joint name " + j)
                continue
            self._pub_joints[j].publish(v)

    def jet_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                rospy.logerror("Invalid joint name "+j)
                continue
            self._pub_joints[j].publish(v)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = self.get_angles()
        start = time.time()
        stop = start+delay
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if t > stop:
                break
            ratio = (t-start)/delay
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            r.sleep()


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j]*coefa+anglesb[j]*(1-coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j]-anglesa[j])
    d /= len(joints)
    return d

# the following array represent the joint mapping from file to real joint names
leg_angle_mappings = [
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

arm_angle_mappings = [
    {'name': 'j_high_arm_r', 'sign': 1}, 
    {'name': 'j_low_arm_r', 'sign': -1},
    {'name': 'j_shoulder_r', 'sign': 1},
    {'name': 'j_high_arm_l', 'sign': -1}, 
    {'name': 'j_low_arm_l', 'sign': -1},
    {'name': 'j_shoulder_l', 'sign': -1}
]


def reset():

    reset_w()


def get_leg_angles_transformed(angles):
    
    # transforms the angles provided in array into the dictionary and converts degrees to radians
    
    result = {}

    for i in range(0, len(angles)):
        result[leg_angle_mappings[i]['name']] = leg_angle_mappings[i]['sign']*angles[i]*math.pi/180
    return result


def get_arm_angles_transformed(angles):
    result = {}
    result['j_high_arm_r'] = angles['j_high_arm_r'] + math.pi/4         # max: 17p/36=85    | min: -p/2 | zero: p/4 | up
    result['j_low_arm_r'] = angles['j_low_arm_r'] - math.pi/2           # max: 31p/36=155   | min: 0    | zero: p/2 | bend
    result['j_shoulder_r'] = angles['j_shoulder_r']                     # max: p            | min: -p   | zero: 0   | up
    result['j_high_arm_l'] = angles['j_high_arm_l']*(-1) - math.pi/4
    result['j_low_arm_l'] = angles['j_low_arm_l']*(-1) + math.pi/2
    result['j_shoulder_l'] = angles['j_shoulder_l']*(-1)
    return result


def reset_arms(darwin):
    angles = { 
        'j_high_arm_r': 0,  # max: +p/2.1           | min: -p/2.2
        'j_low_arm_r': 0,   # max: 29p/36=145deg    | min: 0
        'j_shoulder_r': 0,  # max: +p               | min: -p/2
        'j_high_arm_l': 0,  # max: p/2.1            | min: -p/2.2
        'j_low_arm_l': 0,   # max: 29p/36=145deg    | min: 0
        'j_shoulder_l': 0   # max: p/2.1            | min: -p/2.2
    }
    darwin.set_angles_slow(get_arm_angles_transformed(angles), 1)


def get_arm_angles_dictionary(angles_list): 
    angles = {
        'j_high_arm_r': angles_list[0],
        'j_low_arm_r': angles_list[1],
        'j_shoulder_r': angles_list[2],
        'j_high_arm_l': angles_list[3],
        'j_low_arm_l': angles_list[4],
        'j_shoulder_l': angles_list[5]
    }
    return angles


def mirror_arm_angles(angles):
    mirrored_angles = []
    for angle_dict in angles:
         mirrored_angles.append(mirror_dict_of_angles(angle_dict))
    return mirrored_angles

def mirror_dict_of_angles(angles): 
    tmp_j_high_arm_r = angles['j_high_arm_r']
    tmp_j_low_arm_r = angles['j_low_arm_r']
    tmp_j_shoulder_r = angles['j_shoulder_r']
    angles['j_high_arm_r'] = angles['j_high_arm_l']
    angles['j_low_arm_r'] = angles['j_low_arm_l']
    angles['j_shoulder_r'] = angles['j_shoulder_l']
    angles['j_high_arm_l'] = tmp_j_high_arm_r
    angles['j_low_arm_l'] = tmp_j_low_arm_r
    angles['j_shoulder_l'] = tmp_j_shoulder_r
    return angles


def squeeze(darwin):
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/6, 0,  math.pi/1.6,
        -math.pi/6, 0,  math.pi/1.6
    ])) 
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.7, 0,  math.pi/1.6,
        -math.pi/1.7, 0,  math.pi/1.6
    ]))
    angles.append(get_arm_angles_dictionary([
        -math.pi/4, 0,  math.pi/1.6,
        -math.pi/4, 0,  math.pi/1.6
    ]))
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[1]), 0.4)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[2]), 0.4)
    reset_arms_fight(darwin)


def strike(darwin):
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.7,   math.pi/6,  math.pi/1.5,
        -math.pi/1.7,   math.pi/6,  math.pi/1.5
    ]))
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    reset_arms_fight(darwin)



def uppercut_right_angles(): 
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.8,       0,              math.pi*0.9,
        -math.pi/2.6,   2*math.pi/4,    -13*math.pi/36
    ]))
    return angles

def uppercut(darwin, angles): 
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    reset_arms_fight(darwin) 



def block(darwin): 
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/2, 2*math.pi/3,    math.pi/2,
        -math.pi/2, 2*math.pi/3,    math.pi/2
    ]))
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    time.sleep(0.5)
    reset_arms_fight(darwin)



def straight_right_angles():
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/2, 17*math.pi/20,  math.pi/4,
        -math.pi/2, 2*math.pi/3,    math.pi/6
    ]))
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.8,   0,              math.pi/2,
        -math.pi/2.6,   2*math.pi/4,    -math.pi/4
    ]))
    return angles

def straight(darwin, angles): 
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[1]), 0.35)
    reset_arms_fight(darwin) 



def side_right_angles():
    angles = []
    angles.append(get_arm_angles_dictionary([
        0,          0,  1.5*math.pi/2,
        -math.pi/2, 0,  math.pi/6
    ]))
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.5,   0,              1.3*math.pi/2,
        -math.pi/2.6,   2*math.pi/4,    -math.pi/4
    ]))
    return angles

def side(darwin, angles):
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[1]), 0.35)
    reset_arms_fight(darwin) 



def hammer_right_angles(): 
    angles = []
    angles.append(get_arm_angles_dictionary([
        -math.pi/2, 17*math.pi/20,  math.pi/4,
        -math.pi/2, 2*math.pi/3,    math.pi/6
    ]))
    angles.append(get_arm_angles_dictionary([
        -math.pi/2,     0,              math.pi*1.5,
        -math.pi/2.6,   2*math.pi/4,    -math.pi/4
    ]))
    angles.append(get_arm_angles_dictionary([
        -math.pi/1.7,   0,              math.pi/2.2,
        -math.pi/2.6,   2*math.pi/4,    -math.pi/4
    ]))
    return angles

def hammer(darwin, angles):
    darwin.set_angles_slow(get_arm_angles_transformed(angles[0]), 0.5)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[1]), 0.4)
    darwin.set_angles_slow(get_arm_angles_transformed(angles[2]), 0.4)
    reset_arms_fight(darwin) 



def reset_arms_fight(darwin):
    angles = get_arm_angles_dictionary([
        -math.pi/2.6,   2*math.pi/3,    -11*math.pi/36, 
        -math.pi/2.6,   2*math.pi/3,    -11*math.pi/36
        #               p/2+p/6         -p/4+p/18
    ])
    darwin.set_angles_slow(get_arm_angles_transformed(angles), 0.5)


def reset_legs_fight(darwin):
    angles = [ 
       9.516219606e-12,     # 'j_pelvis_l': 0,
       3.99624770745e+01,   # 'j_thigh2_l': 0, 2.960183487e+01 * 1.35
       -6.844553674e-12,    # 'j_thigh1_l': 0,
       -7.931234095e+01,    # 'j_tibia_l': 0,
       4.971050607e+01,     # 'j_ankle1_l': 0,
       2.468110432e-12,     # 'j_ankle2_l': 0,
       2.407043782e-11,     # 'j_pelvis_r': 0,
       6.71091831945e+01,   # 'j_thigh2_r': 0, 4.971050607e+01 * 1.35
       -1.999932783e-11,    # 'j_thigh1_r': 0,
       -7.931234095e+01,    # 'j_tibia_r': 0,
       2.960183487e+01,     # 'j_ankle1_r': 0,
       2.518999307e-12      # 'j_ankle2_r': 0 
    ]
    darwin.set_angles_slow(get_leg_angles_transformed(angles), 1.5) 


def initialize():

    teleop_thread = TeleopThread(0, "TELEOP_THREAD")
    teleop_thread.start()
    # teleop_thread.join()

    rospy.init_node('darwin_figher', anonymous=True)
    darwin = Darwin()

    # darwin.set_angles_slow(get_leg_angles_transformed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), 2)
    # time.sleep(2)
    # reset() 

    # angles = np.load('angles_darwin_1.npy')
    # rospy.loginfo(angles)
    # np.savetxt('angles_darwin_txt.txt', angles)
    reset_legs_fight(darwin) 
    reset_arms_fight(darwin)

    while True:
        rospy.loginfo("""
            Uppercut left: a
            Uppercut right: o
            Straight left: e
            Straight right: u
            Hammer left: ;
            Hammer right: q
            Side left: j
            Side right: k
            Squeeze: '
            Strike: ,
            Block: .
            Exit: z
        """)
        inp = raw_input()        
        if inp == 'a':
            uppercut(darwin, mirror_arm_angles(uppercut_right_angles()))
        elif inp == 'o':
            uppercut(darwin, uppercut_right_angles())
        if inp == 'e':
            straight(darwin, mirror_arm_angles(straight_right_angles()))
        elif inp == 'u':
            straight(darwin, straight_right_angles())
        if inp == ';':
            hammer(darwin, mirror_arm_angles(hammer_right_angles()))
        elif inp == 'q':
            hammer(darwin, hammer_right_angles())
        if inp == 'j':
            side(darwin, mirror_arm_angles(side_right_angles()))
        elif inp == 'k':
            side(darwin, side_right_angles())
        elif inp == '\'':
            squeeze(darwin)
        elif inp == ',':
            strike(darwin)
        elif inp == '.':
            block(darwin)
        elif inp == 'z':
            rospy.loginfo("Bye!")
            break

        elif inp == '11':
            call(["mpg321", "/home/robotis/Music/WhatNowDarwin.mp3"])

        elif inp == '12':
            teleop_thread.cmd("DO IT")
            # process = Popen(["roslaunch", "robotis_op_teleop", "robotis_op_teleop_keyboard.launch"], stdout=PIPE)   
            # (output, err) = process.communicate()
            # exit_code = process.wait()

        else:
            rospy.loginfo("No such command")


    # rospy.loginfo('Setting the initital position')
    # time.sleep(4)

class TeleopThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name 
        self.cmd = "COMMAND"
    def run(self):
        rospy.loginfo("Running")
        while True:
            if self.cmd != "":
                rospy.loginfo("A new command %s"%(self.cmd))
                # self.cmd = "" 
            time.sleep(1)
    def cmd(self, command):
        if self.cmd == "":
            self.cmd = command
        

if __name__ == '__main__':
    try:
        initialize()

    except rospy.ROSInterruptException:
        pass
