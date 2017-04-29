#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

import dynamic_reconfigure.client
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import Int32

from sensor_msgs.msg import JointState

import std_srvs.srv
import time

# from subprocess import call

class Darwin:

    def __init__(self, ns="/robotis_op/"):
        self.ns = ns
        self.joints = None
        self.angles = None

        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        
        self.max_speed = 0.500
        self.max_turn = 60.0*math.pi/180.0

        self.speed = 0
        self.turn = 0
        
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
        
        self._pub_cmd_vel = rospy.Publisher(self.ns+"cmd_vel", Twist)
        self._pub_enable_walking = rospy.Publisher(self.ns+"enable_walking", Bool)
        self._pub_start_action = rospy.Publisher(self.ns+"start_action", Int32)

    def enable_walking(self, msg):
        self._pub_enable_walking.publish(msg)

    def start_action(self, msg):
        self._pub_start_action.publish(msg)

    def cmd_vel(self, msg):
        self._pub_cmd_vel.publish(msg)

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
            self._pub_joints

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
    darwin.set_angles_slow(get_leg_angles_transformed(angles), 1) 


def initialize(): 

    rospy.init_node('darwin_figher', anonymous=True)
    darwin = Darwin()

    max_tv = darwin.max_speed
    max_rv = darwin.max_turn
    walking = False
    dirty = False

    speed = 0
    turn = 0

    reset_legs_fight(darwin)
    time.sleep(2) 

    while True:
        rospy.loginfo("OK")
        inp = raw_input()        
        if inp == 'a':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            uppercut(darwin, mirror_arm_angles(uppercut_right_angles()))
        elif inp == 'o':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            uppercut(darwin, uppercut_right_angles())
        if inp == 'e':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            straight(darwin, mirror_arm_angles(straight_right_angles()))
        elif inp == 'u':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            straight(darwin, straight_right_angles())
        if inp == ';':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            hammer(darwin, mirror_arm_angles(hammer_right_angles()))
        elif inp == 'q':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            hammer(darwin, hammer_right_angles())
        elif inp == 'j':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            side(darwin, mirror_arm_angles(side_right_angles()))
        elif inp == 'k':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            side(darwin, side_right_angles())
        elif inp == '\'':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            squeeze(darwin)
        elif inp == ',':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            strike(darwin)
        elif inp == '.':
            if walking:
                darwin.enable_walking(False)
                rospy.sleep(0.2) 
            block(darwin)
        elif inp == 'z':
            if walking:
                darwin.enable_walking(False)
            rospy.loginfo("Bye!")
            break 

        elif inp == "c":    # ^
            darwin.speed = 1
            darwin.turn = 0 
            dirty = True
        elif inp == "h":    # <
            darwin.speed = 0
            darwin.turn = 1 
            dirty = True
        elif inp == "w":    # |
            darwin.speed = -1
            darwin.turn = 0 
            dirty = True
        elif inp == "r":    # ^>
            darwin.speed = 1
            darwin.turn = -1 
            dirty = True
        elif inp == "n":    # >
            darwin.speed = 0
            darwin.turn = -1 
            dirty = True
        elif inp == "g":    # <^
            darwin.speed = 1
            darwin.turn = 1 
            dirty = True
        elif inp == "v":    # |>
            darwin.speed = -1
            darwin.turrn = -1 
            dirty = True
        elif inp == "m":    # <|
            darwin.speed = -1
            darwin.turn = 1 
            dirty = True
        elif inp == "t":    # stop
            darwin.speed = 0
            darwin.turn = 0
            dirty = True

        elif inp == '1':    # kick left
            darwin.start_action(13) 
            walking = False
        elif inp == '2':    # kick right
            darwin.start_action(12) 
            walking = False
        elif inp == '3':    # get up forward
            darwin.start_action(10) 
            walking = False
        elif inp == '4':    # get up backward
            darwin.start_action(11)
            walking = False

        elif inp == '9':
            darwin.enable_walking(True) 
            walking = True
        elif inp == '0':
            darwin.enable_walking(False)
            walking = False

        elif inp == "b":    # +al
            max_tv += max_tv/10
            max_rv += max_rv/10
            dirty = True
        elif inp == "x":    # -al
            max_tv -= max_tv/10
            max_rv -= max_rv/10
            dirty = True
        elif inp == "d":    # +l
            max_tv += max_tv/10
            dirty = True
        elif inp == "i":    # -l
            max_tv -= max_tv/10
            dirty = True
        elif inp == "f":    # +a
            max_rv += max_rv/10
            dirty = True
        elif inp == "y":    # -a
            max_rv -= max_rv/10
            dirty = True

        else:
            rospy.loginfo("No such command") 
    
        if dirty:
            darwin.twist.linear.x = darwin.speed * max_tv
            darwin.twist.angular.z = darwin.turn * max_rv
            darwin.cmd_vel(darwin.twist)
            darwin.dirty = False 

if __name__ == '__main__':
    try:
        initialize()

    except rospy.ROSInterruptException:
        pass
