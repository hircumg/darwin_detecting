from bitbots.ipc.ipc import *
from bitbots.robot.pypose import *
from bitbots.util.kinematicutil import *
from bitbots.robot.kinematics import *
from bitbots.util.animation import *
import numpy as np
import time

class StairClimb :

    def __init__ ( self ):
        self.ipc = SharedMemoryIPC()
        self.pose = PyPose()
        self.robot = Robot()
        self.task = KinematicTask( self.robot )

   
    def wait_for_end( self, sleeptime =2):
        time.sleep (0.1)
        while ( not self.ipc.controlable ) or ( self.ipc.get_state () == STATE_ANIMATION_RUNNING ):
            time.sleep(0.05)
        time.sleep( sleeptime )

    def update_pose( self , sleeptime =2):
        self.ipc.update( self.pose )
        self.wait_for_end( sleeptime )
        self.pose = self.ipc.get_pose()
        self.robot.update( self.pose )

    def animation_play( self, animation, sleeptime =0.5):
        play_animation( animation, self.ipc )
        self.wait_for_end(sleeptime)
        self.pose = self.ipc.get_pose()
        self.robot.update( self.pose )

    def walk_init( self ):
        self.wait_for_end (0)
        self.animation_play (" walkready " ,0)

    def perform_task( self, array, i, legnr, speed =1):
        if legnr ==1:
            self.task.perform(0, 34 + i, [(1 , 0 , 0) , array ], (1e-2 , 1), (0, 3), 100, [15 + i], [7 + i, 17 + i])
        else :
            self.task.perform(0, 35 - i, [(1 , 0 , 0), array ], (1e-2 , 1), (0, 3), 100, [16 - i], [8 -i, 18 - i])
        self.robot.set_angles_to_pose( self . pose , -1 , speed )

    def walk_step( self, height, i ):
        i = i % 2
        if i == 1:
            balance =" balance_right "
            foot_up = np.array(( -20 , 47 , -265+ height ))
            foot_forward = np.array((120 , 47 , -265+ height ))
            foot_down = np.array((120 , 47 , -275+ height ))
            balance_shift =" balance_right_to_left "
            foot_up2 = np.array(( -50 , -47 , -270+ height ))
            foot_forward2 = np.array((35 , -47 , -270+ height ))
            foot_down2 = np.array((60 , -47 , -275+ height ))
        else :
            balance =" balance_left "
            foot_up = np.array(( -20 , -47 , -285+ height ))
            foot_forward = np.array((120 , -47 , -285+ height ))
            foot_down = np.array((120 , -47 , -315+ height ))
            balance_shift =" balance_left_to_right "
            foot_up2 = np . array(( -50 , 47 , -270+ height ))
            foot_forward2 = np.array((35 , 47 , -270+ height ))
            foot_down2 = np.array((60 , 47 , -275+ height ))

        self.animation_play( balance )

        self.perform_task( foot_up ,i ,1)
        self.update_pose(1)

        self.perform_task( foot_forward ,i ,1)
        if i ==1:
            self.pose.l_ankle_pitch.goal = -20
            self.pose.r_ankle_roll.goal = -12
            self.pose.r_knee.goal = 49
        else :
            self.pose.r_ankle_pitch.goal = 20
            self.pose.l_ankle_roll.goal = 12
            self.pose.l_knee.goal = -47
            self.update_pose (1)

        self.perform_task( foot_down ,i ,1 ,0.5)
        if i ==1:
            self.pose.l_ankle_pitch.goal = -5
            self.pose.l_ankle_roll.goal = -15
        else :
            self.pose.r_ankle_pitch.goal = -24
            self.pose.r_ankle_roll.goal = 15
            self.update_pose(0.5)

        self.animation_play( balance_shift )

        self.perform_task( foot_up2 ,i ,2)
        if i ==1:
            self.pose.l_ankle_roll.goal = 20
        else :
            self.pose.r_ankle_roll.goal = -20
            self.update_pose(1)

        self.perform_task( foot_forward2 ,i ,2)
        self.pose.r_hip_pitch.goal = -100
        self.pose.l_hip_pitch.goal = 100
        self.update_pose (1)

        self.perform_task( foot_down2 ,i ,2 ,0.5)
        self.pose.r_hip_pitch.goal = -110
        self.pose.l_hip_pitch.goal = 110
        if i ==1:
            self.pose.r_ankle_pitch.goal = 52
            self.pose.r_ankle_pitch.speed = 20
        else :
            self.pose.l_ankle_pitch.goal = -52
            self.pose.l_ankle_pitch.speed = 20
            self.update_pose(0.5)

        self.pose.l_ankle_roll.goal = 0
        self.pose.r_ankle_roll.goal = 0
        self.pose.l_ankle_roll.speed = 10
        self.pose.r_ankle_roll.speed = 10
        self.update_pose(1)

        self.animation_play(" walkready ")

    def walk_stairs( self , steps =1 , height =45):
        self.walk_init()
        for i in range (1 , steps +1):
            self.walk_step( height , i )