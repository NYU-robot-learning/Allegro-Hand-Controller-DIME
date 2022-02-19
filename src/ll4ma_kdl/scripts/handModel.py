#!/usr/bin/env python

import rospy
import roslib
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
#from sensor_msgs.msg import JointState

import numpy as np
from math import sin, cos, pi
import time

_DISPLAY_RATE = 100
_EE_NAMES = ['index_biotac_surface', 'middle_biotac_surface', 'ring_biotac_surface', 'thumb_biotac_surface']
_EE_NAMES_BIOTAC_ORIGINS = ['index_biotac_origin', 'middle_biotac_origin', 'ring_biotac_origin', 'thumb_biotac_origin']
_J_NAMES=['index_link_0','index_link_1','index_link_2','index_tip','middle_link_0','middle_link_1','middle_link_2', 'middle_tip','ring_link_0','ring_link_1','ring_link_2', 'ring_tip','thumb_link_0','thumb_link_1','thumb_link_2', 'thumb_tip']
_INDEX_IDX = 0
_MIDDLE_IDX = 1
_RING_IDX = 2
_THUMB_IDX = 3
_NUM_FINGERS = 4
_NUM_FINGER_JOINTS = 4

_TEST_TORQUE_CONTROL=False
_TEST_IK_CONTROL=True

class HandModel:
    '''
    A model for the hand (allegro right) kinematics.
    '''
    def __init__(self, urdf_param_value):
        #        rospy.init_node('hand_model_node')

        # Load KDL trees for each hand finger
        self.robot = Robot.from_parameter_server(urdf_param_value)
        self.tree = kdl_tree_from_urdf_model(self.robot)
        #print '\nNum segements:',self.tree.getNrOfSegments(),'\n'

        task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist()

        #self.base_link = self.robot.get_root()
        self.base_link = 'palm_link'
        self.finger_chains = []
        self.finger_chains_biotac_origins = []
        self.joint_chains=[]
        for i, ee_name in enumerate(_EE_NAMES):
            fc = KDLKinematics(self.robot, self.base_link, ee_name)
            #fc.set_wdls_weights(weights_ts = task_space_ik_weights)
            #fc._ik_wdls_kdl.setLambda(0.003)
            self.finger_chains.append(fc)
            fc_origin = KDLKinematics(self.robot, self.base_link, _EE_NAMES_BIOTAC_ORIGINS[i])
            self.finger_chains_biotac_origins.append(fc_origin)
            #print 'Link', i, 'num joints:',fc.chain.getNrOfJoints()
        for i,j_name in enumerate(_J_NAMES):
            jc=KDLKinematics(self.robot,self.base_link,j_name)
            self.joint_chains.append(jc)
        
            

    def FK_joint(self,joint_angles,EE_index,j_index):
        '''
        Method to return task coordinates between palm link and any joint
        '''
        fi_x=self.joint_chains[EE_index*4+j_index].forward(joint_angles)

        return fi_x

    def FK(self, joint_angles, EE_index, end_link = None):
        '''
        Method to convert joint positions to task coordinates.
        @param end_link if None, the end of chain will be taken as end link
        @return 4x4 numpy.mat homogeneous transformation
        '''
        if end_link is not None and "biotac_origin" in end_link:
            return self.finger_chains_biotac_origins[EE_index].forward(joint_angles)

        return self.finger_chains[EE_index].forward(joint_angles, end_link)
    
    def Jacobian(self,joint_angles,EE_index):
        ji_x = self.finger_chains[EE_index].jacobian(joint_angles)
        return ji_x


    def IK(self, fingers_desired, finger=None):
        '''
        Get inverse kinematics for desired finger poses of all fingers
        fingers_desired - a list of desired finger tip poses in order of finger_chains[]
        returns - a list of lists of pyhton arrays of finger joint configurations
        TODO: Allow for single finger computation (named fingers)
        '''
        q_desired_fingers = []
        if finger is not None:
            # TODO: solve for a single finger...
            return None
        for i, f_d in enumerate(fingers_desired):
            q_desired_fingers.append(self.finger_chains[i].inverse_wdls(f_d))
            # q_desired_fingers.append(self.finger_chains[i].inverse(f_d))
        return q_desired_fingers
