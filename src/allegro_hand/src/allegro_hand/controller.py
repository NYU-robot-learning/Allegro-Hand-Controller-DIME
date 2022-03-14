# !/usr/bin/env python

# Basic imports
import os
import numpy as np
import yaml
import csv

# Other ROS imports
import rospy
from sensor_msgs.msg import JointState

# Other imports
from datetime import datetime
from copy import deepcopy as copy
from IPython import embed

# Put proper path
from allegro_hand.utils import *

# List of all ROS Topics
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
GRAV_COMP_TOPIC = '/allegroHand/grav_comp_torques' 
COMM_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'

# Maximum permitted values
MAX_ANGLE = 2.1
MAX_TORQUE = 0.3

DEFAULT_VAL = None

class AllegroController(object):
    def __init__(self):
        try:
            rospy.init_node('allegro_hand_node')
        except:
            pass

        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(GRAV_COMP_TOPIC, JointState, self._sub_callback_grav_comp)
        rospy.Subscriber(COMM_JOINT_STATE_TOPIC, JointState, self._sub_callback_cmd__joint_state)

        self.joint_comm_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=-1)

        self.current_joint_pose = DEFAULT_VAL
        self.grav_comp = DEFAULT_VAL
        self.cmd_joint_state = DEFAULT_VAL
        
    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_callback_grav_comp(self, data):
        self.grav_comp = data

    def _sub_callback_cmd__joint_state(self, data):
        self.cmd_joint_state = data
      
    def hand_pose(self, desired_action = np.zeros(16), absolute = True):
        if self.current_joint_pose == DEFAULT_VAL:
            print('No joint data received!')
            return

        action = self._clip(desired_action, MAX_ANGLE)
        current_angles = self.current_joint_pose.position

        if absolute is True:
            desired_angles = np.array(action)
        else:
            desired_angles = np.array(action) + np.array(current_angles)

        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])

        self.joint_comm_publisher.publish(desired_js)

    def apply_joint_torque(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint data received!')
            return

        action = self._clip(action, MAX_TORQUE)
        desired_torques = np.array(action)

        desired_js = copy(self.current_joint_pose)
        desired_js.position = list([])
        desired_js.effort = list(desired_torques)
        print('Applying the Desired Joint Torques:', desired_js.effort)
        self.joint_comm_publisher.publish(desired_js)

    def _clip(self, action, value):
        return np.clip(action, -value, value)

    def log_current_pose(self, log_file):
        if self.current_joint_pose is not DEFAULT_VAL:
            current_angles = self.current_joint_pose.position
            current_velocity = self.current_joint_pose.velocity
            current_torque = self.current_joint_pose.effort
        else:
            current_angles = DEFAULT_VAL
            current_velocity = DEFAULT_VAL
            current_torque = DEFAULT_VAL

        if self.grav_comp is not DEFAULT_VAL:
            grav_comp_torques = self.grav_comp.effort
        else: 
            grav_comp_torques = DEFAULT_VAL

        if self.cmd_joint_state is not DEFAULT_VAL:
            cmd_joint_position = self.cmd_joint_state.position
            cmd_joint_torque = self.cmd_joint_state.effort
        else:
            cmd_joint_position = DEFAULT_VAL
            cmd_joint_torque = DEFAULT_VAL

        time = get_datetime()

        print('Write done at:', time)

        with open(log_file, 'a') as csvfile:
            log_writer = csv.writer(csvfile, delimiter=' ')

            log_writer.writerow(
                [time]
                + [current_angles]
                + [current_velocity] 
                + [current_torque]
                + [grav_comp_torques]
                + [cmd_joint_position]
                + [cmd_joint_torque]
                )