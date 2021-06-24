#!/usr/bin/env python
import rospy
from datetime import datetime
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy as copy
from IPython import embed
import os
import csv

# List of all subscribers
SUB_TOPIC_NAME_1 = '/allegroHand_0/joint_states'
SUB_TOPIC_NAME_2 = '/allegroHand/grav_comp_torques'
SUB_TOPIC_NAME_3 = '/allegroHand/commanded_joint_states'

DEFAULT_VAL = None

# Function to check and create a directory if not found
def make_dir(folder):
    if not os.path.exists(folder):
        print('Making Directory: {}'.format(folder))
        os.makedirs(folder)
    else:
        print('Exisiting Directory: {}'.format(folder))


def get_datetime():
    now = datetime.now()
    return now.strftime('%Y-%m-%d-%H-%M-%S-%f')

# Logger object definition
class Logger(object):
    def __init__(self, log_folder):
        try:
            rospy.init_node('logger')
        except:
            pass

        rospy.Subscriber(SUB_TOPIC_NAME_1, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(SUB_TOPIC_NAME_2, JointState, self._sub_callback_grav_comp)
        rospy.Subscriber(SUB_TOPIC_NAME_3, JointState, self._sub_callback_cmd__joint_state)

        self.current_joint_pose = DEFAULT_VAL
        self.grav_comp = DEFAULT_VAL
        self.cmd_joint_state = DEFAULT_VAL
        
        self.log_folder = log_folder
        make_dir(self.log_folder)
        
        self.log_file = os.path.join(self.log_folder, get_datetime()+'.csv')

    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_callback_grav_comp(self, data):
        self.grav_comp = data

    def _sub_callback_cmd__joint_state(self, data):
        self.cmd_joint_state = data

    # Function to log the current parameters
    def log_current_pose(self):
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

        with open(self.log_file, 'a') as csvfile:
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


    def _norm(self, action):
        return np.clip(action, -MAX_ANGLE, MAX_ANGLE)

if __name__ == '__main__':
    log_path = '/home/grail/catkin_ws/src/allegro_hand_ros_v4/src/allegro_hand/log'    
    L = Logger(log_path)

    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        L.log_current_pose()
        r.sleep()