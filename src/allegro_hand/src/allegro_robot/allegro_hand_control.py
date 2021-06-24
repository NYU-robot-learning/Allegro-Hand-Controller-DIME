# !/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy as copy
from IPython import embed
import os
from datetime import datetime
import yaml
import csv


# List of all subscribers
SUB_TOPIC_NAME_1 = '/allegroHand_0/joint_states'
SUB_TOPIC_NAME_2 = '/allegroHand/grav_comp_torques'
SUB_TOPIC_NAME_3 = '/allegroHand/commanded_joint_states'
PUB_TOPIC_NAME = '/allegroHand_0/joint_cmd'

# Max values
MAX_ANGLE = 1.7
MAX_TORQUE = 0.3

# Yaml path for joint pose angles
YAML_PATH = '/home/grail/catkin_ws/src/allegro_hand_ros_v4/src/allegro_hand_parameters/poses.yaml'

# Path for logging movements
LOG_PATH = '/home/grail/catkin_ws/src/allegro_hand_ros_v4/src/allegro_hand/log' 

DEFAULT_VAL = None

# Utility functions
# Function to check and create a directory if not found
def make_dir(folder):
    if not os.path.exists(folder):
        print('Making Directory: {}'.format(folder))
        os.makedirs(folder)
    else:
        print('Directory Exisits : {}'.format(folder))

# Function to get current date and time
def get_datetime():
    now = datetime.now()
    return now.strftime('%Y-%m-%d-%H-%M-%S-%f')

class AllegroEnv(object):
    def __init__(self, log_folder = LOG_PATH):
        try:
            rospy.init_node('allegro_node')
        except:
            pass

        rospy.Subscriber(SUB_TOPIC_NAME_1, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(SUB_TOPIC_NAME_2, JointState, self._sub_callback_grav_comp)
        rospy.Subscriber(SUB_TOPIC_NAME_3, JointState, self._sub_callback_cmd__joint_state)

        self.pub = rospy.Publisher(PUB_TOPIC_NAME, JointState, queue_size=-1)

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
      
    def step_joint_angle(self, action=np.zeros(16)):
        if self.current_joint_pose == DEFAULT_VAL:
            print('No joint pose read')
            return
        action = self._norm(action, MAX_ANGLE)
        current_angles = self.current_joint_pose.position
        desired_angles = np.array(action) + np.array(current_angles)
        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])
        print('Desired Joint Position:', desired_js.position)
        self.pub.publish(desired_js)

    def step_joint_torque(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint pose read')
            return
        action = self._norm(action, MAX_TORQUE)
        desired_torques = np.array(action)
        desired_js = copy(self.current_joint_pose)
        desired_js.position = list([])
        desired_js.effort = list(desired_torques)
        print('Desired Joint Torques:', desired_js.effort)
        self.pub.publish(desired_js)

    def pose_step(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint pose read')
            return
        action = self._norm(action, MAX_ANGLE)
        current_angles = self.current_joint_pose.position
        desired_angles = np.array(action) 
        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])
        self.pub.publish(desired_js)

    def poses(self):
        actions = []

        with open(YAML_PATH, 'r') as file:
            yaml_file = yaml.full_load(file)

            for key, array in yaml_file.items():
                actions.append(array)

        actions = np.array(actions)

        for iterator in range(len(actions)):
            print('This is pose number', iterator + 1)
            self.pose_step(actions[iterator])
            print('Pausing for next action! - (5 secs)')
            rospy.sleep(5)

        print('All poses done')

    def _norm(self, action, value):
        return np.clip(action, -value, value)

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
