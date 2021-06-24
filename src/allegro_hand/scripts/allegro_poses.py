#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy as copy
from IPython import embed
import yaml

SUB_TOPIC_NAME = '/allegroHand_0/joint_states'
PUB_TOPIC_NAME = '/allegroHand_0/joint_cmd'
MAX_ANGLE = 0.3

YAML_PATH = '/home/grail/catkin_ws/src/allegro_hand_ros_v4/src/allegro_hand_parameters/poses.yaml'

# actions = [
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1, 1.5, 0, 1, 1, 1]
# ]

class AllegroEnv(object):
    def __init__(self, max_action = MAX_ANGLE):
        try:
            rospy.init_node('test_node')
        except:
            pass

        self.pub = rospy.Publisher(PUB_TOPIC_NAME, JointState, queue_size=-1)
        rospy.Subscriber(SUB_TOPIC_NAME, JointState, self._sub_callback)
        self.current_joint_pose = None
        self.max_action = max_action

    def _sub_callback(self, data):
        self.current_joint_pose = data

    def step(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint pose read')
            return
        action = self._norm(action)
        current_angles = self.current_joint_pose.position
        desired_angles = np.array(action) 
        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])
        # print('desired JP:', desired_js.position)
        self.pub.publish(desired_js)

    def _norm(self, action):
        return np.clip(action, -self.max_action, self.max_action)

    def poses(self, action_array):

        for iterator in range(len(action_array)):
            print('This is pose', iterator)
            self.step(action_array[iterator])
            rospy.sleep(5)
    
        print('All poses done')

if __name__ == '__main__':

    allegro = AllegroEnv(max_action = 1.7)

    actions = []

    with open(YAML_PATH, 'r') as file:
        yaml_file = yaml.full_load(file)

        for key, array in yaml_file.items():
            print(array)
            actions.append(array)
    
    actions = np.array(actions)
    embed()
    # allegro.poses(actions)
