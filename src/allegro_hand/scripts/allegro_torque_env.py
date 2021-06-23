#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy as copy
from IPython import embed

SUB_TOPIC_NAME = '/allegroHand_0/joint_states'
PUB_TOPIC_NAME = '/allegroHand_0/joint_cmd'
MAX_TORQUE = 0.3

class AllegroEnv(object):
    def __init__(self):
        try:
            rospy.init_node('test_node')
        except:
            pass

        self.pub = rospy.Publisher(PUB_TOPIC_NAME, JointState, queue_size=-1)
        rospy.Subscriber(SUB_TOPIC_NAME, JointState, self._sub_callback)
        self.current_joint_pose = None

    def _sub_callback(self, data):
        self.current_joint_pose = data

    def step(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint pose read')
            return
        action = self._norm(action)
        desired_torques = np.array(action)
        desired_js = copy(self.current_joint_pose)
        desired_js.position = list([])
        desired_js.effort = list(desired_torques)
        print('desired torques:', desired_js.effort)
        self.pub.publish(desired_js)

    def _norm(self, action):
        return np.clip(action, -MAX_TORQUE, MAX_TORQUE)

if __name__ == '__main__':
    embed()