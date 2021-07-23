#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
# import time
from allegro_robot.allegro_hand_control import AllegroEnv

CAMERA_HAND_SUB = '/mediapipe_hand_joints'

class AllegroCameraHand(object):
    def __init__(self):
        self.allegro = AllegroEnv()

        rospy.Subscriber(CAMERA_HAND_SUB, JointState, self._sub_callback_camera_hand_joint_state)
        self.camera_hand_joint_state = JointState()       

    def _sub_callback_camera_hand_joint_state(self, data):
        self.camera_hand_joint_state = data 

    def camera_hand_movement(self):
        while(self.camera_hand_joint_state):
            desired_position = self.camera_hand_joint_state.position
            print(desired_position)
            self.allegro.pose_step(desired_position)
            rospy.sleep(0.05)


if __name__ == '__main__':
    handObject = AllegroCameraHand()
    handObject.camera_hand_movement()