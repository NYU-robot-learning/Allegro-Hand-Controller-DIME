#!/usr/bin/python3
import rospy
import yaml
import numpy as np

from allegro_hand.controller import AllegroController

# Yaml path for joint pose angles
YAML_PATH = '/home/sridhar/dexterous_arm/arm_stuff/src/Allegro-hand-controller-noetic/src/allegro_hand_parameters/poses.yaml'

def perform_poses(yaml_file):
    # Initializing the controller
    allegro_controller = AllegroController()

    # Loading the actions from YAML file
    actions = []
    with open(yaml_file, 'r') as file:
        yaml_file = yaml.full_load(file)
        for key, array in yaml_file.items():
            actions.append(array)

    actions = np.array(actions)

    # Performing all the actions
    for iterator in range(len(actions)):
        print('Hand is performing pose:', iterator + 1)
        allegro_controller.hand_pose(actions[iterator], True)
        
        print('Pausing so that the robot reaches the commanded hand pose...\n')
        rospy.sleep(5)

    print('Finished all the poses!')

if __name__ == '__main__':
    perform_poses(YAML_PATH)
