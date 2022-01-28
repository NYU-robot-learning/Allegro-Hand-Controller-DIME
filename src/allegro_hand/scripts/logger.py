#!/usr/bin/python3
import os
import rospy

from allegro_hand.controller import AllegroController
from allegro_hand.utils import *

def logger(log_dir):
    # Checking if logging directory exists
    make_dir(log_dir)

    # Creating the log file
    log_file = os.path.join(log_dir, get_datetime()+'.csv')

    # Initializing controller
    allegro_controller = AllegroController()   

    # Logging the Joint State at 300 Hz
    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        allegro_controller.log_current_pose(log_file)
        r.sleep()

if __name__ == '__main__':
    logger()