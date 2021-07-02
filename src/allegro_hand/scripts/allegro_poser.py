#!/usr/bin/python3
import rospy

from allegro_robot.allegro_hand_control import AllegroEnv

if __name__ == '__main__':
    allegro = AllegroEnv()   

    allegro.poses()