#!/usr/bin/env python
import rospy

from allegro_robot.allegro_hand_control import AllegroEnv

if __name__ == '__main__':

    allegro = AllegroEnv()   

    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        allegro.log_current_pose()
        r.sleep()