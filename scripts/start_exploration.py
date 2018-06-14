#!/usr/bin/env python

import rospy
from exploration.core import Core, State

if __name__ == '__main__':

    print('Initialising core...')
    rospy.init_node('exploration_core', anonymous=True)
    core = Core()
    print('Done!')

    print('Control loop is running.')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        next_state = core.step()
        if next_state == State.Done:
            print('Exploration script has finished!')
            break
        rate.sleep()
