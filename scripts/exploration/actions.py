import math
import rospy
import numpy as np


class Actions:

    def __init__(self, core, datastore, movement_handler):
        self.core = core
        self.store = datastore
        self.movement = movement_handler

    def takeoff(self):
        sonar_height = self.store.get_sonar_height()
        if sonar_height is None:
            print('Waiting for sonar height data...')
            return False

        difference = self.core.config['hover_height'] - sonar_height

        if difference > 0.01:
            z_vel = np.sign(difference) * 0.3
            self.movement.request_velocity(lin_z=z_vel)
            return False

        self.movement.request_velocity(lin_z=0)
        return True

    def full_radial_scan(self, start_time):
        ang_vel = 0.2
        full_rotation_time = 2 * math.pi / ang_vel
        now = rospy.Time.now()
        duration = now - start_time
        if duration < rospy.Duration(full_rotation_time):
            self.movement.request_velocity(ang_z=ang_vel)
            return False
        else:
            self.movement.request_velocity(ang_z=0)
            return True
