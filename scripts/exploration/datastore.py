import rospy
from sensor_msgs.msg import Range


class DataStore:

    def __init__(self):
        self.sonar_height = None

        rospy.Subscriber('/sonar_height', Range, self.sonar_height_callback)

    def get_sonar_height(self):
        return self.sonar_height

    def sonar_height_callback(self, height_data):
        self.sonar_height = height_data.range
