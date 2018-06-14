import rospy
from geometry_msgs.msg import Twist


class MovementHandler:

    def __init__(self, core, datastore):
        self.state = None
        self.core = core
        self.store = datastore

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_vel_msg = None

    def fix_hover(self):
        sonar_height = self.store.get_sonar_height()
        if sonar_height is not None:
            hover_height = self.core.config['hover_height']
            difference = hover_height - sonar_height
            if abs(difference) > 0.01:
                self.request_velocity(lin_z=difference)

    def start_step(self, state):

        # Initialise a new velocity message for the current step
        self.current_vel_msg = Twist()
        self.current_vel_msg.linear.x = 0
        self.current_vel_msg.linear.y = 0
        self.current_vel_msg.linear.z = 0
        self.current_vel_msg.angular.x = 0
        self.current_vel_msg.angular.y = 0
        self.current_vel_msg.angular.z = 0

        self.state = state

    def finish_step(self):

        # Adjust altitude of the drone if necessary
        if self.state not in self.core.config['exclude_from_fix_hover']:
            self.fix_hover()

        self.velocity_publisher.publish(self.current_vel_msg)
        self.state = None

    def request_velocity(self, lin_x=None, lin_y=None, lin_z=None, ang_x=None, ang_y=None, ang_z=None):
        if self.current_vel_msg is None:
            return None

        if lin_x is not None:
            self.current_vel_msg.linear.x = lin_x
        if lin_y is not None:
            self.current_vel_msg.linear.y = lin_y
        if lin_z is not None:
            self.current_vel_msg.linear.z = lin_z
        if ang_x is not None:
            self.current_vel_msg.angular.x = ang_x
        if ang_y is not None:
            self.current_vel_msg.angular.y = ang_y
        if ang_z is not None:
            self.current_vel_msg.angular.z = ang_z

        return self.current_vel_msg
