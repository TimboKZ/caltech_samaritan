#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

HOVER_HEIGHT = 1.0

last_twist_msg = None
sonar_height = None


def force_hover():
    """
    Forces the robot to hover at a particular altitude. When planner outputs some velocity commands, this script tweaks
    them to ensure hover and publishes them to the drone.
    """
    global last_twist_msg
    global sonar_height

    rospy.init_node('force_hover', anonymous=True)
    rospy.Subscriber('/planned_cmd_vel', Twist, planned_cmd_vel_callback)
    rospy.Subscriber('/sonar_height', Range, sonar_height_callback)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        msg = Twist()
        msg.linear.z = 0

        if last_twist_msg is not None:
            rospy.loginfo('Using planned supplied Twist message!')
            msg.linear.x = last_twist_msg.linear.x
            msg.linear.y = last_twist_msg.linear.y
            msg.angular.x = last_twist_msg.angular.x
            msg.angular.y = last_twist_msg.angular.y
            msg.angular.z = last_twist_msg.angular.z
        else:
            msg.linear.x = 0
            msg.linear.y = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

        if sonar_height is not None:
            difference = HOVER_HEIGHT - sonar_height
            if abs(difference) > 0.01:
                msg.linear.z = difference

        rospy.loginfo('Sending Twist message...')
        velocity_publisher.publish(msg)

        last_twist_msg = None
        rate.sleep()


def planned_cmd_vel_callback(twist_msg):
    global last_twist_msg
    last_twist_msg = twist_msg


def sonar_height_callback(range_msg):
    global sonar_height
    sonar_height = range_msg.range


if __name__ == '__main__':
    force_hover()
