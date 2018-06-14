#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

z_speed = 1.0
rot_speed = 1.0
active_keys = {}

def move():
    rospy.init_node('quadrotor_teleop', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        if key_down(keyboard.Key.space):
            vel_msg.linear.z += 1
        elif key_down(keyboard.Key.ctrl):
            vel_msg.linear.z -= 1

        if key_down(keyboard.Key.left):
            vel_msg.angular.z += 1
        elif key_down(keyboard.Key.right):
            vel_msg.angular.z -= 1

        if key_down(keyboard.Key.up):
            vel_msg.linear.x += 1
        elif key_down(keyboard.Key.down):
            vel_msg.linear.x -= 1

        velocity_publisher.publish(vel_msg)
        rate.sleep()


def key_down(key):
    val = active_keys.get(key)
    return val is not None


def on_press(key):
    active_keys[key] = True


def on_release(key):
    active_keys[key] = None
    if key == keyboard.Key.esc:
        listener.stop()


if __name__ == '__main__':

    print('Starting teleop script (press Ctrl+c to exit)...')
    print('Up: space, Down: ctrl, Direction: arrows.')

    try:
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        move()
    except rospy.ROSInterruptException:
        pass
