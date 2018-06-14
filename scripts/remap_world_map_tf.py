#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    """
    Remaps world --> base_footprint transform to map --> base_footprint.
    """
    rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base_footprint', 'world', rospy.Time.now())
            broadcaster.sendTransform(trans, rot, rospy.Time.now(), 'base_footprint', 'map')
            print('Published!')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
