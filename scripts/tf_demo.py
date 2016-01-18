#!/usr/bin/env python
"""Simple tf demo.  Print the current distance from the robot to the
origin of the odom frame.

Author: Nathan Sprague
Version: 11/2014
"""

import rospy
import tf
from geometry_msgs.msg import PointStamped
import math

class TFDemo(object):
    """ Simple tf demo node. """

    def __init__(self):
        """ Initialize the tf demo node. """
        rospy.init_node('tf_demo')

        self.tf_listener = tf.TransformListener()

        # Give the listener some time to accumulate transforms...
        rospy.sleep(1.0)

        while not rospy.is_shutdown():

            # Create the point (0,0,0) stamped with the robot's frame...
            point_base = PointStamped()
            point_base.header.frame_id = '/base_link'
            point_base.header.stamp = rospy.get_rostime()

            # Now transform the point into the /odom frame...
            try:
                # The available transforms may be running behind the time
                # stamp on the data.  tf will raise an extrapolation exception
                # if we ask it to transform a point with a "future" time stamp.
                # This call waits (up to 1.0 second) for the necessary
                # transform to become available.
                self.tf_listener.waitForTransform('/base_link', # from here
                                                  '/odom',     # to here
                                                  point_base.header.stamp,
                                                  rospy.Duration(1.0))

                point_odom = self.tf_listener.transformPoint('/odom',
                                                             point_base)

                cur_distance = math.sqrt(point_odom.point.x**2 +
                                         point_odom.point.y**2)

                rospy.loginfo("Distance from origin: {}".format(cur_distance))
                rospy.sleep(.5)

            except tf.Exception as e:
                print(e)
                print("(May not be a big deal.)")

if __name__ == '__main__':
    TFDemo()
