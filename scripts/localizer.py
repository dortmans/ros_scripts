#!/usr/bin/env python
""" Fake localization node.  

ROS navigation code typically expects 'map' to be the base coordinate
frame.  A functioning navigation system would figure out where the
robot is in the map and then communicate that location by publishing a
transform from 'map' to 'odom'.

This node fakes that process so that the tf tree will include a
map frame.  

The same thing could be accomplished with the static_transform_publisher 
tool:
  http://www.ros.org/wiki/tf#static_transform_publisher
We are using a Python node instead in case we want to use it as the
starting point for a -real- localization node in the future.

More info on ROS frame standards can be found here:
REP 105: http://www.ros.org/reps/rep-0105.html#coordinate-frames

Author: Nathan Sprague
Version: 2/13/2013
"""
import rospy
import tf
import numpy as np

class Localizer(object):
    """  This class may someday provide localization. 
    
    Currently it just broadcasts an identity transform from 'map' to
    'odom'.
    """
    
    def __init__(self):
        """ Initialize the localizer node. """
        
        rospy.init_node('localizer')
        br = tf.TransformBroadcaster()
 
        # Broadcast the transform at 10 HZ
        while not rospy.is_shutdown():
            br.sendTransform((0., 0., 0.), (0., 0. , 0. , 1.),
                             rospy.Time.now(),
                             "odom",
                             "map")
            rospy.sleep(.1)


if __name__ == '__main__':
    try:
        td = Localizer()
    except rospy.ROSInterruptException:
        pass
