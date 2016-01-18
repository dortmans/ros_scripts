#!/usr/bin/env python
""" 
Simple node to publish PointStamped messages at a random location
in order to practice working with tf. 

Messages are published to the /goal_point topic. 

Author: Nathan Sprague
Version: 2/3/2015
"""

import rospy
from geometry_msgs.msg import PointStamped
import random
import numpy as np

class GoalPublisher(object):
    """ 
    This class publishes a random goal position as a stamped point
    in the /odom frame of reference. 
    """
    
    def __init__(self):
        """ Initialize the goal publisher node."""
        
        self._point_pub = rospy.Publisher('/goal_point', PointStamped)
        rospy.init_node('goal_publisher')

        # Generate a random goal:
        self._goal = np.zeros(3)
        self._goal[0:2] = np.round(np.random.rand(2) * 4 - 2, 1)

        # Create the point message. 
        point = PointStamped()

        # Publish the goal at 10 HZ
        while not rospy.is_shutdown():
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "/odom"
            point.point.x = self._goal[0]
            point.point.y = self._goal[1]
            self._point_pub.publish(point)
            rospy.sleep(.1)

if __name__ == '__main__':
    GoalPublisher()
