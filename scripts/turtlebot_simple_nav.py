#!/usr/bin/env python
"""
Drive the turtlebot to an arbitrary point, without regard to obstacles.

UNFINISHED!

Author: Nathan Sprague
Version: 2/3/15
"""

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

class TurtleBotNav(object):
    """
    Node that moves a TurtleBot to a designated goal location using
    simple proportional control.

    Publishes to:
      /cmd_vel_mux/input/navi

    Subscribes to
      /goal_point
    """

    def __init__(self):
        """Set up the node. """

        rospy.init_node('turtlebot_nav')

        rospy.Subscriber("/goal_point", PointStamped, self.goal_callback)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)

        self.angular_gain = 1.5
        self.linear_gain = 1.0
        self.goal_point = None

        # Wait for the first goal message.
        while self.goal_point is None and not rospy.is_shutdown():
            rospy.sleep(.1)

        self.main_loop()

    def goal_callback(self, point):
        """ Store the goal point into an instance variable. """
        self.goal_point = point

    def main_loop(self):
        """ Continually navigate to the current goal point. """

        # BROKEN!  This information should come from the goal message.
        # We need to use tf to transform self.goal_point into the
        # base_link coordinate frame.
        # Goal location in /base_link coordinate frame:
        goalx = 1.0
        goaly = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            distance = math.sqrt(goalx**2 + goaly**2)
            if distance > .01:
                rospy.loginfo("Current Distance: {:.3f}".format(distance))

                angle = math.atan2(goaly, goalx)
                twist.angular.z = angle * self.angular_gain

                # Limit to a maximum velocity of pi/2...
                twist.angular.z = min(math.pi/2, twist.angular.z)

                # Don't move backwards...
                twist.linear.x = max(0, goalx * self.linear_gain)

                # Limit to a maximum velocity of .5...
                twist.linear.x = min(.5, twist.linear.x)

            self.pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    TurtleBotNav()
