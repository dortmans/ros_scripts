#!/usr/bin/env python
"""
This node demonstrates PID control by moving the Turtlebot
so that it maintains a fixed distance to a target.

Author: Nathan Sprague
Version:
"""
import rospy
import math
import pid
import numpy as np
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class PIDChaser(object):
    """
    This class represents a node that moves the Turtlebot based on
    the scan topic.

    Subscribes to:
      /scan                   - laser scan topic

    Publishes to:
    p_error, i_error, d_error - error terms for PID control.

    Required parameters:
    p_gain, i_gain, d_gain    - gain terms for the PID controller.
    """

    def __init__(self):
        """
        Set up the node.  There is no main loop, all publishing
        will happen in response to scan callbacks.
        """
        rospy.init_node('pid_chaser')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback,
                         queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
                                       queue_size=10)
        self.p_pub = rospy.Publisher('p_error', Float32, queue_size=10)
        self.i_pub = rospy.Publisher('i_error', Float32, queue_size=10)
        self.d_pub = rospy.Publisher('d_error', Float32, queue_size=10)

        # These should probably come from the parameter server as
        # well...
        self.target_distance = 1.25
        self.max_vel = .2

        self.twist = Twist()

        if (not rospy.has_param('p_gain') or
            not rospy.has_param('i_gain') or
            not rospy.has_param('d_gain')):
            rospy.logerr("PID Parameters not set.  Exiting")
            sys.exit(0)

        p_gain = rospy.get_param('p_gain')
        i_gain = rospy.get_param('i_gain')
        d_gain = rospy.get_param('d_gain')

        self.controller = pid.PID(p_gain, i_gain, d_gain, -.1, .1)

        rospy.spin()


    def mean_distance(self, scan):
        """
        Calculate free distance ahead as the average of a range of
        scan values.
        """
        mid_index = len(scan.ranges) // 2
        dists = [val for val in scan.ranges[mid_index-10:mid_index+10]
                 if not math.isnan(val)]
        return np.mean(dists)

    def scan_callback(self, scan):
        """
        Read the latest laser scan and publish an appropriate twist
        message to move the robot to the desired target distance.
        """
        dist = self.mean_distance(scan)
        if not math.isnan(dist):

            error = self.target_distance - dist
            self.controller.update_PID(error)
            self.twist.linear.x = -self.controller.cmd

            if self.twist.linear.x > self.max_vel:
                self.twist.linear.x = self.max_vel
            if self.twist.linear.x < -self.max_vel:
                self.twist.linear.x = -self.max_vel
        else:
            # just slow down...
            self.twist.linear.x = self.twist.linear.x * .8

        self.vel_pub.publish(self.twist)
        self.p_pub.publish(self.controller.p_error)
        self.i_pub.publish(self.controller.i_error)
        self.d_pub.publish(self.controller.d_error)

if __name__ == "__main__":
    PIDChaser()
