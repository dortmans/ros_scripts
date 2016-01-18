#!/usr/bin/env python

""" A very simple node for testing out the Python debugger """

from std_msgs.msg import String
import rospy

class SimpleNode(object):
    """ A simple node that listens on the chatter topic and echos it to the console """
    def __init__(self):
        rospy.init_node('simple_node')
        rospy.Subscriber('/chatter', String, self.process_chatter)

    def process_chatter(self, msg):
        print msg

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = SimpleNode()
    node.run()