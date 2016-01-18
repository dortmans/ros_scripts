#!/usr/bin/env python
"""Simple mouse-based interface for sending goal locations to a
  turtlsim navigation node.

Author: Nathan Sprague
Version 1/25/15

"""

# Template borrowed from:
# http://www.dev-explorer.com/articles/python-with-curses

import curses
import rospy
from geometry_msgs.msg import PointStamped

class GoalTeleop(object):
    """ Teleop class. Handles curses display and ROS communication. """

    def __init__(self, screen):
        """ Set up the display etc. """

        self._point_pub = rospy.Publisher('/goal_point', PointStamped)
        rospy.init_node('mouse_goal_publisher')

        self._screen = screen
        curses.noecho()
        screen.keypad(1)
        screen.nodelay(1)
        curses.mousemask(1)

    def run(self):
        """ Main loop.  Executes until the user presses q. """
        point = PointStamped() 
        point.header.frame_id = "/odom"
        point.point.x = float("inf")
        
        while not rospy.is_shutdown():
            
            point.header.stamp = rospy.Time.now()
            self._screen.addstr(1, 1,
                                " Use the mouse to click a goal location.")
            self._screen.addstr(2, 1, "Press 'q' to quit.")

            event = self._screen.getch()
            if event == curses.KEY_MOUSE:
                _, mx, my, _, _ = curses.getmouse()
                size_y, size_x = self._screen.getmaxyx()
                relative_x = mx / float(size_x)
                relative_y = (1.0 - my / float(size_y))
                try:
                    self._screen.addstr(my, mx, "X")
                except curses.error:
                    pass
                self._screen.refresh()

                point.point.x = relative_x * 2.0 - 1.0
                point.point.y = relative_y * 2.0 - 1.0
                
                self._screen.clear()

            if event == ord("q"):
                break

            if point.point.x != float("inf"):
                self._point_pub.publish(point)
            rospy.sleep(.05)

        curses.endwin()


def launch(screen):
    """ Created so that curses.wrapper can be used. """
    teleop = GoalTeleop(screen)
    teleop.run()


if __name__ == "__main__":
    curses.wrapper(launch)
