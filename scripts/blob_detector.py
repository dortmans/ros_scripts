#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class BlobDetector(object):
    """ The BlobDetector is a Python object that encompasses a ROS node 
        that can process images from the camera and search for blobs within """
    def __init__(self, image_topic):
        """ Initialize the blob detector """
        rospy.init_node('blob_detector')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('camera_image')             # a window for the latest camera image
        cv2.setMouseCallback('camera_image', self.process_mouse_event)
        rospy.Subscriber(image_topic, Image, self.process_image)
        cv2.namedWindow('image_info')               # a window to show color values of pixels

    def detect(self):
        """ Search for a blob in the last image found """
        if self.cv_image == None:
            return
        my_image = deepcopy(self.cv_image)

        for i in range(my_image.shape[0]):
            for j in range(my_image.shape[1]):
                # process pixels here
                pass

        # draw a red circle at the center of your blob
        cv2.circle(my_image, (int(0), int(0)), 5, (0,0,255),-1)
        cv2.imshow('tracking_window', my_image)
        cv2.waitKey(5)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("camera_image", self.cv_image)
        cv2.waitKey(5)

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.detect()
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window, 'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = BlobDetector("/camera/image_raw")
    node.run()