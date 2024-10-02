#!/usr/bin/env python
#coding:utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("view", cv_image)

def showImage():
    rospy.init_node('showImage', anonymous = True)
    cv2.namedWindow("view", cv2.WINDOW_AUTOSIZE)
    cv2.startWindowThread()
    rospy.Subscriber('ShowImage', Image, callback)
    rospy.spin()
    cv2.destroyWindow("view")

if __name__ == '__main__':
    showImage()
