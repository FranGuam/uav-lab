#!/usr/bin/env python
#coding:utf-8

import rospy
import sys
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def pubImage():
    rospy.init_node('pubImage', anonymous = True)
    pub = rospy.Publisher('ShowImage', Image, queue_size = 10)
    rate = rospy.Rate(10)
    bridge = CvBridge()
    get_image_name = []
    path = "/data/HW2/picture_data/dataset"
    for item in os.listdir(path):
        get_image_name.append(os.path.join(path, item))
    while not rospy.is_shutdown():
        for imagepath in get_image_name:
          image = cv2.imread(imagepath)
          image = cv2.resize(image, (900, 450))
          pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
          rate.sleep()

if __name__ == '__main__':
    try:
        pubImage()
    except rospy.ROSInterruptException:
        pass
