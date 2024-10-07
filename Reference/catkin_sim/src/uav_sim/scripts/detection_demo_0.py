#!/usr/bin/python
#-*- encoding: utf8 -*-
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Int32,Float32MultiArray
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError


class DetectNode:

    def __init__(self):
        rospy.init_node('detection_node', anonymous=True)
        rospy.logwarn('detection node set up.')
        self.image_ = None
        self.bridge_ = CvBridge()
        self.rate=rospy.Rate(20)

        # 接收摄像头图像
        self.imageSub_ = rospy.Subscriber('/AKM_1/camera/rgb/image_raw', Image, self.imageCallback)  

        while not rospy.is_shutdown():
            self.GetMask()
            self.rate.sleep()
     

    def imageCallback(self, msg):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            print(err)
        
    def GetMask(self):
        if self.image_ is None:
            return
        print("接受到图片")
        print("The shape of image is ", self.image_.shape)


if __name__ == '__main__':
    cn = DetectNode()