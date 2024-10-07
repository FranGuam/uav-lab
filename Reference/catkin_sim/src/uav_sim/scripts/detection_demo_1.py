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
        self.color_range_ = [(0, 43, 46), (10, 255, 255)]# 红色的HSV范围
        self.bridge_ = CvBridge()
        self.rate=rospy.Rate(20)
        # 接收摄像头图像
        self.imageSub_ = rospy.Subscriber('/AKM_1/camera/rgb/image_raw', Image, self.imageCallback)  
        self.maskPub = rospy.Publisher('/AKM_1/mask', Image, queue_size=10)

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
        #self.image_ = cv2.GaussianBlur(self.image_.copy(), (3, 3), 0)  # 高斯模糊
        # 将图片转换到HSV空间
        image_hsv_orin = cv2.cvtColor(self.image_.copy(), cv2.COLOR_BGR2HSV)  
        image_hsv = image_hsv_orin[:,:,:]
        height = image_hsv.shape[0]
        width = image_hsv.shape[1]

        data_red=np.where( (image_hsv[:,:,0]>=self.color_range_[0][0] )& (image_hsv[:,:,0]<=self.color_range_[1][0]) \
                        & (image_hsv[:,:,1]>=self.color_range_[0][1]) & (image_hsv[:,:,1]<=self.color_range_[1][1]) \
                        & (image_hsv[:,:,2]>=self.color_range_[0][2]) & (image_hsv[:,:,2]<=self.color_range_[1][2]) )

        center=data_red[1].sum()
        img_target=cv2.inRange(image_hsv_orin,(self.color_range_[0][0],self.color_range_[0][1],self.color_range_[0][2]),(self.color_range_[1][0],self.color_range_[1][1],self.color_range_[1][2]))
        #输入图像与输入图像在掩模条件下按位与，得到掩模范围内的原图像
        img_specifiedColor=cv2.bitwise_and(self.image_.copy(),self.image_.copy(),mask=img_target)

        mask_msg = self.bridge_.cv2_to_imgmsg(img_specifiedColor, "bgr8")
        self.maskPub.publish(mask_msg)


if __name__ == '__main__':
    cn = DetectNode()
