from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROSVideoDisplay(QLabel):
    def __init__(self, parent=None):
        super(ROSVideoDisplay, self).__init__(parent)
        self.bridge = CvBridge()
        
        # 设置显示区域大小和样式
        self.setMinimumSize(640, 480)
        self.setMaximumSize(640, 480)
        self.setStyleSheet("""
            QLabel {
                border: 1px solid #ccc;
                border-radius: 5px;
                background-color: #f0f0f0;
            }
        """)
        self.setScaledContents(True)  # 允许图像缩放以适应标签大小
        
        # 订阅ROS图像话题
        self.image_sub = rospy.Subscriber(
            "/tello_image",
            Image,
            self.image_callback
        )
        
        self.latest_frame = None
        
        # 创建定时器用于更新显示
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms更新一次，约33fps
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 转换为RGB格式
            self.latest_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logerr(e)
    
    def update_frame(self):
        if self.latest_frame is not None:
            # 将numpy数组转换为QImage
            height, width, channel = self.latest_frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(self.latest_frame.data, width, height, 
                         bytes_per_line, QImage.Format_RGB888)
            # 显示图像
            self.setPixmap(QPixmap.fromImage(q_img))
    
    def closeEvent(self, event):
        # 关闭时清理资源
        self.timer.stop()
        if hasattr(self, 'image_sub'):
            self.image_sub.unregister()