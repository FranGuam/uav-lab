from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
import cv2

class VideoDisplay(QLabel):
    def __init__(self, parent=None):
        super(VideoDisplay, self).__init__(parent)
        self.cap = cv2.VideoCapture(0)  # 打开摄像头或视频流
        
        # 设置显示区域大小
        self.setMinimumSize(640, 480)
        
        # 创建定时器用于更新视频帧
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms更新一次，约33fps
    
    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # 将OpenCV的BGR格式转换为RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 将numpy数组转换为QImage
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            # 显示图像
            self.setPixmap(QPixmap.fromImage(q_img))
    
    def closeEvent(self, event):
        self.cap.release() 