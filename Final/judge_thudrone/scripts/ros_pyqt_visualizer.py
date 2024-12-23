'''
Author: Mrasamu
Date: 2024-12-08 01:14:54
LastEditors: Mrasamu
LastEditTime: 2024-12-14 11:36:44
description: file content
FilePath: /cursor/ros_pyqt_visualizer.py
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtCore import QTimer
from std_msgs.msg import String
from qt_gui import Ui_MainWindow
import yaml

class RosVisualizer(QMainWindow):
    def __init__(self):
        super(RosVisualizer, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.first_hit = True
        # 初始化ROS节点
        rospy.init_node('qt_visualizer', anonymous=True)
        
        # 创建订阅者
        self.subscriber = rospy.Subscriber('/judge', String, self.callback)
        self.score_subscriber = rospy.Subscriber('/score', String, self.score_callback)
        self.team_subscriber = rospy.Subscriber('/team_name', String, self.team_callback)
        
        # 创建定时器用于更新GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 每100ms更新一次
        
        # 创建定时器用于检查答案文件
        self.answer_timer = QTimer()
        self.answer_timer.timeout.connect(self.check_answer_file)
        self.answer_timer.start(1000)  # 每秒检查一次
        
        self.received_data = ""
        self.correct_answer = ""
        self.current_score = "0"
        self.team_name = "Team 1"
        self.last_modified_time = 0  # 记录文件最后修改时间
        self.config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')  # YAML配置文件路径
        
        # 定义球的样式
        self.ball_styles = {
            'R': '<span style="color: red; font-size: 30px;">&#9679;</span>',
            'B': '<span style="color: blue; font-size: 30px;">&#9679;</span>',
            'G': '<span style="color: green; font-size: 30px;">&#9679;</span>'
        }
        
        # 初始加载配置文件
        self.load_config_file()

    def format_answer(self, answer_str):
        """将字符串转换为带颜色球的HTML格式"""
        # 定义默认的灰色球样式（减小字体大小）
        default_style = '<span style="color: gray; font-size: 30px;">&#9679;</span>'
        
        # 创建两行显示：第一行显示彩色球，第二行显示原始字符
        balls_html = ""
        
        # 在原始字符之间添加三个空格，减小字符大小
        chars = list(answer_str.strip())
        chars_with_spaces = "   ".join(chars)
        chars_html = f'<span style="font-family: monospace; font-size: 16px;">{chars_with_spaces}</span>'
        
        for char in answer_str.strip():
            if char in self.ball_styles:
                balls_html += self.ball_styles[char] + " "
            else:
                balls_html += default_style + " "
        
        # 组合两行显示，减小行间距
        html_text = f'<div style="text-align: center;">{balls_html}</div>'
        html_text += f'<div style="text-align: center; margin-top: 5px;">{chars_html}</div>'
        
        return html_text

    def check_answer_file(self):
        """检查配置文件是否更新"""
        try:
            import os
            current_mtime = os.path.getmtime(self.config_file)
            if current_mtime > self.last_modified_time:
                self.load_config_file()
                self.last_modified_time = current_mtime
        except Exception as e:
            rospy.logerr(f"检查配置文件错误：{str(e)}")

    def load_config_file(self):
        """加载YAML配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                
            # 读取队伍名称和正确答案
            self.team_name = config.get('team_name', 'Team 1')
            self.correct_answer = config.get('correct_answer', '').strip()
            
            # 更新显示
            self.ui.teamDisplay.setText(self.team_name)
            formatted_answer = self.format_answer(self.correct_answer)
            self.ui.correctAnswer.setHtml(formatted_answer)
            
            # 更新最后修改时间
            import os
            self.last_modified_time = os.path.getmtime(self.config_file)
        except Exception as e:
            error_msg = f"读取配置文件错误：{str(e)}"
            self.ui.correctAnswer.setText(error_msg)
            rospy.logerr(error_msg)

    def callback(self, data):
        """ROS话题回调函数"""
        if self.first_hit:
            self.received_data = data.data
            self.first_hit = False
        else:
            pass

    def score_callback(self, data):
        """分数话题回调函数"""
        self.current_score = data.data

    def team_callback(self, data):
        """队伍名称话题回调函数"""
        self.team_name = data.data

    def update_gui(self):
        """更新GUI显示"""
        formatted_answer = self.format_answer(self.received_data)
        self.ui.playerAnswer.setHtml(formatted_answer)
        self.ui.scoreDisplay.setText(self.current_score)
        self.ui.teamDisplay.setText(self.team_name)  # 更新队伍名称显示
        
    def closeEvent(self, event):
        """程序关闭时的处理"""
        rospy.signal_shutdown('Qt GUI closed')
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = RosVisualizer()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
