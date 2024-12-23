'''
Author: Mrasamu
Date: 2024-12-08 01:15:00
LastEditors: Mrasamu
LastEditTime: 2024-12-14 11:33:13
description: file content
FilePath: /cursor/qt_gui.py
'''
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from ros_video_display import ROSVideoDisplay

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(600, 300)
        MainWindow.setMinimumSize(600, 300)
        MainWindow.setMaximumSize(800, 400)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # 创建垂直布局
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        
        # 添加分数显示区域（移到最上面）
        self.scoreWidget = QtWidgets.QWidget()
        self.scoreLayout = QtWidgets.QHBoxLayout(self.scoreWidget)
        
        # 定义统一的标签样式
        label_style = """
            QLabel {
                font-size: 20px;
                font-weight: bold;
            }
        """
        
        # 队伍名称标签和显示
        self.teamLabel = QtWidgets.QLabel("队伍名称：")
        self.teamLabel.setStyleSheet(label_style)
        self.scoreLayout.addWidget(self.teamLabel)
        
        self.teamDisplay = QtWidgets.QLabel("Team 1")  # 默认显示
        self.teamDisplay.setStyleSheet("""
            QLabel {
                font-size: 24px;
                font-weight: bold;
                color: #000000;
                min-width: 150px;
                padding: 5px;
                border: 2px solid #ccc;
                border-radius: 5px;
                background-color: #f8f8f8;
            }
        """)
        self.scoreLayout.addWidget(self.teamDisplay)
        
        # 添加一些间距
        spacer = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.scoreLayout.addItem(spacer)
        
        # 分数标签和显示
        self.scoreLabel = QtWidgets.QLabel("选手得分：")
        self.scoreLabel.setStyleSheet(label_style)
        self.scoreLayout.addWidget(self.scoreLabel)
        
        self.scoreDisplay = QtWidgets.QLabel("0")
        self.scoreDisplay.setStyleSheet("""
            QLabel {
                font-size: 24px;
                font-weight: bold;
                color: #FF4500;
                min-width: 100px;
                padding: 5px;
                border: 2px solid #ccc;
                border-radius: 5px;
                background-color: #f8f8f8;
            }
        """)
        self.scoreLayout.addWidget(self.scoreDisplay)
        
        self.scoreLayout.addStretch()  # 添加弹性空间
        self.scoreWidget.setMaximumHeight(50)
        self.scoreLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.addWidget(self.scoreWidget)
        
        # 创建水平布局，用于并排显示答案组件
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.addLayout(self.horizontalLayout)
        
        # 左侧 - 正确答案部分
        self.leftWidget = QtWidgets.QWidget()
        self.leftLayout = QtWidgets.QVBoxLayout(self.leftWidget)
        
        # 正确答案标签
        self.correctLabel = QtWidgets.QLabel("正确答案")
        self.correctLabel.setStyleSheet(label_style)
        self.leftLayout.addWidget(self.correctLabel)
        
        self.correctAnswer = QtWidgets.QTextBrowser()
        self.correctAnswer.setStyleSheet("""
            QTextBrowser {
                background-color: white;
                border: 1px solid #ccc;
                border-radius: 5px;
                padding: 5px;
                font-size: 16px;
                line-height: 1.2;
            }
        """)
        self.leftLayout.addWidget(self.correctAnswer)
        
        # 右侧 - 选手答案部分
        self.rightWidget = QtWidgets.QWidget()
        self.rightLayout = QtWidgets.QVBoxLayout(self.rightWidget)
        
        # 选手答案标签
        self.playerLabel = QtWidgets.QLabel("选手答案")
        self.playerLabel.setStyleSheet(label_style)
        self.rightLayout.addWidget(self.playerLabel)
        
        self.playerAnswer = QtWidgets.QTextBrowser()
        self.playerAnswer.setStyleSheet("""
            QTextBrowser {
                background-color: white;
                border: 1px solid #ccc;
                border-radius: 5px;
                padding: 5px;
                font-size: 16px;
                line-height: 1.2;
            }
        """)
        self.rightLayout.addWidget(self.playerAnswer)
        
        # 将左右两部分添加到水平布局中
        self.horizontalLayout.addWidget(self.leftWidget)
        self.horizontalLayout.addWidget(self.rightWidget)
        
        # 添加视频显示区域
        self.videoWidget = QtWidgets.QWidget()
        self.videoLayout = QtWidgets.QVBoxLayout(self.videoWidget)
        self.videoLayout.setContentsMargins(0, 0, 0, 0)
        
        # 添加视频标签
        self.videoLabel = QtWidgets.QLabel("视频流")
        self.videoLabel.setStyleSheet(label_style)
        self.videoLayout.addWidget(self.videoLabel)
        
        # 添加ROS视频显示组件
        self.videoDisplay = ROSVideoDisplay()
        self.videoDisplay.setFixedSize(640, 480)  # 修改为 640x480
        self.videoLayout.addWidget(self.videoDisplay)
        
        self.verticalLayout.addWidget(self.videoWidget)
        
        # 调整窗口大小以适应视频显示
        MainWindow.resize(800, 700)  # 调整窗口高度
        MainWindow.setMinimumSize(800, 700)
        MainWindow.setMaximumSize(1200, 800)
        
        # 添加弹性空间
        self.verticalLayout.addStretch()
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        # 创建菜单栏
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        
        # 创建状态栏
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "ROS答案对比工具"))