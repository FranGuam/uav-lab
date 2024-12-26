# STEP1
连接飞机，启动tello_state节点
# STEP2
chmod +x demo.sh
./demo.sh
此时会中界面看到队名，答案，图像等，相关设置可以在config.yaml中修改
# STEP3
运行scripts中的score.py
向topic “/judge”发送类型为 std_msgs/String 的消息，界面会根据答案给出分数（起飞和降落为手动计分，同学们不用管，只确保自己能将识别结果发到“/judge”中即可）
注意裁判机只会接收一次识别结果，所以务必在识别全部完成之后统一发送，而且在命令行中打印，若裁判机没有收到会根据打印结果手动赋分

第一阶段评分：
起飞区出发：10；识别球：10x3（顺序请参照规则）；降落：10
举例：答案为“RGB”，接收到“RBG”，得分为10
第二阶段评分：
起飞区出发：10；识别球：15x2（顺序为A队放置区-B队放置区）；降落：10；
举例：答案为“RG”，接收到“RB”，得分为15