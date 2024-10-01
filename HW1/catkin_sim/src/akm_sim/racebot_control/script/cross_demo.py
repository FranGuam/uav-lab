#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class CrossDemo:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()
        self.direction = 0
        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name+'/odom', Odometry, self.odom_callback)
        self.Main()
        exit()
    
    def Main(self):
        self.turn(4.25, 2.95, "right")
        rospy.loginfo("Turn 1 completed!")
        while not self.is_in_target_area(3.5, 5, 2.5, 3):
            self.CarMove(1, 0)
            rospy.sleep(0.05)
            self.process_odom()
        self.CarMove(0, 0)
        rospy.loginfo("Straight 1 completed!")
        self.turn(1, 14.5, "left")
        rospy.loginfo("Turn 2 completed!")
        while not self.is_in_target_area(0, 1.5, 14, 15.5):
            self.CarMove(1, 0)
            rospy.sleep(0.05)
            self.process_odom()
        self.CarMove(0, 0)
        rospy.loginfo("Straight 2 completed!")
        rospy.loginfo("Racecar reached, stop!")

    def turn(self, target_x, target_y, direction):
        while abs(self.calc_target_angle(target_x, target_y) - self.direction) > 20:
            self.CarMove(1, 1 if direction == "left" else -1)
            rospy.sleep(0.05)
            self.process_odom()
        while abs(self.calc_target_angle(target_x, target_y) - self.direction) > 5:
            self.CarMove(0.3, 0.3 if direction == "left" else -0.3)
            rospy.sleep(0.05)
            self.process_odom()
        while abs(self.calc_target_angle(target_x, target_y) - self.direction) > 1:
            self.CarMove(0.1, 0.1 if direction == "left" else -0.1)
            rospy.sleep(0.05)
            self.process_odom()
        self.CarMove(0, 0)

    def calc_target_angle(self, x, y):
        delta_x = x - self.pose.position.x
        delta_y = y - self.pose.position.y
        return math.atan2(delta_y, delta_x) / math.pi * 180

    def is_in_target_area(self, x_min, x_max, y_min, y_max):
        return (x_min < self.pose.position.x < x_max) and (y_min < self.pose.position.y < y_max)

    def process_odom(self):
        euler = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        self.direction = euler[2] / math.pi * 180
        # print("x: " + str("%.2f" % self.pose.position.x) + " y: " + str("%.2f" % self.pose.position.y) + " direction: " + str("%.2f" % self.direction))
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmd_pub.publish(self.cmd_twist)

    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        CrossDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
    
