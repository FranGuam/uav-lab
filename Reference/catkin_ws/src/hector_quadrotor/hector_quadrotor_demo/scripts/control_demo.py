#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class PublishCmdVel:
    def __init__(self, cmd_pub):
        self.cmd_pub = cmd_pub

    def cmdStop(self):    
        twist = Twist()
        self.cmd_pub.publish(twist)

    def cmdForward(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.x = 1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdBackward(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.x = -1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdUp(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.z = 1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdDown(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.z = -1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdLeft(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.y = 1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdRight(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.linear.y = -1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdTurnleft(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.angular.z = 1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()

    def cmdTurnright(self, speed=1.0, duration=1.0):    
        twist = Twist()
        twist.angular.z = -1.0 * speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmdStop()
    

if __name__=="__main__":
    try:
        rospy.init_node('control_demo')
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        while cmd_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for /cmd_vel connections...")
            rospy.sleep(1)
        rospy.loginfo("Connected to /cmd_vel")
        controller = PublishCmdVel(cmd_pub)

        controller.cmdUp(duration=2)
        controller.cmdForward(duration=1)
        controller.cmdRight(duration=1)       
        controller.cmdTurnright(duration=1)
  

    except rospy.ROSInterruptException:
        print("ROSInterruptException")
