#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


localization = [0,0,0,0,0,0]

def poseCallback(msg):
    global localization
    localization = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]
    pass

def distance(orin_local,final_local):
    global localization
    result = (orin_local[0]-final_local[0])**2 + (orin_local[1]-final_local[1])**2 + (orin_local[2]-final_local[2])**2
    return result
        

class PublishCmdVel:
    def __init__(self, cmd_pub):
        self.cmd_pub = cmd_pub

    def cmdStop(self):    
        twist = Twist()
        self.cmd_pub.publish(twist)

    def cmdForward(self, speed=1.0, duration=1.0):  
        global localization  
        twist = Twist()
        twist.linear.x = 1.0 * speed
        orin_local = localization.copy()  
        while distance(orin_local,localization) < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdBackward(self, speed=1.0, duration=1.0):  
        global localization
        orin_local = localization.copy()      
        twist = Twist()
        twist.linear.x = -1.0 * speed
        while distance(orin_local,localization) < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdUp(self, speed=1.0, duration=1.0):    
        global localization
        orin_local = localization.copy()  
        twist = Twist()
        twist.linear.z = 1.0 * speed
        while distance(orin_local,localization) < duration:
            #print(distance(orin_local,localization))
            self.cmd_pub.publish(twist)
        #rospy.sleep(duration)
        self.cmdStop()

    def cmdDown(self, speed=1.0, duration=1.0):  
        global localization
        orin_local = localization.copy()      
        twist = Twist()
        twist.linear.z = -1.0 * speed
        while distance(orin_local,localization) < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdLeft(self, speed=1.0, duration=1.0):    
        global localization
        orin_local = localization.copy()    
        twist = Twist()
        twist.linear.y = 1.0 * speed
        while distance(orin_local,localization) < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdRight(self, speed=1.0, duration=1.0):    
        global localization
        orin_local = localization.copy()    
        twist = Twist()
        twist.linear.y = -1.0 * speed
        while distance(orin_local,localization) < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdTurnleft(self, speed=1.0, duration=1.0):
        global localization
        orin_local = localization.copy()       
        twist = Twist()
        twist.angular.z = 1.0 * speed
        while orin_local[5]-localization[5] < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()

    def cmdTurnright(self, speed=1.0, duration=1.0):    
        global localization
        orin_local = localization.copy()   
        twist = Twist()
        twist.angular.z = -1.0 * speed
        while localization[5]-orin_local[5] < duration:
            self.cmd_pub.publish(twist)
        self.cmdStop()
    

if __name__=="__main__":
    try:
        rospy.init_node('control_demo')
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        state_sub = rospy.Subscriber('/ground_truth/state', Odometry, poseCallback)
        while cmd_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for /cmd_vel connections...")
            rospy.sleep(1)
        rospy.loginfo("Connected to /cmd_vel")
        controller = PublishCmdVel(cmd_pub)

        controller.cmdUp(duration=1)
        controller.cmdForward(duration=0.5)
        controller.cmdRight(duration=0.5)       
        controller.cmdTurnleft(duration=0.1)
  

    except rospy.ROSInterruptException:
        print("ROSInterruptException")
