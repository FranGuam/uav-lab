#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import cv2
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


INTERVAL = 0.005
LINEAR_SPEED_THRESHOLD = 0.01
ANGULAR_SPEED_THRESHOLD = 0.01

pose = Odometry().pose.pose
twist = Odometry().twist.twist
image = Image()
bridge = CvBridge()

def poseCallback(msg: Odometry):
    global pose, twist   
    pose = msg.pose.pose
    twist = msg.twist.twist
    return

def yaw(pose):
    euler =  euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return euler[2] # in (-pi, pi)

def angleDiff(current, target):
    angleDiff = target - current
    if angleDiff > math.pi:
        angleDiff -= 2 * math.pi
    elif angleDiff < -math.pi:
        angleDiff += 2 * math.pi
    return angleDiff

def direction(pose, target_x, target_y):
    return math.atan2(target_y - pose.position.y, target_x - pose.position.x)

def imageCallback(msg):
    try:
        global image
        image = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as err:
        rospy.logerr(err)

def detect(image, color_range_list):
    answer_list = []
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    for i in range(len(color_range_list)):
        color_range = color_range_list[i]
        mask = cv2.inRange(img_hsv, color_range[0], color_range[1])
        answer_list.append(cv2.countNonZero(mask))
    rospy.loginfo("answer_list: %s" % answer_list)
    return answer_list

def decision(answer_list, threshold):
    if max(answer_list) < threshold:
        return 'e'
    else:
        choices = ['r', 'y', 'b']
        return choices[answer_list.index(max(answer_list))]
        

class Controller:
    def __init__(self, cmd_pub):
        self.cmd_pub = cmd_pub

    def cmdStop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def cmdX(self, speed):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_pub.publish(twist)

    def cmdY(self, speed):
        twist = Twist()
        twist.linear.y = speed
        self.cmd_pub.publish(twist)

    def cmdZ(self, speed):
        twist = Twist()
        twist.linear.z = speed
        self.cmd_pub.publish(twist)

    def cmdTurn(self, speed):
        twist = Twist()
        twist.angular.z = speed
        self.cmd_pub.publish(twist)
    
    def stop(self, warning=False):
        if abs(twist.linear.x) > LINEAR_SPEED_THRESHOLD or abs(twist.linear.y) > LINEAR_SPEED_THRESHOLD or abs(twist.linear.z) > LINEAR_SPEED_THRESHOLD or abs(twist.angular.z) > ANGULAR_SPEED_THRESHOLD:
            if warning:
                rospy.logwarn("Drone still moving! Stop first...")
            self.cmdStop()
            while abs(twist.linear.x) > LINEAR_SPEED_THRESHOLD or abs(twist.linear.y) > LINEAR_SPEED_THRESHOLD or abs(twist.linear.z) > LINEAR_SPEED_THRESHOLD or abs(twist.angular.z) > ANGULAR_SPEED_THRESHOLD:
                rospy.sleep(INTERVAL)
    
    def log(self):
        print("x: %.3f, y: %.3f, z: %.3f, angle_z: %.4f\nvx: %.3f, vy: %.3f, vz: %.3f, wz: %.4f" % (pose.position.x, pose.position.y, pose.position.z, yaw(pose), twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z))
    
    def speedZ(self, velocity):
        rospy.loginfo("Z-axis Velocity: %.1f m/s" % velocity)
        # print("z: %.5f, vz: %.5f" % (pose.position.z, twist.linear.z))
        while abs(twist.linear.z - velocity) > LINEAR_SPEED_THRESHOLD:
            self.cmdZ(velocity)
            rospy.sleep(INTERVAL)
        # print("z: %.5f, vz: %.5f" % (pose.position.z, twist.linear.z))
    
    def moveZ(self, target):
        self.stop(True)
        rospy.loginfo("MoveZ to target height: %.2f m" % target)
        self.log()
        sign = 1 if pose.position.z < target else -1
        if (target - pose.position.z) * sign > 1.2:
            self.speedZ(1.0 * sign)
            while (target - pose.position.z) * sign > 0.65:
                self.cmdZ(1.0 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0.3 * sign)
            while (target - pose.position.z) * sign > 0.12:
                self.cmdZ(0.3 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0.1 * sign)
            while (target - pose.position.z) * sign > -0.007:
                self.cmdZ(0.1 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0)
        elif (target - pose.position.z) * sign > 0.3:
            self.speedZ(0.3 * sign)
            while (target - pose.position.z) * sign > 0.15:
                self.cmdZ(0.3 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0.1 * sign)
            while (target - pose.position.z) * sign > 0.01:
                self.cmdZ(0.1 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0)
        elif (target - pose.position.z) * sign > 0.05:
            self.speedZ(0.1 * sign)
            while (target - pose.position.z) * sign > 0.02:
                self.cmdZ(0.1 * sign)
                rospy.sleep(INTERVAL)
            self.speedZ(0)
        else:
            rospy.loginfo("Already at the target height!")
        rospy.loginfo("MoveZ to target height completed! Delta: %.3f" % (target - pose.position.z))
        self.log()
        return self.stop()
    
    def speedTurn(self, velocity):
        rospy.loginfo("Turn Velocity: %.1f rad/s" % velocity)
        # print("angle_z: %.5f, wz: %.5f" % (yaw(pose), twist.angular.z))
        while abs(twist.angular.z - velocity) > ANGULAR_SPEED_THRESHOLD:
            self.cmdTurn(velocity + (velocity - twist.angular.z) * 2)
            rospy.sleep(INTERVAL)
        self.cmdTurn(velocity)
        # print("angle_z: %.5f, wz: %.5f" % (yaw(pose), twist.angular.z))
    
    def turn(self, target):
        self.stop(True)
        rospy.loginfo("Turn to target orientation: %.4f rad" % target)
        self.log()
        sign = 1 if angleDiff(yaw(pose), target) > 0 else -1
        if angleDiff(yaw(pose), target) * sign > 0.25:
            self.speedTurn(0.5 * sign)
            while angleDiff(yaw(pose), target) * sign > 0.18:
                self.cmdTurn(0.5 * sign)
                rospy.sleep(INTERVAL)
            self.speedTurn(0.1 * sign)
            while angleDiff(yaw(pose), target) * sign > 0.04:
                self.cmdTurn(0.1 * sign)
                rospy.sleep(INTERVAL)
            self.speedTurn(0)
        elif angleDiff(yaw(pose), target) * sign > 0.01:
            self.speedTurn(0.1 * sign)
            while angleDiff(yaw(pose), target) * sign > 0.003:
                self.cmdTurn(0.1 * sign)
                rospy.sleep(INTERVAL)
            self.speedTurn(0)
        else:
            rospy.loginfo("Already at the target orientation!")
        rospy.loginfo("Turn to target orientation completed! Delta: %.4f" % angleDiff(yaw(pose), target))
        self.log()
        return self.stop()
    
    def turnTo(self, target_x, target_y):
        return self.turn(direction(pose, target_x, target_y))
    
    def moveXY(self, target_x, target_y, precise=False):
        self.stop(True)
        rospy.loginfo("MoveXY to target position: (%.2f, %.2f)" % (target_x, target_y))
        self.log()
        self.turnTo(target_x, target_y)
        while math.sqrt((target_x - pose.position.x) ** 2 + (target_y - pose.position.y) ** 2) > 0.3:
            self.cmdX(1)
            rospy.sleep(INTERVAL)
        self.stop()
        if precise:
            while math.sqrt((target_x - pose.position.x) ** 2 + (target_y - pose.position.y) ** 2) > 0.015:
                self.turnTo(target_x, target_y)
                while abs(angleDiff(yaw(pose), direction(pose, target_x, target_y))) < 0.05:
                    self.cmdX(0.1)
                self.stop()
        rospy.loginfo("MoveXY to target position completed! Delta: (%.3f, %.3f)" % (target_x - pose.position.x, target_y - pose.position.y))
        self.log()
        return self.stop()

    def moveXYZ(self, target_x, target_y, target_z, precise=False):
        self.stop(True)
        rospy.loginfo("Move to target position: (%.2f, %.2f, %.2f)" % (target_x, target_y, target_z))
        self.log()
        self.moveZ(target_z)
        self.moveXY(target_x, target_y, precise)
        if precise:
            self.moveZ(target_z)
        rospy.loginfo("Move to target position completed! Delta: (%.3f, %.3f, %.3f)" % (target_x - pose.position.x, target_y - pose.position.y, target_z - pose.position.z))
        self.log()
        return self.stop()

    def move(self, target_x, target_y, target_z, look_x, look_y, precise=False):
        self.stop(True)
        rospy.loginfo("Move to target position and orientation: (%.2f, %.2f, %.2f) to (%.2f, %.2f)" % (target_x, target_y, target_z, look_x, look_y))
        self.log()
        self.moveXYZ(target_x, target_y, target_z, precise)
        self.turnTo(look_x, look_y)
        if precise:
            self.turnTo(look_x, look_y)
        rospy.loginfo("Move to target position and orientation completed!")
        self.log()
        return self.stop()
    

def testZ(controller: Controller):
    # controller.speedZ(1.0)
    # # 0.300～0.790  2.119～2.609 delta 0.490
    # rospy.sleep(0.5)
    # controller.speedZ(0.3)
    # # 1.313～1.811  3.132～3.630 delta 0.498
    # rospy.sleep(0.5)
    # controller.speedZ(0.1)
    # # 1.965～2.076  3.788～3.899 delta 0.111
    # rospy.sleep(0.5)
    # controller.speedZ(0)
    # # 2.130～2.151  3.952～3.973 delta 0.021
    # rospy.sleep(5)
    # controller.speedZ(0)
    # # 2.131 2.119  3.953 3.941 offset 0.032

    # controller.speedZ(0.3)
    # # 0.300～0.412  0.760～0.872 delta 0.112
    # rospy.sleep(0.5)
    # controller.speedZ(0.1)
    # # 0.565～0.694  1.032～1.157 delta 0.127
    # rospy.sleep(0.5)
    # controller.speedZ(0)
    # # 0.746～0.770  1.212～1.235 delta 0.024
    # rospy.sleep(5)
    # controller.speedZ(0)
    # # 0.764 0.759  1.229 1.224 offset 0.011

    # controller.speedZ(0.1)
    # # 0.300～0.325  0.398～0.423 delta 0.025
    # rospy.sleep(0.5)
    # controller.speedZ(0)
    # # 0.375～0.400  0.474～0.499 delta 0.025
    # rospy.sleep(5)
    # controller.speedZ(0)
    # # 0.400 0.398  0.499 0.498 offset 0.0015

    # controller.speedZ(0)

    controller.moveZ(2.4) # 2.394
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(0.9) # 0.893
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(0.8) # 0.798
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(0.6) # 0.599
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(2.4) # 2.401
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(2.7) # 2.697
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(2.0) # 1.998
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(1.0) # 1.006
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(0.5) # 0.498
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")
    controller.moveZ(0.2) # 0.202
    rospy.sleep(10)
    controller.speedZ(0)
    rospy.logwarn("Stop")

def testTurn(controller: Controller):
    # controller.speedTurn(0.5)
    # # 1.5708～1.6293  2.2784～2.3367 -2.7958~-2.7375 delta 0.0584
    # rospy.sleep(3)
    # controller.speedTurn(0.1)
    # # 2.1049～2.1714  -3.0008～-2.9310 -1.3165~-1.2450 delta 0.0665~0.0715
    # rospy.sleep(1)
    # controller.speedTurn(0)
    # # 2.2626～2.2760  -2.8284～-2.8148 -1.1371~-1.1210 delta 0.0134~0.0161
    # rospy.sleep(10)
    # controller.speedTurn(0)
    # # 2.27832 -2.7959 -1.0919 offset 0.03

    # controller.speedTurn(0.1)
    # # 2.4381～2.4457  2.6555～2.6630 delta 0.0076
    # rospy.sleep(1)
    # controller.speedTurn(0)
    # # 2.6577～2.6685  2.7714～2.7837 delta 0.0108~0.0123
    # rospy.sleep(10)
    # controller.speedTurn(0)
    # # 2.6555 2.7727 offset 0.01

    controller.moveZ(1.5)

    controller.speedTurn(0)

    controller.turn(0)
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(3.0) # 3.03447
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-3.0) # -3.00066
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-2.5) # -2.53400
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-1.8) # -1.82086
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-0.8) # -0.80718
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(0.7) # 0.70942
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(2.7) # 2.71976
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-1.1) # -1.07034
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")

    controller.turn(0)
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-0.25) # 0.24664
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-0.335) # 0.33624
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    controller.turn(-0.35) # 0.35507
    rospy.sleep(10)
    controller.speedTurn(0)
    rospy.logwarn("Stop")
    
def testXY(controller: Controller):
    
    controller.moveZ(1)

    controller.moveXY(1, 1)
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(7, 2) # x: 6.878, y: 1.997
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(5, 1) # x: 5.014, y: 0.997
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(1, 2) # x: 1.027, y: 1.883
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(1, 1, True) # x: 0.992, y: 1.000
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(7, 2, True) # x: 6.998, y: 1.993
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(5, 1, True) # x: 5.007, y: 0.997
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")
    controller.moveXY(1, 2, True) # x: 1.003, y: 1.995
    rospy.sleep(10)
    controller.log()
    rospy.logwarn("Stop")

def crossWindow(controller):
    controller.move(1.75, 2, 2, 1.75, 3)
    answer_list = detect(image, [color_range_red])
    # Yes: 6175
    # No: 0
    if answer_list[0] > 1000:
        controller.moveXYZ(1.75, 2, 1)
        controller.moveXY(1.75, 4)
        return
    
    controller.move(4.25, 2, 2, 4.25, 3)
    answer_list = detect(image, [color_range_red])
    # Yes: 5066
    # No: 0
    if answer_list[0] > 1000:
        controller.moveXYZ(4.25, 2, 1)
        controller.moveXY(4.25, 4)
    else:
        controller.moveXYZ(6.75, 2, 1)
        controller.moveXY(6.75, 4)

def observe(controller):
    result = ['e', 'e', 'e', 'e', 'e']

    controller.moveXY(3.5, 4)
    controller.move(2, 7.5, 1, 3.5, 7.5) # 2
    answer_list = detect(image, [color_range_red, color_range_yellow, color_range_blue])
    # Red Yes: 6915 No: 0
    # Yellow Yes: 6740 No: 0
    # Blue Yes: 6626 No: 0
    result[1] = decision(answer_list, 1000)

    controller.moveXY(2, 12.5)
    controller.move(4, 12.5, 2, 4, 11) # 4
    answer_list = detect(image, [color_range_red, color_range_yellow, color_range_blue])
    # Red Yes: 6832 No: 108
    # Yellow Yes: 6673 No: 0
    # Blue Yes: 6673 No: 0
    result[3] = decision(answer_list, 1000)

    controller.turnTo(1, 14.5) # 5
    answer_list = detect(image, [color_range_red, color_range_yellow, color_range_blue])
    # Red Yes: 896 No: 0
    # Yellow Yes: 874 No: 0
    # Blue Yes: 875 No: 0
    result[4] = decision(answer_list, 500)

    controller.moveXY(5.5, 12.5)
    controller.move(6.5, 9.5, 2, 6.5, 7) # 1
    answer_list = detect(image, [color_range_red, color_range_yellow, color_range_blue])
    # Red Yes: 2371 No: 0
    # Yellow Yes: 2405 No: 0
    # Blue Yes: 2386 No: 0
    result[0] = decision(answer_list, 1000)

    controller.turnTo(5, 9.5) # 3
    answer_list = detect(image, [color_range_red, color_range_yellow, color_range_blue])
    # Red Yes: 5544 No: 10
    # Yellow Yes: 5476 No: 10
    # Blue Yes: 5399 No: 0
    result[2] = decision(answer_list, 1000)

    rospy.logwarn("result: %s" % result)
    return ''.join(result)

def land(controller):
    controller.moveXY(5.5, 12.5)
    controller.moveXY(7, 14.5)
    controller.moveZ(0.2)


if __name__=="__main__":
    try:
        rospy.init_node('control_fire')
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        state_sub = rospy.Subscriber('/ground_truth/state', Odometry, poseCallback, queue_size=1)
        while cmd_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for /cmd_vel connections...")
            rospy.sleep(1)
        rospy.loginfo("Connected to /cmd_vel")
        controller = Controller(cmd_pub)
        
        camera_sub = rospy.Subscriber('/front_cam/camera/image', Image, imageCallback)
        color_range_red = [(0, 120, 70), (10, 255, 255)]
        color_range_yellow = [(26, 43, 46), (34, 255, 255)]
        color_range_blue = [(100, 43, 46), (124, 255, 255)]

        result_pub = rospy.Publisher('/tello/target_result', String)

        # testZ(controller)
        # controller.moveZ(1.5)
        # testTurn(controller)
        # testXY(controller)

        crossWindow(controller)
        result_pub.publish(observe(controller))
        land(controller)

    except rospy.ROSInterruptException:
        print("ROSInterruptException")
