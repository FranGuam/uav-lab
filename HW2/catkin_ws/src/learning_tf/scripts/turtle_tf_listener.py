#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

def follow(tf_listener: tf.listener, follower: str, leader: str) -> geometry_msgs.msg.Twist:
    (trans,rot) = tf_listener.lookupTransform(follower, leader, rospy.Time(0))
    angular = 4 * math.atan2(trans[1], trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    return cmd
    
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    spawner(8, 10, 0, 'turtle3')
    
    turtle_vel2 = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    turtle_vel3 = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            turtle_vel2.publish(follow(listener, 'turtle2', 'turtle1'))
            turtle_vel3.publish(follow(listener, 'turtle3', 'turtle2'))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
