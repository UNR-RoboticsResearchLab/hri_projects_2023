#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

global angle_best

def callback(msg):
    angle_curr = msg.angle_min
    global angle_best
    angle_best = 0
    angle_best_dist = msg.range_min

    for i in msg.ranges:
        if(i > angle_best_dist):
            angle_best_dist = i
            angle_best = angle_curr
        angle_curr += msg.angle_increment

def square():
    global angle_best
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('base_scan', LaserScan, callback)
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x=1
        twist.angular.z = angle_best
        pub.publish(twist)    
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass