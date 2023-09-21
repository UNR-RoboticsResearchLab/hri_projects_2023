#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

global angle_best
global pose0
global pose1
global pose2
global pose3

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

def callback0(msg):
    global pose0
    pose0 = msg.pose.pose

def callback1(msg):
    global pose1
    pose1 = msg.pose.pose

def callback2(msg):
    global pose2
    pose2 = msg.pose.pose

def callback3(msg):
    global pose3
    pose3 = msg.pose.pose

def join_group():
    global angle_best 
    global pose1
    global pose2
    global pose3
    rospy.init_node('obstacle_avoidance', anonymous=True)

    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    # sub = rospy.Subscriber('base_scan', LaserScan, callback)
    sub0 = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, callback0)
    sub1 = rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry, callback1)
    sub2 = rospy.Subscriber('/robot_2/base_pose_ground_truth', Odometry, callback2)
    sub3 = rospy.Subscriber('/robot_3/base_pose_ground_truth', Odometry, callback3)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
            r1_pos = np.array([pose1.position.x,pose1.position.y,pose1.position.z])
            r2_pos = np.array([pose2.position.x,pose2.position.y,pose2.position.z])
            r3_pos = np.array([pose3.position.x,pose3.position.y,pose3.position.z])

            dist_r1_r3 = np.linalg.norm(r1_pos - r3_pos)
            goal = np.copy(r2_pos)
            goal[1]-= dist_r1_r3



        except NameError:
            print("pose not initiallized")

        
        # twist = Twist()
        # twist.linear.x=1
        # twist.angular.z = angle_best
        # pub.publish(twist)    
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        join_group()
    except rospy.ROSInterruptException:
        pass