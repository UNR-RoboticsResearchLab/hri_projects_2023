#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np


global laser_scan

def callback(msg):
    global laser_scan
    laser_scan = msg

def avoid_obstacle(yaw):
    global laser_scan
    angle_curr = laser_scan.angle_min
    num_left = 0
    num_right = 0
    for i in laser_scan.ranges:
        if(i <= .5):
            # if(angle_curr <= np.pi/2 and angle_curr >= -np.pi/2):
            if(angle_curr < 0):
                num_left += 1
            else:
                num_right += 0
        angle_curr += laser_scan.angle_increment
    
    if(num_right == 0 and num_left == 0):
        return yaw, 0
    elif(num_left > num_right):
        yaw = np.pi
    else:
        yaw = -np.pi
    return yaw, 1

def angle_to_goal(x,y):
    yaw = np.arctan2(y,x)
    return yaw

def go_to_person():
    rospy.init_node('join_group', anonymous=True)

    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, callback)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) 
    transform = TransformStamped()
    global laser_scan
    laser_scan = LaserScan()
    

    while not rospy.is_shutdown():
        twist = Twist()
        waiting = 1
        while waiting:
            try:
                transform = tfBuffer.lookup_transform('robot_0/base_link', 'goal', rospy.Time())
                waiting = 0
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            
        yaw = angle_to_goal(transform.transform.translation.x,transform.transform.translation.y)
       
        dist_from_goal = np.linalg.norm(np.array([transform.transform.translation.x,transform.transform.translation.y]))
        

        (yaw,obstacle) = avoid_obstacle(yaw)
        twist.angular.z = yaw / np.pi
        if(obstacle):
            twist.linear.x = 0

        if dist_from_goal > 1:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0
            twist.angular.z = 0

        pub.publish(twist)    
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        go_to_person()
    except rospy.ROSInterruptException:
        pass