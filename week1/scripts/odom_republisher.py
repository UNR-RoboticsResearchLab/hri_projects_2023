#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3, Pose, PoseWithCovariance 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

def callback(data):
    (roll,pitch,yaw) = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    roll = roll * 180 / np.pi
    pitch = pitch * 180 / np.pi
    yaw = yaw * 180 / np.pi
    print(roll, pitch, yaw)

def talker():
    sub = rospy.Subscriber('odom', Odometry, callback)
    rospy.init_node('odom_republisher', anonymous=True)
    
    rospy.spin()
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
