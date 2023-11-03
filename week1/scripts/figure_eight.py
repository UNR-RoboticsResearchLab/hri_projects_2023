#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3, Pose, PoseWithCovariance 
from nav_msgs.msg import Odometry
import numpy as np

def figure_eight():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('figure_eight', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        twist = Twist()
        for j in range(4):
            for i in range(10):
                twist.angular.z=0
                twist.linear.x=10
                rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()
            for i in range(10):
                twist.angular.z=np.pi/2
                twist.linear.x=0
                rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()

        for j in range(4):
            for i in range(10):
                twist.angular.z=0
                twist.linear.x=10
                rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()
            for i in range(10):
                twist.angular.z=-(np.pi/2)
                twist.linear.x=0
                rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()
        
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        figure_eight()
    except rospy.ROSInterruptException:
        pass
