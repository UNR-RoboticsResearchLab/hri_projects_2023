#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
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

def go_to_person():
    rospy.init_node('go_to_person', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) # 10hz
    transform = TransformStamped()

    while not rospy.is_shutdown():
        twist = Twist()
        try:
            transform = tfBuffer.lookup_transform('base_link', 'person', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("waiting for transform")

        (roll,pitch,yaw) = euler_from_quaternion([transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w])

        print(np.linalg.norm(np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])))
        twist.linear.x = 1
        twist.angular.z = yaw

        # pub.publish(twist)    
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        go_to_person()
    except rospy.ROSInterruptException:
        pass