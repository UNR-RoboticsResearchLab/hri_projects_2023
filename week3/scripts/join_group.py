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
    vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    sub0 = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, callback0)
    sub1 = rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry, callback1)
    sub2 = rospy.Subscriber('/robot_2/base_pose_ground_truth', Odometry, callback2)
    sub3 = rospy.Subscriber('/robot_3/base_pose_ground_truth', Odometry, callback3)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) # 10hz

    waiting = True
    while waiting:
        try:
            r1_pos = np.array([pose1.position.x,pose1.position.y,pose1.position.z])
            r2_pos = np.array([pose2.position.x,pose2.position.y,pose2.position.z])
            r3_pos = np.array([pose3.position.x,pose3.position.y,pose3.position.z])

            dist_r1_r3 = np.linalg.norm(r1_pos - r3_pos)
            goal = np.copy(r2_pos)
            goal[1]-= dist_r1_r3

            goal_point = PoseStamped()
            goal_point.pose.position.x = goal[0]
            goal_point.pose.position.y = goal[1]
            goal_point.pose.position.z = goal[2]

            quaternion = quaternion_from_euler(0,0,np.pi/2)
            goal_point.pose.orientation.x = quaternion[0]
            goal_point.pose.orientation.y = quaternion[1]
            goal_point.pose.orientation.z = quaternion[2]
            goal_point.pose.orientation.w = quaternion[3]

            goal_point.header.frame_id = "robot_0/odom"

            marker = Marker()
            marker.header.frame_id = "robot_0/odom"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = 1
            marker.pose.position.y = 1
            marker.pose.position.z = 1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            

            waiting = False

        except NameError:
            continue

    while not rospy.is_shutdown():
        twist = Twist()
        r0_pos = np.array([pose0.position.x,pose0.position.y,pose0.position.z])
        r0_transform = TransformStamped()
        # r0_transform.child_frame_id = "robot0"
        # r0_transform.header.frame_id = "robot0/base_link"
        # r0_transform.transform.rotation = pose0.orientation
        # r0_transform.transform.translation = pose0.position
        try:
            r0_transform = tfBuffer.lookup_transform('robot_0/base_link', 'robot_0/odom', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("waiting for transform")
        
        # print(r0_transform.transform.translation)
        # print(r0_pos)
        transform = do_transform_pose(goal_point, r0_transform)
        # print(transform.pose.position)
        (roll, pitch, yaw) = euler_from_quaternion([transform.pose.orientation.x,transform.pose.orientation.y,transform.pose.orientation.z,transform.pose.orientation.w])
        twist.angular.z = yaw
        dist_r0_goal = np.linalg.norm(np.array([r0_transform.transform.translation.x,r0_transform.transform.translation.y,r0_transform.transform.translation.z]))
        twist.linear.x = dist_r0_goal
        
        # print(dist_r0_goal)
        print((yaw * 180)/np.pi)
        # print()
        pub.publish(twist)    
        vis_pub.publish( marker )
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        join_group()
    except rospy.ROSInterruptException:
        pass