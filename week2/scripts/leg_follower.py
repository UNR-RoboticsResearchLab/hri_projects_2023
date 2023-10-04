#!/usr/bin/env python3

import tf_conversions

import tf2_ros

import rospy
import math

import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0
goal=Point()
case=''

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def handle_leggies(msg):
    global goal

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "legs"

    if len(msg.people) == 0:
        print("no leggies")
        return None


    t.transform.translation.x = msg.people[0].pos.x
    t.transform.translation.y = msg.people[0].pos.y
    t.transform.translation.z = msg.people[0].pos.z
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    print(f"leggies found:{t.transform.translation.z}")

    goal.x = t.transform.translation.x
    goal.y = t.transform.translation.y

def avoid_follow(dat):
    global case

    range={
        "right" : min(min(dat.ranges[0:239]) , 2),
        "center" : min(min(dat.ranges[240:479]) , 2),
        "left" : min(min(dat.ranges[480:719]) , 2)
    }

    if ( range["right"] >1  and range["center"] > 1 and range["left"] >1):
        case='free'
        print("front free")
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        case='front'
        print("front wall")
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        case='right'
        print("right wall")
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        case='left'
        print("left wall")




rospy.init_node("speed_controller")

rospy.Subscriber("/odom", Odometry, newOdom)
rospy.Subscriber('/base_scan', LaserScan, avoid_follow)
rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

print(goal)

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y
    opt_theta = 0

    angle_to_goal = atan2(inc_y, inc_x)

    alpha = angle_to_goal - theta
    beta = angle_to_goal - theta +2*math.pi
    gamma = angle_to_goal - theta -2*math.pi

    if(abs(alpha) < abs(beta) and abs(alpha) < abs(gamma)):
        opt_theta = alpha
    elif(abs(beta) < abs(alpha) and abs(beta) < abs(gamma)):
        opt_theta = beta
    else:
        opt_theta = gamma

    if abs(angle_to_goal-theta)>0.1:
        while (abs(angle_to_goal - theta) > .1):
            speed.linear.x = 0
            speed.angular.z = opt_theta
            pub.publish(speed)
            rospy.sleep(0.1)
    else:
        if case=='free':
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        elif case=='front':
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif case=='right':
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif case=='left':
            speed.linear.x = 0.0
            speed.angular.z = 0.3

        pub.publish(speed)
        rospy.sleep(0.1)
