#!/usr/bin/env python3

import tf_conversions
import tf2_ros
import rospy
import math
import numpy as np
import circle_fit as cf
import geometry_msgs.msg
import copy
import scipy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import *
from week3.msg import Group
from math import atan2
from math import sqrt
from circle_fit import taubinSVD
from scipy.spatial import distance


msg_group=Group()
linearx=0
angularz=0
theta = 0.0
x=0.0
y=0.0

goal=Point()
speed=Twist()

move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
case=''

def findRadius(p1, p2):

    rad = sqrt(((p2[0] - p1[0])*(p2[0] - p1[0]))+((p2[1] - p1[1])*(p2[1] - p1[1])))

    return rad

def is_line(person_array):

    #print(len(person_array))

    det_arr = np.zeros(shape=(len(person_array),3))
    

    for i in range(len(person_array)):
        det_person_array = np.array([1, person_array[i].pos.x, person_array[i].pos.y])
        det_arr[i] = det_person_array
        #print(det_arr)

    det = np.linalg.det(det_arr)

    #print(det_arr)

    #if collinear
    print("Line Determinant: " + str(det))

    return det

def midpoint(p1, p2):
    mid_point = [0, 0]
    mid_point = [(p1[0]+p2[0])/2, (p1[1]+p2[1])/2]
    return mid_point

def longest_distance_in_line(person_array):
    point_coordinates = []

    if len(person_array) >= 2:

        for i in range(len(person_array)):
            person_point_array = [person_array[i].pos.x, person_array[i].pos.y]
            point_coordinates.append(person_point_array)

    point_dists = distance.cdist(point_coordinates, point_coordinates, 'euclidean')

    best_pair_index = np.unravel_index(point_dists.argmax(), point_dists.shape)

    print("Longest Distance: " + str(np.max(point_dists)))
    print("Best Pair: " + str(point_coordinates[best_pair_index[0]]) + str(point_coordinates[best_pair_index[1]]))

    line_midpoint = midpoint(point_coordinates[best_pair_index[0]],point_coordinates[best_pair_index[1]])

    print("Midpoint: " + str(line_midpoint))

    return np.max(point_dists), line_midpoint


#cite: https://github.com/AlliedToasters/circle-fit
def is_circle(person_array):

    point_coordinates = []

    if len(person_array) >= 2:

        for i in range(len(person_array)):
            person_point_array = [person_array[i].pos.x, person_array[i].pos.y]
            point_coordinates.append(person_point_array)
            #print(person_point_array)


    xc, yc, r, sigma = taubinSVD(point_coordinates)
            
    return xc, yc

def avoid_follow(dat):
    global case

    range={
        "right" : min(min(dat.ranges[0:239]) , 2),
        "center" : min(min(dat.ranges[240:479]) , 2),
        "left" : min(min(dat.ranges[480:719]) , 2)
    }

    if ( range["right"] >1  and range["center"] > 1 and range["left"] >1):
        case='free'
        return case
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        case='front'
        return case
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        case='right'
        return case
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        case='left'
        return case

def move_to_goal():
    global speed, theta,x,y
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
            move_pub.publish(speed)
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

        move_pub.publish(speed)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def handle_leggies(msg):
    global goal, msg_group
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "legs"

    if len(msg.people) == 0:
        print("No Legs Detected\n----")
        return None
    
    if len(msg.people) <= 2:
        print("Not enough people to clasify or not in valid classification. Total number detected: " + str(len(msg.people)) + "\n----")
        return None

    if len(msg.people) > 0:
        print("Total Number of People: "+ str(len(msg.people)))

        if abs(is_line(msg.people))<1.1:
            for i in range(len(msg.people)):
                msg.people[i].name = "line_1_" + msg.people[i].name
                print(msg.people[i].name)
            
            goal.x = msg.people[0].pos.x - 2.0
            goal.y = msg.people[0].pos.y
            move_to_goal()

            print("People in a line\n----")
        else:
            for i in range(len(msg.people)):
                msg.people[i].name = "circle_1_" + msg.people[i].name
                print(msg.people[i].name)

            c , r = is_circle(msg.people)
            goal.x = c
            goal.y = r
            move_to_goal()

            print(is_circle(msg.people))
            print("People in a circle\n----")


        for i in range(len(msg.people)):
            t.transform.translation.x = msg.people[i].pos.x
            t.transform.translation.y = msg.people[i].pos.y
            t.transform.translation.z = msg.people[i].pos.z
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            #print(f"person_{i} :" + str(t.transform))


    
        
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node("legs")


    rospy.Subscriber("/odom", Odometry, newOdom)
    rospy.Subscriber('/base_scan', LaserScan, avoid_follow)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    pub = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)

    # while(not rospy.is_shutdown()):
    #     print("here")
    #     msg_viz = line_marker(2)
    #     msg_viz.header.stamp = rospy.Time.now()
    #     marker_pub.publish(msg_viz)

    rospy.spin()
