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
from circle_fit import taubinSVD
from scipy.spatial import distance


msg_group=Group()
msg_viz = Marker()
linearx=0
angularz=0
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

goal=Point()

def line_marker(size, midpoint):
    global msg_viz

    msg_viz.header.frame_id = "robot_0/odom"
    
    msg_viz.ns = "MARKER_VIZ"
    msg_viz.id = 0

    msg_viz.type = Marker.CUBE
    msg_viz.action = Marker.ADD

    msg_viz.pose.position.x = midpoint[0]
    print(msg_viz.pose.position.x)
    msg_viz.pose.position.y = midpoint[1]
    print(msg_viz.pose.position.y)
    msg_viz.pose.position.z = 0
    msg_viz.pose.orientation.x = 0.0
    msg_viz.pose.orientation.y = 0.0
    msg_viz.pose.orientation.z = 0.0
    msg_viz.pose.orientation.w = 1.0

    msg_viz.scale.x = size
    msg_viz.scale.y = 1.0
    msg_viz.scale.z = 1.0

    msg_viz.color.r = 0.0
    msg_viz.color.g = 1.0
    msg_viz.color.b = 0.0
    msg_viz.color.a = 1.0

    msg_viz.lifetime = rospy.Duration()

    msg_viz.frame_locked = True

    return msg_viz

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
            
    return xc


def handle_leggies(msg):
    global goal, msg_group, msg_viz, marker_pub
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
            msg_viz = line_marker(longest_distance_in_line(msg.people)[0], longest_distance_in_line(msg.people)[1])
            msg_viz.header.stamp = rospy.Time.now()
            marker_pub.publish(msg_viz)

            print("People in a line\n----")
        else:
            for i in range(len(msg.people)):
                msg.people[i].name = "circle_1_" + msg.people[i].name
                print(msg.people[i].name)
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

    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    pub = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)

    # while(not rospy.is_shutdown()):
    #     print("here")
    #     msg_viz = line_marker(2)
    #     msg_viz.header.stamp = rospy.Time.now()
    #     marker_pub.publish(msg_viz)

    rospy.spin()
