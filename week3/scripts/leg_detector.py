#!/usr/bin/env python3

import tf_conversions

import tf2_ros

import rospy
import math
import numpy as np
import circle_fit as cf


import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
from math import atan2
from circle_fit import taubinSVD


msg_twist=Twist()
linearx=0
angularz=0

goal=Point()

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
    global msg_twist, goal
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "legs"

    if len(msg.people) == 0:
        print("No Legs Detected")
        return None

    if len(msg.people) > 0:
        print("Total Number of People: "+ str(len(msg.people)))

        if abs(is_line(msg.people))<1.1:
            for i in range(len(msg.people)):
                msg.people[i].name = "line_1_" + msg.people[i].name
                print(msg.people[i].name)
            print("People in a line")
        else:
            for i in range(len(msg.people)):
                msg.people[i].name = "circle_1_" + msg.people[i].name
                print(msg.people[i].name)
            print(is_circle(msg.people))
            print("People in a circle")


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



def listener():

    rospy.init_node("legs")
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    pub = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)
    
    rospy.spin()



if __name__ == '__main__':
    listener()