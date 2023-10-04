#!/usr/bin/env python3

import tf_conversions

import tf2_ros

import rospy
import math
import numpy as np

import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
from math import atan2


msg_twist=Twist()
linearx=0
angularz=0

goal=Point()

def is_line(person_array):

    print(len(person_array))

    det_arr = np.zeros(shape=(len(person_array),3))
    

    for i in range(len(person_array)):
        det_person_array = np.array([1, person_array[i].pos.x, person_array[i].pos.y])
        det_arr[i] = det_person_array
        print(det_arr)

    det = np.linalg.det(det_arr)

    print(det_arr)

    #if collinear
    print(det)

    #if diagonal negative

    return det

def is_circle(person_array):



    return None

def circle_from_3_points(z1:complex, z2:complex, z3:complex) -> tuple[complex, float]:
    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f"Duplicate points: {z1}, {z2}, {z3}")
        
    w = (z3 - z1)/(z2 - z1)
    
    # You should change 0 to a small tolerance for floating point comparisons
    if abs(w.imag) <= 0:
        raise ValueError(f"Points are collinear: {z1}, {z2}, {z3}")
        
    c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1  # Simplified denominator
    r = abs(z1 - c)
    
    return c, r



def handle_leggies(msg):
    global msg_twist, goal
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "legs"

    if len(msg.people) == 0:
        print("no leggies")
        return None

    if len(msg.people) > 0:
        print(len(msg.people))

        for i in range(len(msg.people)):
            t.transform.translation.x = msg.people[i].pos.x
            t.transform.translation.y = msg.people[i].pos.y
            t.transform.translation.z = msg.people[i].pos.z
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            print(f"person_{i} :" + str(t.transform))
        
        if abs(is_line(msg.people))<1.1:
            print("people in a line")
        else:
            print("people not in a line")



    br.sendTransform(t)



def listener():

    rospy.init_node("legs")
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    pub = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)
    
    
    rospy.spin()



if __name__ == '__main__':
    listener()