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

msg_twist=Twist()
linearx=0
angularz=0

goal=Point()

def is_line(person_array):

    #if on the same x axis

    

    #if on the same y axis

    #if diagonal pos

    #if diagonal negative




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




    br.sendTransform(t)



def listener():

    rospy.init_node("legs")
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    pub = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)
    
    
    rospy.spin()



if __name__ == '__main__':
    listener()