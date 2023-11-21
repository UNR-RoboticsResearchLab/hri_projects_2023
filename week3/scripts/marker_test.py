#!/usr/bin/env python3

import tf_conversions
import tf2_ros
import rospy
import math
import numpy as np
import circle_fit as cf
import geometry_msgs.msg
import copy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import *
from week3.msg import Group

msg_viz = Marker()

def line_marker(size):
    global msg_viz

    msg_viz.header.frame_id = "robot_0/base_link"
    
    msg_viz.ns = "MARKER_VIZ"
    msg_viz.id = 0

    msg_viz.type = Marker.CUBE
    msg_viz.action = Marker.ADD

    msg_viz.pose.position.x = 0
    msg_viz.pose.position.y = 0
    msg_viz.pose.position.z = 0
    msg_viz.pose.orientation.x = 0.0
    msg_viz.pose.orientation.y = 0.0
    msg_viz.pose.orientation.z = 0.0
    msg_viz.pose.orientation.w = 1.0

    msg_viz.scale.x = size
    msg_viz.scale.y = size
    msg_viz.scale.z = size

    msg_viz.color.r = 0.0
    msg_viz.color.g = 1.0
    msg_viz.color.b = 0.0
    msg_viz.color.a = 1.0

    msg_viz.lifetime = rospy.Duration()

    msg_viz.frame_locked = True

    print(msg_viz)


    return msg_viz



if __name__ == '__main__':
    rospy.init_node("legs")
    marker_pub = rospy.Publisher("/marker_pub", Marker, queue_size=10)

    while(not rospy.is_shutdown()):
        print("here")
        msg_viz = line_marker(2)
        msg_viz.header.stamp = rospy.Time.now()
        marker_pub.publish(msg_viz)
