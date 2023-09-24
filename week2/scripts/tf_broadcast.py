#!/usr/bin/env python3  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import math
from people_msgs.msg import PositionMeasurementArray
from math import atan2
from nav_msgs.msg import Odometry

msg_twist=Twist()
linearx=0
angularz=0

theta=0

isBlocked=True
foundLegs=False
odomData=None

goal=Point()

def localRad(data):
    global odomData, theta
    
    odomData = data

    local_or = data.pose.pose.orientation
    or_list = [local_or.x, local_or.y, local_or.z, local_or.w]
    theta = local_or.z

def faceHuman(rad):
    global theta, odomData

    if(rad > odomData.pose.pose.orientation.z):
        while(theta<rad):
            msg_twist.linear.x=0
            msg_twist.angular.z=0.5
            pub.publish(msg_twist)
            rospy.sleep(0.1)
    else:
        while(theta > rad):
            msg_twist.linear.x=0
            msg_twist.angular.z=-0.5
            pub.publish(msg_twist)
            rospy.sleep(0.1)



def avoid_follow(dat):
    global isBlocked, linearx, angularz, msg_twist

    range={
        "right" : min(min(dat.ranges[0:239]) , 2),
        "center" : min(min(dat.ranges[240:479]) , 2),
        "left" : min(min(dat.ranges[480:719]) , 2)
    }

    if ( range["right"] >1  and range["center"] > 1 and range["left"] >1):
        isBlocked=False
        linearx=0.4
        angularz=0
        print("front free")
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        isBlocked=True
        linearx=0
        angularz=-0.5
        print("front wall")
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        isBlocked=True
        linearx=0
        angularz=0.5
        print("right wall")
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        isBlocked=True
        linearx=0
        angularz=-0.5
        print("left wall")

    msg_twist.linear.x=linearx
    msg_twist.angular.z=angularz
    pub.publish(msg_twist)


def handle_leggies(msg):
    global foundLegs, isBlocked, linearx, angularz, msg_twist
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "legs"

    if len(msg.people) == 0:
        print("no leggies")
        foundLegs=False
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
    
    faceHuman(t.transform.rotation.z)



def listener():
    global pub

    rospy.init_node('leggies_broadcaster')
    rospy.Subscriber('/odom', Odometry, localRad)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    rospy.Subscriber('/base_scan', LaserScan, avoid_follow)
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    
    rospy.spin()



if __name__ == '__main__':
    listener()
