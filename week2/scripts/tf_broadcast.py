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
case=''
linearx=0
angularz=0

goal=Point()
theta=0

def new_odom(dat):
    global x
    global y
    global theta

    x = dat.pose.pose.position.x
    y = dat.pose.pose.position.y


def handle_leggies(msg):
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
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    theta=t.transform.rotation.z

    br.sendTransform(t)

    print(f"leggies found{t.transform.translation}")
    
    goal.x = t.transform.rotation.x
    goal.y = t.transform.rotation.y

    print(goal.x)

    
    linearx=0.0
    angularz=0.2

    msg_twist.linear.x = linearx
    msg_twist.angular.z = angularz

    pub.publish(msg_twist)



def listener():
    global pub

    rospy.init_node('leggies_broadcaster')
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    
    #rospy.Subscriber('/base_scan', LaserScan, avoid_follow)
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    
    rospy.spin()



if __name__ == '__main__':
    listener()
