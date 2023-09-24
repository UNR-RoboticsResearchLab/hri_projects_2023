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

msg=Twist()
case=''
linearx=0
angularz=0

goal=Point()
theta=0


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
    goal.x = t.transform.rotation.y

    print(goal.x)


def avoid_follow(data):

    rot_to_goal = atan2(goal.y, goal.x)

    range={
        "right" : min(min(data.ranges[0:239]) , 2),
        "center" : min(min(data.ranges[240:479]) , 2),
        "left" : min(min(data.ranges[480:719]) , 2)
    }

    if abs(rot_to_goal - theta) > 0.1:
        linearx = 0.0
        angularz = 0.3

        if ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
            case = 'something do be in front of me!'
            linearx=0
            angularz=-0.5
        else:
            linear.x = 0.0
            angular = 0.5

    pub.publish(msg)

def listener():
    global pub

    rospy.init_node('leggies_broadcaster')
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_leggies)
    
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    sub = rospy.Subscriber('/base_scan', LaserScan, avoid_follow)
    
    
    rospy.spin()



if __name__ == '__main__':
    listener()
