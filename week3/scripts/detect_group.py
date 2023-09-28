#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np


global laser_scan

def get_transform(frame_id, tfBuffer):
    waiting = 1
    while waiting:
        try:
            transform = tfBuffer.lookup_transform('robot_0/base_link', frame_id, rospy.Time())
            waiting = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    return transform

def get_line_eq(a,b):
    m = (a.transform.translation.y - b.transform.translation.y)/(a.transform.translation.x - b.traansform.translation.x)
    s = a.transform.translation.y - m*a.transform.translation.x
    
    return m,s

def detect_line(a,b,c):
    m,s = get_line_eq(a,b)
    c_predic = m*c.transform.translation.x + b
    c_diff = c.transform.translation.y - c_predic
    if(c_diff >= -1 and c_diff <= 1):
        return 1
    else:
        return 0

def get_line_goal(a,b,c):
    a_arr = np.array([a.transform.translation.x,a.transform.translation.y])
    b_arr = np.array([b.transform.translation.x,b.transform.translation.y])
    c_arr = np.array([c.transform.translation.x,c.transform.translation.y])

    m,s = get_line_eq(a,b)
def detect_group():
    rospy.init_node('go_to_person', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) 
    transform = TransformStamped()
    global laser_scan
    laser_scan = LaserScan()
    

    while not rospy.is_shutdown():
        twist = Twist()
        waiting = 1

        person0 = get_transform('person0',tfBuffer)
        person1 = get_transform('person1',tfBuffer)
        person2 = get_transform('person2',tfBuffer)

        if(detect_line(person0,person1,person2)):
            goal=get_line_goal(person0,person1,person2)
        else:
            goal=get_circle_goal(person0,person1,person2)
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        detect_group()
    except rospy.ROSInterruptException:
        pass