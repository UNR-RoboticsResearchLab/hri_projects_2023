#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np
import tf2_geometry_msgs


global laser_scan

def to_odom(goal, tf_buffer):
    waiting = 1
    while waiting:
        try:
            odom = tf_buffer.lookup_transform('robot_0/odom', 'robot_0/base_link', rospy.Time())
            waiting = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    goal_pt = PointStamped()
    goal_pt.point.x = goal.transform.translation.x
    goal_pt.point.y = goal.transform.translation.y
    goal_pt.header.frame_id = 'robot_0/base_link'

    print(goal_pt.point.x, goal_pt.point.y)

    goal_pt = tf2_geometry_msgs.do_transform_point(goal_pt, odom)

    goal.header.frame_id='robot_0/odom'
    goal.transform.translation.x = goal_pt.point.x
    goal.transform.translation.y = goal_pt.point.y

    print(goal.transform.translation.x, goal.transform.translation.y)
    
    return goal

def get_transform(frame_id, tf_buffer):
    waiting = 1
    while waiting:
        try:
            transform = tf_buffer.lookup_transform('robot_0/base_link', frame_id, rospy.Time())
            waiting = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    return transform

def get_line_eq(a,b):
    m = (a.transform.translation.y - b.transform.translation.y)/(a.transform.translation.x - b.transform.translation.x)
    s = a.transform.translation.y - m*a.transform.translation.x
    
    return m,s

def detect_line(a,b,c):
    m,s = get_line_eq(a,b)
    c_predic = m*c.transform.translation.x + s
    c_diff = c.transform.translation.y - c_predic
    if(c_diff >= -1 and c_diff <= 1):
        return 1
    else:
        return 0
    
def detect_circle(a,b,c):
    a_arr = np.array([a.transform.translation.x,a.transform.translation.y])
    b_arr = np.array([b.transform.translation.x,b.transform.translation.y])
    c_arr = np.array([c.transform.translation.x,c.transform.translation.y])

    if(np.linalg.norm(a_arr - b_arr) <= 5 and np.linalg.norm(b_arr - c_arr) <= 5):
        return 1
    else:
        return 0
    
def create_transform(x,y):
    goal_t = TransformStamped()
    goal_t.transform.translation.x = x
    goal_t.transform.translation.y = y
    goal_t.transform.rotation.w = 1
    goal_t.header.frame_id = 'robot_0/base_link'
    goal_t.child_frame_id = 'goal'

    return goal_t

def get_line_goal(a,b,c):
    a_arr = np.array([a.transform.translation.x,a.transform.translation.y])
    b_arr = np.array([b.transform.translation.x,b.transform.translation.y])
    c_arr = np.array([c.transform.translation.x,c.transform.translation.y])
    coord_arr = np.array([a_arr,b_arr,c_arr])
    coord_arr = np.sort(coord_arr,axis=0)

    dist=np.average([np.linalg.norm(coord_arr[0]-coord_arr[1]),
                     np.linalg.norm(coord_arr[1]-coord_arr[2])])

    m,s = get_line_eq(a,b)

    theta = np.arctan(m)
    goal_x = np.cos(theta)*dist + coord_arr[2][0]    
    goal_y = m * goal_x + s

    goal_t = create_transform(goal_x, goal_y)

    return goal_t

def get_circle_goal(a,b,c):
    x_arr = np.array([a.transform.translation.x,b.transform.translation.x,c.transform.translation.x])
    y_arr = np.array([a.transform.translation.y,b.transform.translation.y,c.transform.translation.y])

    goal_x = np.average(x_arr)
    goal_y = np.average(y_arr)

    goal_t = create_transform(goal_x, goal_y)

    return goal_t

def detect_group():
    rospy.init_node('detect_goal', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(1) 
    global laser_scan
    laser_scan = LaserScan()

    person0 = get_transform('person0',tf_buffer)
    person1 = get_transform('person1',tf_buffer)
    person2 = get_transform('person2',tf_buffer)

    if(detect_line(person0,person1,person2)):
        print("here1")
        goal=get_line_goal(person0,person1,person2)
    elif(detect_circle(person0,person1,person2)):
        print("here2")
        goal=get_circle_goal(person0,person1,person2)
    else:
        print("here3")
        goal = create_transform(0,0)

    goal = to_odom(goal,tf_buffer)

    while not rospy.is_shutdown():
        goal.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(goal)
        rate.sleep()
    
    
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        detect_group()
    except rospy.ROSInterruptException:
        pass