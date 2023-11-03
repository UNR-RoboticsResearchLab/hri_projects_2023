#!/usr/bin/env python3

import rospy
import numpy as np
from people_msgs.msg import PositionMeasurementArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster
from week4.msg import Person, Group, GroupArray

global people

def callback(msg):
    global people
    people = []
    for person in msg.people:
        people.append(np.array([person.pos.x,person.pos.y,person.pos.z]))
    people = np.array(people)

def detect_group(people):
    groups = []
    for a in people:
        group = [a]
        for b in people:
            if (np.linalg.norm(a-b) < 10 and not np.array_equal(a,b)):
                group.append(b)
        if (len(group)>1):
            group = np.array(group)
            group = np.sort(group,axis = 0)
            groups.append(group)
    
    groups = np.array(groups)
    groups = np.unique(groups,axis=0)

    return groups

def get_line_eq(a,b):
    m = (a[1] - b[1])/(a[0] - b[0])
    s = a[1] - m*a[0]
    
    return m,s

def detect_line(group):
    a = group[0]
    b = group[1]
    c = group[2]

    m,s = get_line_eq(a,b)
    c_predic = m*c[0] + s
    c_diff = c[1] - c_predic
    if(c_diff >= -1 and c_diff <= 1):
        return 1
    else:
        return 0

def get_group_type(group):
    if(len(group) == 2 or detect_line(group)):
        return 'line'
    else:
        return 'circle'

def create_msg(groups):
    curr_g = 0
    group_msg_arr = GroupArray()
    for group in groups:
        people = []
        g_type = get_group_type(group)
        curr_p=0
        for person in group:
            id = g_type + '_'+str(curr_g) + '_person_' + str(curr_p)
            people.append(Person(id=id, x=person[0],y=person[1],z=person[2]))
            curr_p += 1
        g = Group()
        g.people=people
        g.size=len(people)
        g.type=g_type
        g.id = g_type + '_' + str(curr_g)
        g.center=get_group_center(people,g.size)
        g.header.stamp = rospy.Time().now()
        g.header.frame_id = 'robot_0/odom'
        group_msg_arr.groups.append(g)
        curr_g += 1

    return group_msg_arr

def get_group_center(group, size):
    x = 0
    y = 0
    z = 0
    for person in group:
        x += person.x
        y += person.y
        z += person.z
    x = x/size
    y = y/size
    z = z/size

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z

    p.orientation = Quaternion(x=0,y=0,z=0,w=1)

    return p

def get_transform(group):
    tf = TransformStamped()
    tf.header.frame_id = group.header.frame_id
    tf.header.stamp = rospy.Time().now()
    tf.transform.rotation = group.center.orientation
    tf.transform.translation.x = group.center.position.x
    tf.transform.translation.y = group.center.position.y
    tf.transform.translation.z = group.center.position.z
    tf.child_frame_id = group.id + '_center'

    return tf


def talker():
    rospy.init_node('group_detector', anonymous=True)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
    pub=rospy.Publisher('/robot_0/detected_groups', GroupArray, queue_size=10)
    br = TransformBroadcaster()
    rate = rospy.Rate(1)

    global people

    rospy.wait_for_message('/people_tracker_measurements', PositionMeasurementArray)

    while not rospy.is_shutdown():
        try:
            groups = detect_group(people)
            group_msg_arr = create_msg(groups)
            for group in group_msg_arr.groups:
                center_tf = get_transform(group)
                br.sendTransform(center_tf)
            pub.publish(group_msg_arr)
            
        except NameError:
            continue
        rate.sleep()
        

# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

