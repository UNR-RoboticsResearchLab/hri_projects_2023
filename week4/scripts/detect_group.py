#!/usr/bin/env python3

import rospy
import numpy as np
from people_msgs.msg import PositionMeasurementArray
from tf.transformations import quaternion_from_euler
import numpy as np
from tf2_ros import TransformBroadcaster
from week4.msg import Person

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

def detect_line(groups):
    a = groups[0]
    b = groups[1]
    c = groups[2]

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
    curr = 0
    people = []
    for group in groups:
        type = get_group_type(group)
        curr += 1



    return groups

def talker():
    rospy.init_node('group_detector', anonymous=True)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
    br = TransformBroadcaster()
    rate = rospy.Rate(1)

    global people

    while not rospy.is_shutdown():
        try:
            groups = detect_group(people)
            groups = get_group_type(groups)
            print(groups)
            print('\n\n')

        except NameError:
            continue
        rate.sleep()
        

# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

