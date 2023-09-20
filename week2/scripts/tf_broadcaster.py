#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped 
from people_msgs.msg import PositionMeasurementArray
from tf.transformations import quaternion_from_euler
import numpy as np
from tf2_ros import TransformBroadcaster

def callback(data):
    best = 0
    best_id = 0
    curr = 0
    for person in data.people:
        if(person.reliability > best):
            best = person.reliability
            best_id = curr
        curr += 1
    br = TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "person"
    t.transform.translation.x=data.people[best_id].pos.x
    t.transform.translation.y=data.people[best_id].pos.y
    t.transform.translation.z=data.people[best_id].pos.z
    
    q = quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    print(len(data.people))
    br.sendTransform(t)

def talker():
    rospy.init_node('leg_tf_publisher', anonymous=True)
    sub = rospy.Subscriber('leg_tracker_measurements', PositionMeasurementArray, callback)
    
    rospy.spin()
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
