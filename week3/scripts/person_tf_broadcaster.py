#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped 
from people_msgs.msg import PositionMeasurementArray
from tf.transformations import quaternion_from_euler
import numpy as np
from tf2_ros import TransformBroadcaster

global people

def callback(msg):
    global people
    people = []
    q = quaternion_from_euler(0, 0, 0)
    curr = 0
    for person in msg.people:
        try:
            t = TransformStamped()
            t.header.frame_id = "robot_0/odom"
            frame_id = str("person"+str(curr))
            t.child_frame_id = frame_id

            t.transform.translation.x=person.pos.x
            t.transform.translation.y=person.pos.y
            t.transform.translation.z=person.pos.z

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            people.append(t)

        except IndexError:
            n = 1

        curr += 1

def talker():
    rospy.init_node('person_tf_publisher', anonymous=True)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
    br = TransformBroadcaster()
    rate = rospy.Rate(1)

    global people

    while not rospy.is_shutdown():
        try:
            for person in people:
                person.header.stamp = rospy.Time.now()
                br.sendTransform(person)
        except NameError:
            continue
        rate.sleep()
        

# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
