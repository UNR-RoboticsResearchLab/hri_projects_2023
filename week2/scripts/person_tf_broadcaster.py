#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped 
from people_msgs.msg import PositionMeasurementArray
from tf.transformations import quaternion_from_euler
import numpy as np
from tf2_ros import TransformBroadcaster

def callback(msg):
    br = TransformBroadcaster()
    q = quaternion_from_euler(0, 0, 0)
    for person in msg.people:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "person"

        t.transform.translation.x=person.pos.x
        t.transform.translation.y=person.pos.y
        t.transform.translation.z=person.pos.z

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
           

def talker():
    rospy.init_node('person_tf_publisher', anonymous=True)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
    
    rospy.spin()
# if __name__ == '__main__':
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
